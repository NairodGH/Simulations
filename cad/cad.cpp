#include "cad.hpp"

static Color colorForKind(SurfaceKind kind)
{
    switch (kind) {
    case SurfaceKind::Cylinder:
        return RED;
    case SurfaceKind::Torus:
        return GREEN;
    case SurfaceKind::Plane:
        return BLUE;
    default:
        return DARKGRAY;
    }
}

static const char* nameForKind(SurfaceKind kind)
{
    switch (kind) {
    case SurfaceKind::Plane:
        return "Plane";
    case SurfaceKind::Cylinder:
        return "Cylinder";
    case SurfaceKind::Torus:
        return "Torus";
    default:
        return "Unknown";
    }
}

CadModel::~CadModel()
{
    for (auto& mesh : meshes)
        UnloadMesh(mesh);
}

// returns the bbox center in original STEP space, used to undo the centering applied in drawCadModel
static Vector3 modelCenter(const CadModel& model)
{
    return { (model.bbox.min.x + model.bbox.max.x) * 0.5f, (model.bbox.min.y + model.bbox.max.y) * 0.5f, (model.bbox.min.z + model.bbox.max.z) * 0.5f };
}

#pragma region constraints
// distance: partner receives the same delta (rigid pair)
// symmetry: partner receives delta with its normal-axis component negated (mirror)
// both: axial component is already zero from clampDeltaForConstraints, so partner receives tangential only
// (if translate on z, normal is 0 0 1, if incomingDelta is 3 4 5, axisScalar is 5 (0 if both ^) and tangential is everything else so 3 4 0)
static Vector3 derivedDelta(const CadModel& model, const ConstraintPair& cp, Vector3 incomingDelta)
{
    if (cp.hasDistance && !cp.hasSymmetry)
        return incomingDelta;
    Vec3 normal = model.faceSurfaces[cp.faceA].axis.zDir.norm();
    double axialScalar = normal.x * incomingDelta.x + normal.y * incomingDelta.y + normal.z * incomingDelta.z;
    double axialmult = cp.hasDistance ? 0.0 : -1.0;
    return { incomingDelta.x + (float)((axialmult - 1.0) * axialScalar * normal.x), incomingDelta.y + (float)((axialmult - 1.0) * axialScalar * normal.y),
        incomingDelta.z + (float)((axialmult - 1.0) * axialScalar * normal.z) };
}

// DFS (Depth-first search, explores as far as possible along each branch before backtracking) over the constraint graph starting from
// rootFace (root face we moved) with rootDelta (how much we moved), returns the full set of (faceId, deltaToApply) pairs for every transitively reachable
// constrained face,
static std::vector<std::pair<int, Vector3>> collectConstrainedMoves(const CadModel& model, int rootFace, Vector3 rootDelta)
{
    std::vector<std::pair<int, Vector3>> queue = { { rootFace, rootDelta } }; // ofc face we moved is in the list
    std::vector<int> visited = { rootFace }; // and we already visited it
    int workId = 0; // cursor into queue, advances instead of popping so queue doubles as the result
    while (workId < (int)queue.size()) {
        auto [currentFace, currentDelta] = queue[workId++];
        for (const auto& constraintPair : model.constraints) {
            if (constraintPair.faceA != currentFace && constraintPair.faceB != currentFace)
                continue; // skip any string that doesn't involve the current face
            int otherFace = (constraintPair.faceA == currentFace) ? constraintPair.faceB : constraintPair.faceA;
            // skip already visited to avoid infinite loops in cyclic constraint graphs (faceA <-> faceB <-> faceC <-> faceA)
            if (std::find(visited.begin(), visited.end(), otherFace) != visited.end())
                continue;
            visited.push_back(otherFace);
            Vector3 otherDelta = derivedDelta(model, constraintPair, currentDelta);
            queue.push_back({ otherFace, otherDelta }); // continue BFS from this face
        }
    }
    queue.erase(queue.begin()); // remove rootFace, caller only wants the constrained partners
    return queue;
}

// clamps the axial component of delta to zero if any pair anywhere in the reachable constraint graph has
// both Distance and Symmetry active (because that combination fully locks the axis for the entire chain,
// ex: face3-D-face2-S+D-face1 -> moving face3 is still locked because the S+D pair is reachable from it)
// returns true if any clamping was applied so the caller can show a toast on the gesture rising edge
static bool clampDeltaForConstraints(const CadModel& model, int movingFace, Vector3& delta)
{
    // walk the full reachable constraint graph from movingFace (same BFS as collectConstrainedMoves)
    std::vector<int> queue = { movingFace };
    std::vector<int> visited = { movingFace };
    int workId = 0;
    while (workId < (int)queue.size()) {
        int currentFace = queue[workId++];
        for (const auto& cp : model.constraints) {
            if (cp.faceA != currentFace && cp.faceB != currentFace)
                continue;
            int otherFace = (cp.faceA == currentFace) ? cp.faceB : cp.faceA;
            if (std::find(visited.begin(), visited.end(), otherFace) != visited.end())
                continue;
            visited.push_back(otherFace);
            queue.push_back(otherFace);
        }
    }
    // check every pair whose both endpoints are in the reachable set for S+D
    bool clamped = false;
    for (const auto& cp : model.constraints) {
        if (!cp.hasDistance || !cp.hasSymmetry)
            continue;
        bool aReachable = std::find(visited.begin(), visited.end(), cp.faceA) != visited.end();
        bool bReachable = std::find(visited.begin(), visited.end(), cp.faceB) != visited.end();
        if (!aReachable || !bReachable)
            continue; // pair is not part of the reachable chain
        Vec3 normal = model.faceSurfaces[cp.faceA].axis.zDir.norm();
        double axialScalar = normal.x * delta.x + normal.y * delta.y + normal.z * delta.z;
        if (std::abs(axialScalar) < 1e-8)
            continue; // purely tangential move (ex: translation on x or y when both are aligned on z), nothing to clamp
        delta.x -= (float)(axialScalar * normal.x);
        delta.y -= (float)(axialScalar * normal.y);
        delta.z -= (float)(axialScalar * normal.z);
        clamped = true;
    }
    return clamped;
}

// applies the (already-clamped) delta transitively across the full constraint graph from movingFace, using DFS so constraint chains (face1->face2->face3) are
// all resolved in one call, each hop derives the correct delta for the next face via derivedDelta (follow for Distance, mirror for Symmetry), cylinder healing
// uses the per-face propagatedHealCaches map keyed by face index so each cylinder is healed exactly once with the correct delta for the face that caps it,
// regardless of how many constraint pairs involve that face
static void propagateConstraints(CadModel& model, int movingFace, Vector3 delta)
{
    for (auto& [otherFace, od] : collectConstrainedMoves(model, movingFace, delta)) {
        model.faceOffsets[otherFace].x += od.x;
        model.faceOffsets[otherFace].y += od.y;
        model.faceOffsets[otherFace].z += od.z;
        // look up the heal cache built specifically for otherFace at gesture start
        for (auto& [cacheId, cacheEntries] : model.propagatedHealCaches) {
            if (cacheId != otherFace)
                continue;
            Vec3 faceNormal = model.faceSurfaces[otherFace].axis.zDir.norm();
            applyCylinderHealCache(model, cacheEntries, faceNormal, od);
            break; // each face has exactly one entry in the map so dont bother continuing
        }
    }
}

// returns the centroid of a face (mean aka just add all vertices then divide result by number of vertices)
// in draw space (STEP-space vertices shifted by -modelCenter + faceOffset, beecause in drawCadModel we MatrixTranslate(-center.x, -center.y, -center.z)
// every mesh before drawing it so the model appears centered at the world origin regardless of where the STEP file placed it),
static Vector3 faceCentroid(const CadModel& model, int faceId)
{
    const TessellatedFace& face = model.pickData[faceId];
    Vector3 center = modelCenter(model);
    Vector3 off = model.faceOffsets[faceId];
    int frontVertCount = (int)face.vertices.size() / 6; // only front-face vertices
    if (frontVertCount == 0)
        return { 0.0f, 0.0f, 0.0f };
    Vector3 sum = { 0.0f, 0.0f, 0.0f };
    for (int i = 0; i < frontVertCount; i++) {
        int base = i * 3;
        sum.x += face.vertices[base] - center.x + off.x;
        sum.y += face.vertices[base + 1] - center.y + off.y;
        sum.z += face.vertices[base + 2] - center.z + off.z;
    }
    float inv = 1.0f / (float)frontVertCount;
    return { sum.x * inv, sum.y * inv, sum.z * inv };
}

// signed distance from origin (0 0 0) to centroid's closest point when going in normal direction
// ex: normal 0 0 1, centroid at 5 3 10, result is 10, if centroid at 5 3 -10 then -10 (can't get closer when following normal's path)
static float computeAxialPos(const CadModel& model, int faceId, int normalRefFaceId)
{
    Vec3 normal = model.faceSurfaces[normalRefFaceId].axis.zDir.norm();
    Vector3 centroid = faceCentroid(model, faceId);
    return (float)(normal.x * centroid.x + normal.y * centroid.y + normal.z * centroid.z);
}

// returns the 3D distance between two face centroids
static float computeCentroidDistance(const CadModel& model, int faceIdA, int faceIdB)
{
    Vector3 cA = faceCentroid(model, faceIdA);
    Vector3 cB = faceCentroid(model, faceIdB);
    float dx = cB.x - cA.x, dy = cB.y - cA.y, dz = cB.z - cA.z;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

#pragma region selection
// Möller–Trumbore ray-triangle intersection, imagine you're in a dark room holding a flashlight, here's a triangular picture frame on the wall
// somewhere in front of you, you want to know if your flashlight beam hits the framed canvas, and if so how far away it is,
// you can't just ask "does my beam cross the plane of the wall" since the wall is infinite, you need to know if the beam lands inside the triangle,
// so you describe any point inside the triangle as "start at corner A, walk a fraction toward B then a fraction toward C"
// if those two fractions are both positive and don't sum to more than 1, you're inside the triangle, this algo solves for exactly those two fractions
// and the distance t simultaneously, using the ray direction and the two triangle edges as the axes of a little local coordinate system,
// if the fractions are in bounds and t is positive (the triangle is in front of you, not behind), your flashlight hits the canvas at distance t
static float rayTriangleIntersect(Ray ray, Vec3 v0, Vec3 v1, Vec3 v2)
{
    const float epsilon = 1e-8f;
    Vec3 edge1 = v1 - v0, edge2 = v2 - v0;
    Vec3 rayDir = { ray.direction.x, ray.direction.y, ray.direction.z };
    Vec3 h = rayDir.cross(edge2);
    float det = (float)edge1.dot(h);
    // ray is parallel to the triangle plane (dot product of edge and perpendicular is ~0)
    if (std::abs(det) < epsilon)
        return -1.0f;
    float invDet = 1.0f / det;
    Vec3 rayOrig = { ray.position.x, ray.position.y, ray.position.z };
    Vec3 s = rayOrig - v0;
    // instead of describing a point's position as "X units right, Y units up from some origin," barycentric coordinates describe it as
    // "how much of each corner of a triangle contributes to this point", every point in or near a triangle gets three numbers (u, v, w) that sum to 1,
    // each number is the "weight" of the corresponding corner (like three magnets each pulling the point toward each other with a certain strength)
    // ex: centroid is (1/3, 1/3, 1/3), corner A is (1, 0, 0), midpoint of edge AB is (1/2, 1/2, 0), a point outside has at least one negative weight
    // u is the barycentric coordinate along edge1, must be in [0,1] to be inside the triangle
    float u = (float)s.dot(h) * invDet;
    if (u < 0.0f || u > 1.0f)
        return -1.0f;
    Vec3 q = s.cross(edge1);
    // v is the barycentric coordinate along edge2, u+v must also stay <= 1
    float v = (float)rayDir.dot(q) * invDet;
    if (v < 0.0f || u + v > 1.0f)
        return -1.0f;
    float t = (float)edge2.dot(q) * invDet;
    // t must be positive (hit is in front of the ray origin, not behind)
    return t > epsilon ? t : -1.0f;
}

// tests the ray against every triangle of every face and returns the index of the closest hit face, -1 if nothing hit
// adds bbox center back to the ray so it's in the same space as the vertex data (drawCadModel subtracts the center)
static int pickFace(const CadModel& model, Ray ray)
{
    Vector3 center = modelCenter(model);
    ray.position.x += center.x;
    ray.position.y += center.y;
    ray.position.z += center.z;

    float closestT = std::numeric_limits<float>::max();
    int hitFace = -1;
    // TODO: AABB pre-pass per face to cull before the triangle loop, brute-force is fine at current face counts but won't scale
    for (int faceId = 0; faceId < (int)model.pickData.size(); faceId++) {
        const auto& faceData = model.pickData[faceId];
        // offset is in draw space (post-centering), so add it to vertices in the same adjusted space
        // equivalent to subtracting it from the ray origin per face, but cheaper to apply once to a local ray copy
        Vector3 off = model.faceOffsets[faceId];
        Ray faceRay = ray;
        faceRay.position.x -= off.x;
        faceRay.position.y -= off.y;
        faceRay.position.z -= off.z;
        for (int i = 0; i + 2 < (int)faceData.indices.size(); i += 3) {
            // fetch the three vertex positions for this triangle from the flat XYZ array
            int i0 = faceData.indices[i] * 3, i1 = faceData.indices[i + 1] * 3, i2 = faceData.indices[i + 2] * 3;
            Vec3 v0 = { faceData.vertices[i0], faceData.vertices[i0 + 1], faceData.vertices[i0 + 2] };
            Vec3 v1 = { faceData.vertices[i1], faceData.vertices[i1 + 1], faceData.vertices[i1 + 2] };
            Vec3 v2 = { faceData.vertices[i2], faceData.vertices[i2 + 1], faceData.vertices[i2 + 2] };
            float t = rayTriangleIntersect(faceRay, v0, v1, v2);
            if (t > 0.0f && t < closestT) {
                closestT = t;
                hitFace = faceId;
            }
        }
    }
    return hitFace;
}

#pragma region draw scene
void drawCadModel(const CadModel& model)
{
    // translate so the model is centered at the origin before applying the caller's transform
    // this keeps orbit/zoom behavior symmetric regardless of where the STEP geometry is placed
    // ex: bbox min=(10,0,0), max=(20,0,0) -> center=(15,0,0), translate by (-15,0,0)
    Vector3 center
        = { (model.bbox.min.x + model.bbox.max.x) * 0.5f, (model.bbox.min.y + model.bbox.max.y) * 0.5f, (model.bbox.min.z + model.bbox.max.z) * 0.5f };
    Matrix centeredTransform = MatrixTranslate(-center.x, -center.y, -center.z);
    // every triangle has a front and a back determined by winding order, the GPU normally throws away (culls) triangles whose back is facing the camera,
    // this is an optimization since you never see the inside of a solid mesh, the problem here is that the tessellated faces have no consistent "outside",
    // a plane face could be seen from either side depending on viewing angle, and the normals are what carry the lighting information (not the winding)
    // so backface culling is disabled entirely and instead both a front and a back copy of every triangle are emitted with opposite normals
    // so lighting is correct from both sides (tldr paying with twice the triangles instead of relying on the GPU cull to handle it)
    rlDisableBackfaceCulling();
    for (int i = 0; i < (int)model.meshes.size(); i++) {
        Material material = LoadMaterialDefault();
        material.maps[MATERIAL_MAP_DIFFUSE].color = model.colors[i];
        // per-face offset (draw-space translation) stacked on top of the centering transform
        // ex: offset=(0,1,0) -> face shifts 1 unit upward in draw space after centering
        Vector3 off = model.faceOffsets[i];
        Matrix faceTransform = MatrixMultiply(MatrixTranslate(off.x, off.y, off.z), centeredTransform);
        DrawMesh(model.meshes[i], material, faceTransform);
        UnloadMaterial(material); // else will leak every frame for every mesh, not the most efficient but fine at this scale
    }
    rlEnableBackfaceCulling();
}

// redraws the selected face in solid white then black wireframe so it stands out from the scene
static void drawSelectedFaceHighlight(const CadModel& model)
{
    if (model.selectedFace < 0 || model.selectedFace >= (int)model.meshes.size())
        return;
    Vector3 center = modelCenter(model);
    Vector3 off = model.faceOffsets[model.selectedFace];
    Matrix centeredTransform = MatrixMultiply(MatrixTranslate(off.x, off.y, off.z), MatrixTranslate(-center.x, -center.y, -center.z));
    rlDisableBackfaceCulling();
    {
        Material solidMaterial = LoadMaterialDefault();
        solidMaterial.maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
        DrawMesh(model.meshes[model.selectedFace], solidMaterial, centeredTransform);
        UnloadMaterial(solidMaterial);

        rlEnableWireMode();
        {
            Material wireMaterial = LoadMaterialDefault();
            wireMaterial.maps[MATERIAL_MAP_DIFFUSE].color = BLACK;
            DrawMesh(model.meshes[model.selectedFace], wireMaterial, centeredTransform);
            UnloadMaterial(wireMaterial);
        }
        rlDisableWireMode();
    }
    rlEnableBackfaceCulling();
}

// redraws the distance-reference face (shift+right clicked) with a brightened tint (so we know we clicked well)
static void drawSecondFaceHighlight(const CadModel& model)
{
    Vector3 center = modelCenter(model);
    Vector3 off = model.faceOffsets[model.secondFace];
    Matrix centeredTransform = MatrixMultiply(MatrixTranslate(off.x, off.y, off.z), MatrixTranslate(-center.x, -center.y, -center.z));
    rlDisableBackfaceCulling();
    {
        Color base = model.colors[model.secondFace];
        Color brightened = { (unsigned char)std::min(255, (int)base.r + 80), (unsigned char)std::min(255, (int)base.g + 80),
            (unsigned char)std::min(255, (int)base.b + 80), 255 };
        Material solidMaterial = LoadMaterialDefault();
        solidMaterial.maps[MATERIAL_MAP_DIFFUSE].color = brightened;
        DrawMesh(model.meshes[model.secondFace], solidMaterial, centeredTransform);
        UnloadMaterial(solidMaterial);
    }
    rlEnableBackfaceCulling();
}

// computes the AABB of the face in draw space (with centering applied)
static BoundingBox computeFaceBBox(const TessellatedFace& face, Vector3 center, Vector3 offset = { 0.0f, 0.0f, 0.0f })
{
    BoundingBox bbox = { { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() },
        { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() } };
    int frontVertexCount = (int)face.vertices.size() / 6;
    for (int i = 0; i < frontVertexCount; i++) {
        int base = i * 3;
        float x = face.vertices[base] - center.x + offset.x;
        float y = face.vertices[base + 1] - center.y + offset.y;
        float z = face.vertices[base + 2] - center.z + offset.z;
        bbox.min.x = std::min(bbox.min.x, x);
        bbox.max.x = std::max(bbox.max.x, x);
        bbox.min.y = std::min(bbox.min.y, y);
        bbox.max.y = std::max(bbox.max.y, y);
        bbox.min.z = std::min(bbox.min.z, z);
        bbox.max.z = std::max(bbox.max.z, z);
    }
    return bbox;
}

// brute-force O(n*m) minimum vertex-to-vertex distance between two faces, approximates the true surface-to-surface distance,
// accurate enough for CAD face vertex densities (hundreds at most per face), adjacent faces sharing an edge will return 0 (shared vertex)
// could use our vec3.near but would be slightly worse because would calls sqrt per pair when all we need is to compare distance
// not actually get them (so we dont need to sqrt)
static float computeFaceMinDistance(
    const TessellatedFace& faceA, const TessellatedFace& faceB, Vector3 offsetA = { 0.0f, 0.0f, 0.0f }, Vector3 offsetB = { 0.0f, 0.0f, 0.0f })
{
    float minDistSq = std::numeric_limits<float>::max();
    int countA = (int)faceA.vertices.size() / 6; // front vertices only
    int countB = (int)faceB.vertices.size() / 6;
    for (int i = 0; i < countA; i++) {
        int baseA = i * 3;
        float ax = faceA.vertices[baseA] + offsetA.x;
        float ay = faceA.vertices[baseA + 1] + offsetA.y;
        float az = faceA.vertices[baseA + 2] + offsetA.z;
        for (int j = 0; j < countB; j++) {
            int baseB = j * 3;
            float dx = ax - (faceB.vertices[baseB] + offsetB.x);
            float dy = ay - (faceB.vertices[baseB + 1] + offsetB.y);
            float dz = az - (faceB.vertices[baseB + 2] + offsetB.z);
            // the sphere (your current best distance) fits inside a cube whose side equals the diameter, if you're already outside the cube on any single wall
            // then you're outside the sphere, checking one wall costs one abs
            // (which is cheaper than even a multiplication because it's just zeroing one bit of the float)
            float eps = sqrtf(minDistSq);
            if (std::abs(dx) >= eps || std::abs(dy) >= eps || std::abs(dz) >= eps)
                continue;
            float dSq = dx * dx + dy * dy + dz * dz;
            if (dSq < minDistSq)
                minDistSq = dSq;
        }
    }
    return sqrtf(minDistSq);
}

// computes the AABB of all faces in draw space (vertices minus center plus their current offsets),
// used for both the live model bbox display and the per-face bbox display
// pass center={0,0,0} and a single face's offset to get a per-face bbox, or modelCenter() and per-face offsets for the full model
static BoundingBox computeModelBBox(const CadModel& model, Vector3 center)
{
    BoundingBox bbox = { { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() },
        { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() } };
    for (int fi = 0; fi < (int)model.pickData.size(); fi++) {
        const auto& face = model.pickData[fi];
        Vector3 off = model.faceOffsets[fi];
        int frontVertexCount = (int)face.vertices.size() / 6;
        for (int i = 0; i < frontVertexCount; i++) {
            int base = i * 3;
            float x = face.vertices[base] - center.x + off.x;
            float y = face.vertices[base + 1] - center.y + off.y;
            float z = face.vertices[base + 2] - center.z + off.z;
            bbox.min.x = std::min(bbox.min.x, x);
            bbox.max.x = std::max(bbox.max.x, x);
            bbox.min.y = std::min(bbox.min.y, y);
            bbox.max.y = std::max(bbox.max.y, y);
            bbox.min.z = std::min(bbox.min.z, z);
            bbox.max.z = std::max(bbox.max.z, z);
        }
    }
    return bbox;
}

// draws the whole model bounding box in gold, recomputed each call to reflect current face offsets
static void drawModelBbox(const CadModel& model) { DrawBoundingBox(computeModelBBox(model, modelCenter(model)), GOLD); }

// draws the bounding box of the selected face in orange, for planes this will be a flat rectangle, for cylinders/toruses a proper 3D box
static void drawFaceBbox(const CadModel& model)
{
    if (model.selectedFace < 0 || model.selectedFace >= (int)model.pickData.size())
        return;
    DrawBoundingBox(computeFaceBBox(model.pickData[model.selectedFace], modelCenter(model), model.faceOffsets[model.selectedFace]), ORANGE);
}

// draws small yellow outward normal arrows per triangle of the selected face (front-face triangles only, strided to avoid clutter)
// normalLength should be scaled to the model size, ex: diagonal * 0.03
static void drawFaceNormals(const CadModel& model, float normalLength)
{
    if (model.selectedFace < 0 || model.selectedFace >= (int)model.pickData.size())
        return;
    Vector3 center = modelCenter(model);
    Vector3 off = model.faceOffsets[model.selectedFace];
    const auto& faceData = model.pickData[model.selectedFace];
    // tessGrid emits front vertices first then back vertices (same count), so front indices are all < frontVertexCount
    // we only draw normals for front-face triangles to avoid double arrows pointing in opposite directions
    int frontVertexCount = (int)faceData.vertices.size() / 3 / 2;
    // skip every 3 triangles (6 indices * 3 = 18) to keep the display readable on dense meshes
    for (int i = 0; i + 2 < (int)faceData.indices.size(); i += 18) {
        if (faceData.indices[i] >= frontVertexCount || faceData.indices[i + 1] >= frontVertexCount || faceData.indices[i + 2] >= frontVertexCount)
            continue;
        int i0 = faceData.indices[i] * 3, i1 = faceData.indices[i + 1] * 3, i2 = faceData.indices[i + 2] * 3;
        // triangle centroid in draw space (apply same centering as drawCadModel, plus the face's offset)
        Vector3 triCenter = { (faceData.vertices[i0] + faceData.vertices[i1] + faceData.vertices[i2]) / 3.0f - center.x + off.x,
            (faceData.vertices[i0 + 1] + faceData.vertices[i1 + 1] + faceData.vertices[i2 + 1]) / 3.0f - center.y + off.y,
            (faceData.vertices[i0 + 2] + faceData.vertices[i1 + 2] + faceData.vertices[i2 + 2]) / 3.0f - center.z + off.z };
        // average normal of the three vertices (analytical normals so all three are essentially identical, average is just for correctness)
        Vector3 triNormal = { (faceData.normals[i0] + faceData.normals[i1] + faceData.normals[i2]) / 3.0f,
            (faceData.normals[i0 + 1] + faceData.normals[i1 + 1] + faceData.normals[i2 + 1]) / 3.0f,
            (faceData.normals[i0 + 2] + faceData.normals[i1 + 2] + faceData.normals[i2 + 2]) / 3.0f };
        Vector3 arrowTip = { triCenter.x + triNormal.x * normalLength, triCenter.y + triNormal.y * normalLength, triCenter.z + triNormal.z * normalLength };
        DrawLine3D(triCenter, arrowTip, YELLOW);
    }
}

// draws a single large SKYBLUE arrow/line representing the analytical surface definition of the selected face:
// plane -> normal arrow from face centroid, cylinder/torus -> axis line through the surface origin
// drawn at 5x the triangle arrow length to visually distinguish it from the yellow per-triangle arrows
static void drawFaceAnalyticalAxis(const CadModel& model, float scale)
{
    if (model.selectedFace < 0 || model.selectedFace >= (int)model.faceSurfaces.size())
        return;
    Vector3 center = modelCenter(model);
    Vector3 offset = model.faceOffsets[model.selectedFace];
    const Surface& surface = model.faceSurfaces[model.selectedFace];

    switch (surface.kind) {
    case SurfaceKind::Plane: {
        // one big normal arrow from the face centroid along zDir which is the average position of all front-face vertices in draw space
        // (with centering already applied), used to anchor axis/normal arrows at the visual center of the face
        const TessellatedFace& face = model.pickData[model.selectedFace];
        int frontVertexCount = (int)face.vertices.size() / 6; // front vertices occupy the 1rst half of the flat array (2nd half is the back-face duplicates)
        Vector3 faceCentroid = { 0, 0, 0 };
        for (int i = 0; i < frontVertexCount; i++) {
            int base = i * 3;
            faceCentroid.x += face.vertices[base] - center.x + offset.x;
            faceCentroid.y += face.vertices[base + 1] - center.y + offset.y;
            faceCentroid.z += face.vertices[base + 2] - center.z + offset.z;
        }
        if (frontVertexCount > 0) {
            float inv = 1.0f / (float)frontVertexCount;
            faceCentroid.x *= inv;
            faceCentroid.y *= inv;
            faceCentroid.z *= inv;
        }
        Vec3 normal = surface.axis.zDir.norm();
        Vector3 tip = { faceCentroid.x + (float)normal.x * scale, faceCentroid.y + (float)normal.y * scale, faceCentroid.z + (float)normal.z * scale };
        DrawLine3D(faceCentroid, tip, SKYBLUE);
        break;
    }
    case SurfaceKind::Cylinder:
    case SurfaceKind::Torus: {
        // axis line extending in both directions from the surface origin (which is a point on the axis)
        // surface origin is in STEP space, apply centering then offset to get draw space
        Vector3 axisOrigin = { (float)surface.axis.origin.x - center.x + offset.x, (float)surface.axis.origin.y - center.y + offset.y,
            (float)surface.axis.origin.z - center.z + offset.z };
        Vec3 axisDir = surface.axis.zDir.norm();
        Vector3 axisStart = { axisOrigin.x - (float)axisDir.x * scale, axisOrigin.y - (float)axisDir.y * scale, axisOrigin.z - (float)axisDir.z * scale };
        Vector3 axisEnd = { axisOrigin.x + (float)axisDir.x * scale, axisOrigin.y + (float)axisDir.y * scale, axisOrigin.z + (float)axisDir.z * scale };
        DrawLine3D(axisStart, axisEnd, SKYBLUE);
        break;
    }
    default:
        break;
    }
}

// draws the area-weighted average orientation of all faces as a SKYBLUE arrow from the live model center
// each face contributes its zDir (plane normal / cylinder+torus axis) weighted by its tessellated area
static void drawModelAverageNormal(const CadModel& model, float scale)
{
    Vec3 weightedSum = { 0, 0, 0 };
    for (int i = 0; i < (int)model.faceSurfaces.size(); i++) {
        Vec3 faceAxis = model.faceSurfaces[i].axis.zDir.norm();
        float weight = (i < (int)model.faceAreas.size()) ? model.faceAreas[i] : 1.0f;
        weightedSum = weightedSum + faceAxis * weight;
    }
    Vec3 avgNormal = weightedSum.norm();
    // anchor at the live bbox center in draw space (live center minus the static centering offset)
    // as faces are push/pulled the live center drifts, keeping the arrow visually attached to the actual model centroid
    Vector3 staticCenter = modelCenter(model);
    BoundingBox liveBbox = computeModelBBox(model, staticCenter);
    Vector3 origin3D = { (liveBbox.min.x + liveBbox.max.x) * 0.5f, (liveBbox.min.y + liveBbox.max.y) * 0.5f, (liveBbox.min.z + liveBbox.max.z) * 0.5f };
    Vector3 tip = { origin3D.x + (float)avgNormal.x * scale, origin3D.y + (float)avgNormal.y * scale, origin3D.z + (float)avgNormal.z * scale };
    DrawLine3D(origin3D, tip, SKYBLUE);
}

#pragma region handleControls
// global stuff like camera, N, B, select, constraint panel click, toast decay...
static void handleDefaultControls(
    CadModel& model, Camera3D& camera, float& yaw, float& pitch, float& orbitRadius, bool& showNormals, bool& showBbox, float diagonal)
{
    // left mouse drag -> orbit
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 mouseDelta = GetMouseDelta();
        yaw += mouseDelta.x * 0.4f; // 0.4 deg/pixel feel-tuned for typical screen DPI
        pitch -= mouseDelta.y * 0.4f; // subtract because screen Y is flipped relative to world Y
        // clamp with 1 degree margin off 90° so that view matrix doesn't degenerate if we go directly above/below target
        if (pitch > 89.0f)
            pitch = 89.0f;
        if (pitch < -89.0f)
            pitch = -89.0f;
    }
    // scroll wheel -> zoom
    float scrollDelta = GetMouseWheelMove();
    orbitRadius -= scrollDelta * diagonal * 0.1f; // 10% of diagonal per scroll tick
    if (orbitRadius < diagonal * 0.1f)
        orbitRadius = diagonal * 0.1f;
    if (orbitRadius > diagonal * 10.f)
        orbitRadius = diagonal * 10.f;
    // convert spherical (yaw, pitch, orbitRadius) to Cartesian camera position (x y z) that raylib wants
    // ex: yaw=90deg, pitch=0 -> camera sits on the +X axis looking toward origin
    float yawRadians = DEG2RAD * yaw, pitchRadians = DEG2RAD * pitch;
    camera.position = { orbitRadius * std::cos(pitchRadians) * std::sin(yawRadians), orbitRadius * std::sin(pitchRadians),
        orbitRadius * std::cos(pitchRadians) * std::cos(yawRadians) };

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        int hitFace = pickFace(model, GetMouseRay(GetMousePosition(), camera));
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
            if (model.selectedFace > -1)
                model.secondFace = (hitFace > -1 && hitFace != model.selectedFace) ? hitFace : -1;
        } else {
            model.selectedFace = hitFace;
            model.secondFace = -1;
        }
    }

    // constraint panel, reset button at top reloads the scene, entries below remove that pair, see drawConstraintPanel
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        Vector2 mouse = GetMousePosition();
        int panelX = GetScreenWidth() - 240; // 220 panelWidth + 20 right offset
        const int resetButtonH = 24;
        const int entryH = 26, entryGap = 3;
        Rectangle resetRect = { (float)panelX, 20.0f, 220.0f, (float)resetButtonH };
        if (CheckCollisionPointRec(mouse, resetRect)) {
            model.needsReset = true;
        } else {
            int entryY = 20 + resetButtonH + 4 + 20; // top margin + reset button + gap + header height
            for (int i = 0; i < (int)model.constraints.size(); i++) {
                Rectangle entryRect = { (float)panelX, (float)entryY, 220.0f, (float)entryH };
                if (CheckCollisionPointRec(mouse, entryRect)) {
                    model.constraints.erase(model.constraints.begin() + i);
                    break;
                }
                entryY += entryH + entryGap;
            }
        }
    }

    if (IsKeyPressed(KEY_N))
        showNormals = !showNormals;
    if (IsKeyPressed(KEY_B))
        showBbox = !showBbox;

    // decay active toasts, subtract frame time each frame and remove expired entries
    float dt = GetFrameTime();
    model.toasts.erase(std::remove_if(model.toasts.begin(), model.toasts.end(),
                           [&](Toast& t) {
                               t.timeLeft -= dt;
                               return t.timeLeft <= 0.0f;
                           }),
        model.toasts.end());
}

// translation, undo/redo
static void handle1SelectControls(CadModel& model, float diagonal)
{
    // move selected face along its surface's own axes
    // UP/DOWN = zDir (normal for planes, axis for cylinders/toruses), LEFT/RIGHT = xDir (in-plane tangent), PageUp/PageDown = yDir (zDir*xDir)
    // wasTranslating detects the rising edge of a translation gesture so one continuous hold = one undo entry
    model.translating = false;
    if (model.selectedFace > -1) {
        float step = diagonal * 0.005f;
        const Surface& surf = model.faceSurfaces[model.selectedFace];
        Vec3 zDir = surf.axis.zDir.norm();
        Vec3 xDir = surf.axis.xDir.norm();
        Vec3 yDir = zDir.cross(xDir).norm();
        Vector3& off = model.faceOffsets[model.selectedFace];
        model.translating
            = IsKeyDown(KEY_UP) || IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_PAGE_UP) || IsKeyDown(KEY_PAGE_DOWN);
        // if we just started pressing down a translation key
        if (model.translating && !model.wasTranslating) {
            // always rebuild at every gesture start so connectivity is re-evaluated at the plane's current position
            model.healCache = buildCylinderHealCache(model, model.selectedFace);
            model.cachedHealFace = model.selectedFace;
            // collect the set of faces down the constraints chains that will move along with the current selected face
            Vec3 zDirSeed = model.faceSurfaces[model.selectedFace].axis.zDir.norm();
            Vector3 unitSeed = { (float)zDirSeed.x, (float)zDirSeed.y, (float)zDirSeed.z };
            auto constrainedMoves = collectConstrainedMoves(model, model.selectedFace, unitSeed);
            // build one heal cache per partner face, keyed by face index
            model.propagatedHealCaches.clear();
            for (const auto& [partnerFace, _] : constrainedMoves)
                model.propagatedHealCaches.push_back({ partnerFace, buildCylinderHealCache(model, partnerFace) });
            // snapshot the pre-gesture offsets of the moving face and the ones of all faces down the constraints chains
            UndoEntry entry;
            entry.faceId = model.selectedFace;
            entry.oldOffset = off;
            for (const auto& [partnerFace, _] : constrainedMoves)
                entry.propagatedOffsets.push_back({ partnerFace, model.faceOffsets[partnerFace] });
            // cylinder snapshot covering all faces that will move (selected + all faces down the constraints chains)
            std::vector<int> movingFaces = { model.selectedFace };
            for (const auto& po : entry.propagatedOffsets)
                movingFaces.push_back(po.first);
            for (int mf : movingFaces) {
                for (const auto& healEntry : buildCylinderHealCache(model, mf)) {
                    bool already = false;
                    for (const auto& snap : entry.cylSnapshots)
                        if (snap.cylFaceId == healEntry.cylFaceId) {
                            already = true;
                            break;
                        }
                    if (!already)
                        entry.cylSnapshots.push_back({ healEntry.cylFaceId, model.cylHeightRanges[healEntry.cylFaceId] });
                }
            }
            model.undoStack.push_back(std::move(entry));
            model.redoStack.clear();
        }
        // clear caches if the selected face changed between gestures
        if (model.cachedHealFace != model.selectedFace) {
            model.healCache.clear();
            model.cachedHealFace = -1;
            model.propagatedHealCaches.clear();
        }
        model.wasTranslating = model.translating;
        Vec3 planeNormal = surf.axis.zDir.norm();
        if (!model.translating)
            model.toastPushedThisGesture = false; // show only error toast on translation key press (not on every translation frame xd)
        auto move = [&](int key, Vec3 dir) {
            if (!IsKeyDown(key))
                return;
            Vector3 delta = { (float)dir.x * step, (float)dir.y * step, (float)dir.z * step };
            // clamp axial component to zero for any fully-locked pair (Distance+Symmetry both active)
            bool clamped = clampDeltaForConstraints(model, model.selectedFace, delta);
            if (clamped && !model.toastPushedThisGesture) {
                model.toasts.push_back({ "Axial motion blocked: Distance+Symmetry lock this axis", 2.5f });
                model.toastPushedThisGesture = true;
            }
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            // collect constrained moves once so we can reuse the derived deltas below without a second DFS
            auto constrainedMoves = collectConstrainedMoves(model, model.selectedFace, delta);
            std::unordered_map<int, CylFrameSnapshot> preFrame;
            for (const auto& pe : model.healCache)
                preFrame.emplace(pe.cylFaceId, CylFrameSnapshot { model.cylHeightRanges[pe.cylFaceId], pe.isMaxCap });
            for (const auto& [cacheId, cacheEntries] : model.propagatedHealCaches)
                for (const auto& qe : cacheEntries)
                    preFrame.emplace(qe.cylFaceId, CylFrameSnapshot { model.cylHeightRanges[qe.cylFaceId], qe.isMaxCap });

            double primaryAxialDelta = planeNormal.x * delta.x + planeNormal.y * delta.y + planeNormal.z * delta.z;

            std::unordered_map<int, Vector3> propOds;
            std::unordered_map<int, double> propAxialDeltas;
            for (const auto& [cacheId, cacheEntries] : model.propagatedHealCaches) {
                Vector3 od = { 0, 0, 0 };
                for (const auto& [face, faceDelta] : constrainedMoves)
                    if (face == cacheId) {
                        od = faceDelta;
                        break;
                    }
                propOds[cacheId] = od;
                Vec3 partnerNormal = model.faceSurfaces[cacheId].axis.zDir.norm();
                propAxialDeltas[cacheId] = partnerNormal.x * od.x + partnerNormal.y * od.y + partnerNormal.z * od.z;
            }

            for (auto& [cylId, snap] : preFrame) {
                double newMin = snap.range.heightMin;
                double newMax = snap.range.heightMax;
                // primary cache contribution (at most one entry per cylinder)
                for (auto& pe : model.healCache) {
                    if (pe.cylFaceId != cylId)
                        continue;
                    double signedDelta = primaryAxialDelta * pe.axisDotNormal;
                    if (pe.isMaxCap)
                        newMax += signedDelta;
                    else
                        newMin += signedDelta;
                    break;
                }
                for (auto& [cacheId, cacheEntries] : model.propagatedHealCaches) {
                    double pad = propAxialDeltas.count(cacheId) ? propAxialDeltas.at(cacheId) : 0.0;
                    for (auto& qe : cacheEntries) {
                        if (qe.cylFaceId != cylId)
                            continue;
                        double signedDelta = pad * qe.axisDotNormal;
                        if (qe.isMaxCap)
                            newMax += signedDelta;
                        else
                            newMin += signedDelta;
                        break; // each face owns at most one cap of a given cylinder
                    }
                }
                if (newMax <= newMin + 1e-6) {
                    std::swap(newMin, newMax);
                    for (auto& pe : model.healCache)
                        if (pe.cylFaceId == cylId) {
                            pe.isMaxCap = !pe.isMaxCap;
                            break;
                        }
                    for (auto& [cacheId, cacheEntries] : model.propagatedHealCaches)
                        for (auto& qe : cacheEntries)
                            if (qe.cylFaceId == cylId) {
                                qe.isMaxCap = !qe.isMaxCap;
                                break;
                            }
                }
                retessCylinderFace(model, cylId, newMin, newMax);
            }

            for (auto& [cacheId, cacheEntries] : model.propagatedHealCaches) {
                const Vector3& od = propOds.count(cacheId) ? propOds.at(cacheId) : Vector3 { 0, 0, 0 };
                model.faceOffsets[cacheId].x += od.x;
                model.faceOffsets[cacheId].y += od.y;
                model.faceOffsets[cacheId].z += od.z;
            }
        };
        move(KEY_UP, zDir);
        move(KEY_DOWN, zDir * -1.0);
        move(KEY_RIGHT, xDir);
        move(KEY_LEFT, xDir * -1.0);
        move(KEY_PAGE_UP, yDir);
        move(KEY_PAGE_DOWN, yDir * -1.0);
    } else {
        // user clicked outside model, unselecting everything so clear all
        model.wasTranslating = false;
        model.healCache.clear();
        model.cachedHealFace = -1;
        model.propagatedHealCaches.clear();
    }

    // same principle for both: pop from source stack, capture current state into the opposite stack, push to destination stack, then restore
    auto applyUndoRedoStep = [&](std::vector<UndoEntry>& source, std::vector<UndoEntry>& destination) {
        if (source.empty())
            return;
        UndoEntry entry = source.back();
        source.pop_back();
        UndoEntry opposite;
        opposite.faceId = entry.faceId;
        opposite.oldOffset = model.faceOffsets[entry.faceId];
        for (const auto& snap : entry.cylSnapshots)
            opposite.cylSnapshots.push_back({ snap.cylFaceId, model.cylHeightRanges[snap.cylFaceId] });
        // capture current propagated offsets into the opposite stack before restoring, so redo can reverse the undo
        for (const auto& [faceId, _] : entry.propagatedOffsets)
            opposite.propagatedOffsets.push_back({ faceId, model.faceOffsets[faceId] });
        destination.push_back(std::move(opposite));
        model.faceOffsets[entry.faceId] = entry.oldOffset;
        for (const auto& snap : entry.cylSnapshots) {
            model.cylHeightRanges[snap.cylFaceId] = snap.oldHeight;
            retessCylinderFace(model, snap.cylFaceId, snap.oldHeight.heightMin, snap.oldHeight.heightMax);
        }
        // restore the offsets of faces down the constraints chains that got moved by propagation during this gesture
        for (const auto& [faceId, oldOffset] : entry.propagatedOffsets)
            model.faceOffsets[faceId] = oldOffset;
        model.selectedFace = entry.faceId;
    };
    // both blocked while any translation key is held to prevent messing with an in-progress gesture
    // (ctrl+z is swallowed by the Win32 message loop before Raylib sees the key event XDD, so use ctrl+u instead...)
    if (!model.translating && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_U))
        applyUndoRedoStep(model.undoStack, model.redoStack);
    if (!model.translating && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_Y))
        applyUndoRedoStep(model.redoStack, model.undoStack);
}

// constraints
static void handle2SelectControls(CadModel& model)
{
    if (!model.translating && model.selectedFace > -1 && model.secondFace > -1) {
        int fA = std::min(model.selectedFace, model.secondFace); // always fA < fB so pair lookup is a simple/fast equality check
        int fB = std::max(model.selectedFace, model.secondFace);
        auto pushToast = [&](const std::string& msg) { model.toasts.push_back({ msg, 3.0f }); };
        auto findPairId = [&]() -> int { // returns the index of the existing pair for fA-fB or -1 if none
            for (int i = 0; i < (int)model.constraints.size(); i++)
                if (model.constraints[i].faceA == fA && model.constraints[i].faceB == fB)
                    return i;
            return -1;
        };
        // handle constraint related input
        auto toggleKind = [&](ConstraintKind kind) {
            int id = findPairId();
            ConstraintPair* constraintPair = (id >= 0) ? &model.constraints[id] : nullptr;
            bool active = constraintPair
                && ((kind == ConstraintKind::Distance && constraintPair->hasDistance) || (kind == ConstraintKind::Symmetry && constraintPair->hasSymmetry));
            if (active) {
                // toggle off (clear the kind, remove the entry entirely if no kinds remain)
                if (kind == ConstraintKind::Distance)
                    constraintPair->hasDistance = false;
                else
                    constraintPair->hasSymmetry = false;
                if (!constraintPair->hasDistance && !constraintPair->hasSymmetry)
                    model.constraints.erase(model.constraints.begin() + id);
                return;
            }
            if (!constraintPair && (int)model.constraints.size() >= 20) {
                pushToast("Constraint limit reached (max 20 pairs)");
                return;
            }
            // create the pair entry if it does not exist yet
            if (!constraintPair) {
                ConstraintPair newPair;
                newPair.faceA = fA;
                newPair.faceB = fB;
                model.constraints.push_back(newPair);
                constraintPair = &model.constraints.back();
            }
            if (kind == ConstraintKind::Distance) {
                constraintPair->hasDistance = true;
                constraintPair->targetDistance = computeCentroidDistance(model, fA, fB);
            } else {
                constraintPair->hasSymmetry = true;
                float posA = computeAxialPos(model, fA, fA);
                float posB = computeAxialPos(model, fB, fA);
                constraintPair->symmetryMidpoint = (posA + posB) * 0.5f;
            }
        };
        if (IsKeyPressed(KEY_D))
            toggleKind(ConstraintKind::Distance);
        if (IsKeyPressed(KEY_S))
            toggleKind(ConstraintKind::Symmetry);
    }
}

static void handleControls(CadModel& model, Camera3D& camera, float& yaw, float& pitch, float& orbitRadius, bool& showNormals, bool& showBbox, float diagonal)
{
    handleDefaultControls(model, camera, yaw, pitch, orbitRadius, showNormals, showBbox, diagonal);
    handle1SelectControls(model, diagonal);
    handle2SelectControls(model);
}

#pragma region draw UI
// top-right panel listing all active face-pair constraints as clickable buttons
// clicking any entry removes all constraints for that pair, hover is highlighted
// geometry constants must stay in sync with the panel click detection in handleControls
static void drawConstraintPanel(const CadModel& model)
{
    const int panelRight = GetScreenWidth() - 20;
    const int panelWidth = 220;
    const int panelX = panelRight - panelWidth; // = screenW - 240
    const int resetButtonH = 24;
    const int entryH = 26, entryGap = 3;
    const Color paleOrange = { 220, 170, 100, 255 };
    int panelY = 20;

    // reset button, full panel width, sits above the header
    Vector2 mouse = GetMousePosition();
    Rectangle resetRect = { (float)panelX, (float)panelY, (float)panelWidth, (float)resetButtonH };
    bool resetHovered = CheckCollisionPointRec(mouse, resetRect);
    DrawRectangleRec(resetRect, resetHovered ? Color { 90, 40, 40, 220 } : Color { 55, 28, 28, 200 });
    DrawText("Reset scene", panelX + panelWidth / 2 - MeasureText("Reset scene", 14) / 2, panelY + 5, 14, Color { 220, 140, 140, 255 });
    panelY += resetButtonH + 4;

    DrawText(TextFormat("Constraints (%d/20)", (int)model.constraints.size()), panelX, panelY, 16, paleOrange);
    panelY += 20; // header height, keep in sync with handleControls click detection

    for (int i = 0; i < (int)model.constraints.size(); i++) {
        const ConstraintPair& cp = model.constraints[i];
        Rectangle entryRect = { (float)panelX, (float)panelY, (float)panelWidth, (float)entryH };
        bool hovered = CheckCollisionPointRec(mouse, entryRect);
        Color bgColor = hovered ? Color { 55, 55, 68, 210 } : Color { 32, 32, 44, 190 };
        DrawRectangleRec(entryRect, bgColor);

        int cx = panelX + 6;
        int cy = panelY + 6;
        // face A id
        const char* faceAStr = TextFormat("#%d", cp.faceA);
        DrawText(faceAStr, cx, cy, 14, LIGHTGRAY);
        cx += MeasureText(faceAStr, 14) + 5;
        // kind icons, 13x14 colored squares with a single letter, stacked horizontally
        if (cp.hasDistance) {
            DrawRectangle(cx, cy, 13, 14, ORANGE);
            DrawText("D", cx + 2, cy + 1, 12, BLACK);
            cx += 17;
        }
        if (cp.hasSymmetry) {
            DrawRectangle(cx, cy, 13, 14, SKYBLUE);
            DrawText("S", cx + 2, cy + 1, 12, BLACK);
            cx += 17;
        }
        // face B id
        DrawText(TextFormat("#%d", cp.faceB), cx + 4, cy, 14, LIGHTGRAY);

        panelY += entryH + entryGap;
    }
}

// fading toast notifications rendered at the bottom-center of the screen,
// alpha ramps from 1 down to 0 over the final 0.5s of each toast's 3s lifetime
static void drawToasts(const CadModel& model)
{
    int screenW = GetScreenWidth();
    int toastY = GetScreenHeight() - 54;
    for (const auto& t : model.toasts) {
        float fading = t.timeLeft < 0.5f ? t.timeLeft / 0.5f : 1.0f;
        unsigned char alpha = (unsigned char)(fading * 230);
        int textW = MeasureText(t.message.c_str(), 16);
        int toastX = (screenW - textW) / 2 - 10;
        DrawRectangle(toastX, toastY - 5, textW + 20, 28, Color { 30, 30, 42, alpha });
        DrawText(t.message.c_str(), toastX + 10, toastY, 16, Color { 255, 220, 80, alpha });
        toastY -= 36; // stack upward when multiple toasts are active
    }
}

static void drawUI(const CadModel& model)
{
    // stats panel with running Y so adding lines never requires renumbering
    // four blocks separated by a small gap, each block has its own color tier
    // selectionless = pale lime, 1st selection = pale yellow, 2nd selection = pale orange
    int uiY = 20;
    const int uiStep = 20;
    const int blockGap = 10;
    const Color paleLime = { 160, 210, 130, 255 };
    const Color paleYellow = { 220, 210, 130, 255 };
    const Color paleOrange = { 220, 170, 100, 255 };

    // block 1
    DrawText("RED = Cylinders", 20, uiY, 16, RED);
    uiY += uiStep;
    DrawText("GREEN = toruses (fillets)", 20, uiY, 16, GREEN);
    uiY += uiStep;
    DrawText("BLUE = Planes", 20, uiY, 16, BLUE);
    uiY += uiStep;
    DrawText("DARKGRAY = Other", 20, uiY, 16, DARKGRAY);
    uiY += uiStep + blockGap;

    // block 2
    DrawText(TextFormat("Faces: %d\nTriangles: %d", (int)model.meshes.size(), model.totalTriangleCount), 20, uiY, 16, paleLime);
    uiY += uiStep * 2; // two lines because of the \n

    Vector3 mCenter = modelCenter(model);
    BoundingBox liveBbox = computeModelBBox(model, mCenter);
    DrawText(TextFormat("Width: %.2f\nHeight: %.2f\nDepth: %.2f mm", liveBbox.max.x - liveBbox.min.x, liveBbox.max.y - liveBbox.min.y,
                 liveBbox.max.z - liveBbox.min.z),
        20, uiY, 16, paleLime);
    uiY += uiStep * 3;
    DrawText(TextFormat("Position: %.2f %.2f %.2f", (liveBbox.min.x + liveBbox.max.x) * 0.5f, (liveBbox.min.y + liveBbox.max.y) * 0.5f,
                 (liveBbox.min.z + liveBbox.max.z) * 0.5f),
        20, uiY, 16, paleLime);
    uiY += uiStep;
    DrawText("Drag = orbit", 20, uiY, 16, paleLime);
    uiY += uiStep;
    DrawText("Scroll = zoom", 20, uiY, 16, paleLime);
    uiY += uiStep;
    DrawText("N = normals", 20, uiY, 16, paleLime);
    uiY += uiStep;
    DrawText("B = bbox", 20, uiY, 16, paleLime);
    uiY += uiStep;
    DrawText("RClick = select face", 20, uiY, 16, paleLime);
    uiY += uiStep + blockGap;

    // block 3
    if (model.selectedFace > -1) {
        int faceId = model.selectedFace;
        int frontTris = (int)model.pickData[faceId].indices.size() / 6;
        DrawText(TextFormat(
                     "Face #%d [%s]\nTriangles: %d\nSurface: %.2f mm^2", faceId, nameForKind(model.pickData[faceId].kind), frontTris, model.faceAreas[faceId]),
            20, uiY, 16, paleYellow);
        uiY += uiStep * 3;

        Vector3 center = modelCenter(model);
        BoundingBox faceBbox = computeFaceBBox(model.pickData[faceId], center, model.faceOffsets[faceId]);
        DrawText(TextFormat("Width: %.2f\nHeight: %.2f\nDepth: %.2f mm", faceBbox.max.x - faceBbox.min.x, faceBbox.max.y - faceBbox.min.y,
                     faceBbox.max.z - faceBbox.min.z),
            20, uiY, 16, paleYellow);
        uiY += uiStep * 3;
        DrawText(TextFormat("Position: %.2f, %.2f, %.2f", (faceBbox.min.x + faceBbox.max.x) * 0.5f, (faceBbox.min.y + faceBbox.max.y) * 0.5f,
                     (faceBbox.min.z + faceBbox.max.z) * 0.5f),
            20, uiY, 16, paleYellow);
        uiY += uiStep;

        DrawText("Arrows/page keys = translate", 20, uiY, 16, paleYellow);
        uiY += uiStep;
        DrawText("Ctrl+U/Y = undo/redo", 20, uiY, 16, paleYellow);
        uiY += uiStep;
        DrawText("Shift+RClick = select 2nd face", 20, uiY, 16, paleYellow);
        uiY += uiStep + blockGap;
    }

    // block 4
    if (model.selectedFace > -1 && model.secondFace > -1) {
        float dist = computeFaceMinDistance(
            model.pickData[model.selectedFace], model.pickData[model.secondFace], model.faceOffsets[model.selectedFace], model.faceOffsets[model.secondFace]);
        DrawText(TextFormat("Distance to #%d: %.4f mm", model.secondFace, dist), 20, uiY, 16, paleOrange);
        uiY += uiStep;
        DrawText("D = distance constraint", 20, uiY, 16, paleOrange);
        uiY += uiStep;
        DrawText("S = symmetry constraint", 20, uiY, 16, paleOrange);
        uiY += uiStep;
    }

    // drawn last so they render on top of all other UI
    drawConstraintPanel(model);
    drawToasts(model);
}

#pragma region loadStep
// arcSegs: arc subdivision count (circles, cylinders, toruses), essentially LOD
// 48 segments is an arbitrary feel-tuned value for smooth circles, I'll also use half of it for smaller torus tubes later
CadModel loadStep(const std::string& path, int arcSegs)
{
    CadModel model;
    // initialize bbox inverted so the first real vertex always wins both min and max comparisons
    // ex: first vertex at (3,1,2) -> min becomes (3,1,2), max becomes (3,1,2)
    model.bbox = { { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() },
        { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() } };
    StepMap entityMap = parseStepFile(path);

    // collect all ADVANCED_FACE entity IDs (each face becomes one mesh)
    std::vector<int> faceIds;
    for (auto& [id, entity] : entityMap)
        if (entity.type == "ADVANCED_FACE")
            faceIds.push_back(id);
    std::sort(faceIds.begin(), faceIds.end());

    // for each advanced face, by ID, go down its IDs nesting to resolve the face (shape) and add it to the model
    for (int faceId : faceIds) {
        Surface faceSurface;
        CylinderHeightRange cylHeightRange;
        TessellatedFace tessellatedFace = tessellateAdvancedFace(faceId, entityMap, arcSegs, &faceSurface, &cylHeightRange);
        if (tessellatedFace.indices.empty())
            continue;
        // expand the overall bounding box with all front-face vertices of this face
        // vertices are stored flat as [x0,y0,z0, ...front..., ...back...], back = same positions so only iterate front half
        int frontVertCount = (int)tessellatedFace.vertices.size() / 6; // front_count = total_floats/3 / 2
        for (int i = 0; i < frontVertCount; i++) {
            int base = i * 3;
            model.bbox.min.x = std::min(model.bbox.min.x, tessellatedFace.vertices[base]);
            model.bbox.min.y = std::min(model.bbox.min.y, tessellatedFace.vertices[base + 1]);
            model.bbox.min.z = std::min(model.bbox.min.z, tessellatedFace.vertices[base + 2]);
            model.bbox.max.x = std::max(model.bbox.max.x, tessellatedFace.vertices[base]);
            model.bbox.max.y = std::max(model.bbox.max.y, tessellatedFace.vertices[base + 1]);
            model.bbox.max.z = std::max(model.bbox.max.z, tessellatedFace.vertices[base + 2]);
        }
        Mesh mesh = uploadMesh(tessellatedFace);
        if (mesh.vertexCount == 0)
            continue;
        // all parallel SoA arrays stay in sync, one entry per successfully uploaded face
        model.meshes.push_back(mesh);
        model.colors.push_back(colorForKind(tessellatedFace.kind));
        model.pickData.push_back(tessellatedFace);
        model.faceSurfaces.push_back(faceSurface);
        model.faceAreas.push_back(computeFaceArea(tessellatedFace));
        model.faceOffsets.push_back({ 0.0f, 0.0f, 0.0f });
        model.cylHeightRanges.push_back(cylHeightRange); // will be 0 for other shapes, still need to push for SoA
        model.totalTriangleCount += (int)(tessellatedFace.indices.size() / 6); // front triangles only
    }
    return model;
}

#pragma region main
int main()
{
    SetTraceLogLevel(LOG_WARNING);
    InitWindow(1280, 720, "CAD");
    SetTargetFPS(60);

    // executable either at root or in build/Release idk xd
    const std::string stepPath = std::filesystem::exists("cad/cad.step") ? "cad/cad.step" : "../../cad/cad.step";
    CadModel model;
    try {
        model = loadStep(stepPath);
    } catch (const std::runtime_error& e) {
        TraceLog(LOG_ERROR, "STEP parse failed: %s", e.what());
        CloseWindow();
        return 1;
    }

    Vector3 modelSize = { model.bbox.max.x - model.bbox.min.x, model.bbox.max.y - model.bbox.min.y, model.bbox.max.z - model.bbox.min.z };
    // diagonal of the bounding box, aka "diameter" from one corner to the opposite,
    // used to scale camera distance and zoom speed so they feel consistent regardless of model size
    float diagonal = sqrtf(modelSize.x * modelSize.x + modelSize.y * modelSize.y + modelSize.z * modelSize.z);

    auto initCamera = [&]() {
        // default camera state, feel-tuned
        Camera3D cam = { };
        cam.target = { 0, 0, 0 }; // where it looks at (our model)
        cam.up = { 0, 1, 0 };
        cam.fovy = 45;
        cam.projection = CAMERA_PERSPECTIVE;
        return cam;
    };

    // spherical camera position
    float yaw = 45.0f; // degrees, horizontal rotation
    float pitch = -25.0f; // degrees, vertical rotation
    float orbitRadius = diagonal * 1.8f; // initial distance
    Camera3D camera = initCamera();

    bool showNormals = false, showBbox = false;

    while (!WindowShouldClose()) {
        // reload the model and reinitialise all camera and view state from scratch
        if (model.needsReset) {
            try {
                model = loadStep(stepPath);
            } catch (const std::runtime_error& e) {
                TraceLog(LOG_ERROR, "STEP parse failed: %s", e.what());
                CloseWindow();
                return 1;
            }
            modelSize = { model.bbox.max.x - model.bbox.min.x, model.bbox.max.y - model.bbox.min.y, model.bbox.max.z - model.bbox.min.z };
            diagonal = sqrtf(modelSize.x * modelSize.x + modelSize.y * modelSize.y + modelSize.z * modelSize.z);
            yaw = 45.0f;
            pitch = -25.0f;
            orbitRadius = diagonal * 1.8f;
            camera = initCamera();
            showNormals = false;
            showBbox = false;
        }
        handleControls(model, camera, yaw, pitch, orbitRadius, showNormals, showBbox, diagonal);

        BeginDrawing();
        {
            ClearBackground({ 18, 18, 22, 255 });
            BeginMode3D(camera);
            {
                drawCadModel(model);
                drawSelectedFaceHighlight(model);
                if (model.secondFace > -1)
                    drawSecondFaceHighlight(model);
                if (showBbox) {
                    if (model.selectedFace > -1)
                        drawFaceBbox(model);
                    else
                        drawModelBbox(model);
                }
                if (showNormals) {
                    if (model.selectedFace > -1) {
                        drawFaceNormals(model, diagonal * 0.03f);
                        drawFaceAnalyticalAxis(model, diagonal * 0.15f);
                    } else {
                        drawModelAverageNormal(model, diagonal * 0.3f);
                    }
                }
            }
            EndMode3D();
            drawUI(model);
        }
        EndDrawing();
    }
    CloseWindow();
    return 0;
}