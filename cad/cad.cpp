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

#pragma region loadStep
// arcSegs: arc subdivision count (circles, cylinders, tori), essentially LOD
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
        TessellatedFace tessellatedFace = tessellateAdvancedFace(faceId, entityMap, arcSegs, &faceSurface);
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
        // record height range for cylinders so healing can retessellate with updated bounds
        // project all front-face vertices onto the cylinder axis to recover [heightMin, heightMax]
        CylinderHeightRange chr;
        if (faceSurface.kind == SurfaceKind::Cylinder) {
            Vec3 cylOrigin = faceSurface.axis.origin, cylZ = faceSurface.axis.zDir.norm();
            chr.heightMin = std::numeric_limits<double>::max();
            chr.heightMax = -std::numeric_limits<double>::max();
            int frontVertexCount = (int)tessellatedFace.vertices.size() / 6;
            for (int vi = 0; vi < frontVertexCount; vi++) {
                int base = vi * 3;
                Vec3 pt = { tessellatedFace.vertices[base], tessellatedFace.vertices[base + 1], tessellatedFace.vertices[base + 2] };
                double h = (pt - cylOrigin).dot(cylZ);
                chr.heightMin = std::min(chr.heightMin, h);
                chr.heightMax = std::max(chr.heightMax, h);
            }
        }
        model.cylHeightRanges.push_back(chr);
        model.totalTriangleCount += (int)(tessellatedFace.indices.size() / 6); // front triangles only
    }
    return model;
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
    for (int faceIndex = 0; faceIndex < (int)model.pickData.size(); faceIndex++) {
        const auto& faceData = model.pickData[faceIndex];
        // offset is in draw space (post-centering), so add it to vertices in the same adjusted space
        // equivalent to subtracting it from the ray origin per face, but cheaper to apply once to a local ray copy
        Vector3 off = model.faceOffsets[faceIndex];
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
                hitFace = faceIndex;
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

// redraws the selected face in solid white then yellow wireframe so it stands out from the scene
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
            wireMaterial.maps[MATERIAL_MAP_DIFFUSE].color = YELLOW;
            DrawMesh(model.meshes[model.selectedFace], wireMaterial, centeredTransform);
            UnloadMaterial(wireMaterial);
        }
        rlDisableWireMode();
    }
    rlEnableBackfaceCulling();
}

// redraws the distance-reference face (shift+right clicked) with a brightened tint (so we know we clicked well)
static void drawDistFaceHighlight(const CadModel& model)
{
    Vector3 center = modelCenter(model);
    Vector3 off = model.faceOffsets[model.distFace];
    Matrix centeredTransform = MatrixMultiply(MatrixTranslate(off.x, off.y, off.z), MatrixTranslate(-center.x, -center.y, -center.z));
    rlDisableBackfaceCulling();
    {
        Color base = model.colors[model.distFace];
        Color brightened = { (unsigned char)std::min(255, (int)base.r + 80), (unsigned char)std::min(255, (int)base.g + 80),
            (unsigned char)std::min(255, (int)base.b + 80), 255 };
        Material solidMaterial = LoadMaterialDefault();
        solidMaterial.maps[MATERIAL_MAP_DIFFUSE].color = brightened;
        DrawMesh(model.meshes[model.distFace], solidMaterial, centeredTransform);
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

// draws the whole model bounding box (centered at origin to match drawCadModel) in gold
// recomputed each call to reflect any per-face offsets accumulated since load
static void drawModelBbox(const CadModel& model)
{
    Vector3 center = modelCenter(model);
    BoundingBox dynamicBbox = { { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() },
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
            dynamicBbox.min.x = std::min(dynamicBbox.min.x, x);
            dynamicBbox.max.x = std::max(dynamicBbox.max.x, x);
            dynamicBbox.min.y = std::min(dynamicBbox.min.y, y);
            dynamicBbox.max.y = std::max(dynamicBbox.max.y, y);
            dynamicBbox.min.z = std::min(dynamicBbox.min.z, z);
            dynamicBbox.max.z = std::max(dynamicBbox.max.z, z);
        }
    }
    DrawBoundingBox(dynamicBbox, GOLD);
}

// draws the bounding box of the selected face in orange, for planes this will be a flat rectangle, for cylinders/tori a proper 3D box
static void drawFaceBbox(const CadModel& model)
{
    if (model.selectedFace < 0 || model.selectedFace >= (int)model.pickData.size())
        return;
    Vector3 center = modelCenter(model);
    DrawBoundingBox(computeFaceBBox(model.pickData[model.selectedFace], center, model.faceOffsets[model.selectedFace]), ORANGE);
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

// computes live model bbox across all faces including their current offsets, in STEP space (not draw space)
static BoundingBox computeLiveModelBBox(const CadModel& model)
{
    BoundingBox bbox = { { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() },
        { -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max() } };
    for (int fi = 0; fi < (int)model.pickData.size(); fi++) {
        const auto& face = model.pickData[fi];
        Vector3 off = model.faceOffsets[fi];
        int frontVertexCount = (int)face.vertices.size() / 6;
        for (int i = 0; i < frontVertexCount; i++) {
            int base = i * 3;
            float x = face.vertices[base] + off.x;
            float y = face.vertices[base + 1] + off.y;
            float z = face.vertices[base + 2] + off.z;
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
    BoundingBox liveBbox = computeLiveModelBBox(model);
    Vector3 staticCenter = modelCenter(model);
    Vector3 origin3D = { (liveBbox.min.x + liveBbox.max.x) * 0.5f - staticCenter.x, (liveBbox.min.y + liveBbox.max.y) * 0.5f - staticCenter.y,
        (liveBbox.min.z + liveBbox.max.z) * 0.5f - staticCenter.z };
    Vector3 tip = { origin3D.x + (float)avgNormal.x * scale, origin3D.y + (float)avgNormal.y * scale, origin3D.z + (float)avgNormal.z * scale };
    DrawLine3D(origin3D, tip, SKYBLUE);
}

#pragma region handleControls
static void handleControls(CadModel& model, Camera3D& camera, float& yaw, float& pitch, float& orbitRadius, bool& showNormals, bool& showBbox, float diagonal)
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

    if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)) {
        int hitFace = pickFace(model, GetMouseRay(GetMousePosition(), camera));
        if (IsKeyDown(KEY_LEFT_SHIFT) || IsKeyDown(KEY_RIGHT_SHIFT)) {
            if (model.selectedFace > -1)
                model.distFace = (hitFace > -1 && hitFace != model.selectedFace) ? hitFace : -1;
        } else {
            model.selectedFace = hitFace;
            model.distFace = -1;
        }
    }

    if (IsKeyPressed(KEY_N))
        showNormals = !showNormals;
    if (IsKeyPressed(KEY_B))
        showBbox = !showBbox;

    // push/pull: move selected face along its surface's own axes
    // UP/DOWN = zDir (normal for planes, axis for cylinders/tori), LEFT/RIGHT = xDir (in-plane tangent), PgUp/PgDn = yDir (zDir×xDir)
    // wasTranslating detects the rising edge of a translation gesture so one continuous hold = one undo entry
    bool translating = false;
    if (model.selectedFace > -1) {
        float step = diagonal * 0.005f;
        const Surface& surf = model.faceSurfaces[model.selectedFace];
        Vec3 zDir = surf.axis.zDir.norm();
        Vec3 xDir = surf.axis.xDir.norm();
        Vec3 yDir = zDir.cross(xDir).norm();
        Vector3& off = model.faceOffsets[model.selectedFace];
        translating
            = IsKeyDown(KEY_UP) || IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_PAGE_UP) || IsKeyDown(KEY_PAGE_DOWN);
        if (translating && !model.wasTranslating) {
            // always rebuild at every gesture rising edge so connectivity is re-evaluated at the plane's current
            // position, a stale cache from a prior gesture must never carry over because the plane may have
            // been moved off-axis (disconnected from the cylinder) between gestures
            model.healCache = buildCylinderHealCache(model, model.selectedFace);
            model.cachedHealFace = model.selectedFace;
            // snapshot the pre-gesture cylinder height ranges for every cylinder this gesture will heal so
            // undo/redo can retessellate them back to the correct extent, not just the plane offset
            UndoEntry entry;
            entry.faceIndex = model.selectedFace;
            entry.offsetBefore = off;
            for (const auto& healEntry : model.healCache)
                entry.cylSnapshots.push_back({ healEntry.cylFaceIdx, model.cylHeightRanges[healEntry.cylFaceIdx] });
            model.undoStack.push_back(std::move(entry));
            model.redoStack.clear();
        }
        // invalidate cache if the selected face changed between gestures
        if (model.cachedHealFace != model.selectedFace) {
            model.healCache.clear();
            model.cachedHealFace = -1;
        }
        model.wasTranslating = translating;
        Vec3 planeNormal = surf.axis.zDir.norm();
        auto move = [&](int key, Vec3 dir) {
            if (!IsKeyDown(key))
                return;
            Vector3 delta = { (float)dir.x * step, (float)dir.y * step, (float)dir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, model.healCache, planeNormal, delta);
        };
        move(KEY_UP, zDir);
        move(KEY_DOWN, zDir * -1.0);
        move(KEY_RIGHT, xDir);
        move(KEY_LEFT, xDir * -1.0);
        move(KEY_PAGE_UP, yDir);
        move(KEY_PAGE_DOWN, yDir * -1.0);
    } else {
        model.wasTranslating = false;
        model.healCache.clear();
        model.cachedHealFace = -1;
    }

    // (ctrl+z is swallowed by the Win32 message loop as ASCII SUB before Raylib sees the key event XDD)
    // both blocked while any translation key is held to prevent stomping an in-progress gesture
    // same principle for both: pop from 'src', capture current state into an inverse entry, push to 'dst', then restore
    auto applyUndoRedoStep = [&](std::vector<UndoEntry>& src, std::vector<UndoEntry>& dst) {
        if (src.empty())
            return;
        UndoEntry entry = src.back();
        src.pop_back();
        UndoEntry inverse;
        inverse.faceIndex = entry.faceIndex;
        inverse.offsetBefore = model.faceOffsets[entry.faceIndex];
        for (const auto& snap : entry.cylSnapshots)
            inverse.cylSnapshots.push_back({ snap.cylFaceIdx, model.cylHeightRanges[snap.cylFaceIdx] });
        dst.push_back(std::move(inverse));
        model.faceOffsets[entry.faceIndex] = entry.offsetBefore;
        for (const auto& snap : entry.cylSnapshots) {
            model.cylHeightRanges[snap.cylFaceIdx] = snap.rangeBefore;
            retessCylinderFace(model, snap.cylFaceIdx, snap.rangeBefore.heightMin, snap.rangeBefore.heightMax);
        }
        model.selectedFace = entry.faceIndex;
    };
    if (!translating && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_U))
        applyUndoRedoStep(model.undoStack, model.redoStack);
    if (!translating && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_Y))
        applyUndoRedoStep(model.redoStack, model.undoStack);

    // convert spherical (yaw, pitch, orbitRadius) to Cartesian camera position (x y z)
    // ex: yaw=90deg, pitch=0 -> camera sits on the +X axis looking toward origin
    float yawRadians = DEG2RAD * yaw, pitchRadians = DEG2RAD * pitch;
    camera.position = { orbitRadius * std::cos(pitchRadians) * std::sin(yawRadians), orbitRadius * std::sin(pitchRadians),
        orbitRadius * std::cos(pitchRadians) * std::cos(yawRadians) };
}

#pragma region drawUI
static void drawUI(const CadModel& model)
{
    // stats panel with running Y so adding lines never requires renumbering
    int uiY = 20;
    const int uiStep = 20;
    DrawText("RED = Cylinders", 20, uiY, 16, RED);
    uiY += uiStep;
    DrawText("GREEN = Tori (fillets)", 20, uiY, 16, GREEN);
    uiY += uiStep;
    DrawText("BLUE = Planes", 20, uiY, 16, BLUE);
    uiY += uiStep;
    DrawText("DARKGRAY = Other", 20, uiY, 16, DARKGRAY);
    uiY += uiStep + 6;

    DrawText(TextFormat("Faces: %d\nTriangles: %d", (int)model.meshes.size(), model.totalTriangleCount), 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep * 2; // because of the \n, applied down there too

    // model bbox dimensions and center recomputed each frame to reflect per-face offsets
    BoundingBox liveBbox = computeLiveModelBBox(model);
    Vector3 mCenter = modelCenter(model);
    DrawText(TextFormat("Width: %.2f\nHeight: %.2f\nDepth: %.2f mm", liveBbox.max.x - liveBbox.min.x, liveBbox.max.y - liveBbox.min.y,
                 liveBbox.max.z - liveBbox.min.z),
        20, uiY, 16, LIGHTGRAY);
    uiY += uiStep * 3;
    DrawText(TextFormat("Position: %.2f %.2f %.2f", (liveBbox.min.x + liveBbox.max.x) * 0.5f - mCenter.x, (liveBbox.min.y + liveBbox.max.y) * 0.5f - mCenter.y,
                 (liveBbox.min.z + liveBbox.max.z) * 0.5f - mCenter.z),
        20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;

    DrawText("Drag = orbit", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("Scroll = zoom", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("N = normals", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("B = bounding box", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("RClick = select face", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("On selection:", 20, uiY, 16, YELLOW);
    uiY += uiStep;
    DrawText("Shift+RClick = get distance", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("Arrows/page keys = translate", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("Ctrl+U/Y = undo/redo", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;

    if (model.selectedFace < 0)
        return;

    uiY += 6;
    int faceIdx = model.selectedFace;
    int frontTris = (int)model.pickData[faceIdx].indices.size() / 6;
    DrawText(
        TextFormat("Face #%d [%s]\nTriangles: %d\nSurface: %.2f mm^2", faceIdx, nameForKind(model.pickData[faceIdx].kind), frontTris, model.faceAreas[faceIdx]),
        20, uiY, 16, YELLOW);
    uiY += uiStep * 3;

    // face bbox dimensions and center (in draw space = relative to model center = world coords when model at origin)
    // offset applied so stats match the visual position of the face after push/pull
    Vector3 center = modelCenter(model);
    BoundingBox faceBbox = computeFaceBBox(model.pickData[faceIdx], center, model.faceOffsets[faceIdx]);
    DrawText(TextFormat("Width: %.2f\nHeight: %.2f\nDepth: %.2f mm", faceBbox.max.x - faceBbox.min.x, faceBbox.max.y - faceBbox.min.y,
                 faceBbox.max.z - faceBbox.min.z),
        20, uiY, 16, YELLOW);
    uiY += uiStep * 3;
    DrawText(TextFormat("Position: %.2f, %.2f, %.2f", (faceBbox.min.x + faceBbox.max.x) * 0.5f, (faceBbox.min.y + faceBbox.max.y) * 0.5f,
                 (faceBbox.min.z + faceBbox.max.z) * 0.5f),
        20, uiY, 16, YELLOW);
    uiY += uiStep;

    if (model.distFace > -1) {
        float dist = computeFaceMinDistance(
            model.pickData[model.selectedFace], model.pickData[model.distFace], model.faceOffsets[model.selectedFace], model.faceOffsets[model.distFace]);
        DrawText(TextFormat("Distance to #%d: %.4f mm", model.distFace, dist), 20, uiY, 16, ORANGE);
    }
}

#pragma region main
int main()
{
    InitWindow(1280, 720, "CAD");
    SetTargetFPS(60);

    // executable either at root or in build/Release idk xd
    CadModel model = loadStep(std::filesystem::exists("cad/cad.step") ? "cad/cad.step" : "../../cad/cad.step");

    Vector3 modelSize = { model.bbox.max.x - model.bbox.min.x, model.bbox.max.y - model.bbox.min.y, model.bbox.max.z - model.bbox.min.z };
    // diagonal of the bounding box, aka "diameter" from one corner to the opposite,
    // used to scale camera distance and zoom speed so they feel consistent regardless of model size
    // ex: bbox 10x5x2 -> diagonal = sqrt(100+25+4) = 11.36
    float diagonal = sqrtf(modelSize.x * modelSize.x + modelSize.y * modelSize.y + modelSize.z * modelSize.z);

    // default camera state, feel-tuned
    float yaw = 45.0f; // degrees, horizontal rotation
    float pitch = -25.0f; // degrees, vertical rotation
    float orbitRadius = diagonal * 1.8f; // initial distance
    Camera3D camera = {};
    camera.target = { 0, 0, 0 };
    camera.up = { 0, 1, 0 };
    camera.fovy = 45;
    camera.projection = CAMERA_PERSPECTIVE;

    bool showNormals = false, showBbox = false;

    while (!WindowShouldClose()) {
        handleControls(model, camera, yaw, pitch, orbitRadius, showNormals, showBbox, diagonal);

        BeginDrawing();
        ClearBackground({ 18, 18, 22, 255 });
        BeginMode3D(camera);
        drawCadModel(model);
        drawSelectedFaceHighlight(model);
        if (model.distFace > -1)
            drawDistFaceHighlight(model);
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
        EndMode3D();
        drawUI(model);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}