#include "cad.hpp"

#pragma region 2D ear-clip
// imagine a polygon drawn on paper, an "ear" is any three consecutive vertices where the middle one can be cut off cleanly
// the triangle formed by those three vertices lies entirely inside the polygon and contains no other vertices, ear-clipping just repeatedly
// - finds one of those ears (3 adjacent points forming a triangle)
// - emits the triangle
// - removes the middle vertex
// - repeats by finding a new triangle using the 2 remaining points and a new adjacent one
// however you gotta watch out for reflex vertices, in a "convex" polygon (all inner angles under 180° aka the classic ones),
// a "reflex vertex" describes the specific vertex where a concavity happens, if it's the middle vertex then the triangle we make will be outside of the plane
// so we skip it and make sure it's never a middle vertex and only clip ears at convex ones
// it is done in 2D because the faces are flat planes, you project the 3D boundary down to a flat coordinate system (the plane's own UV tangent frame),
// triangulate there where the math is simple, then lift the result back to 3D

// used only for planes (curved surfaces use analytical UV grid) signed area of a 2D polygon via the shoelace formula positive = CCW winding,
// negative = CW winding ex: unit square CCW [(0,0),(1,0),(1,1),(0,1)] -> +1.0
// same square CW -> -1.0
static double area2D(const std::vector<Vec2>& polygon)
{
    double area = 0;
    int count = (int)polygon.size();
    for (int i = 0; i < count; i++) {
        auto &current = polygon[i], &next = polygon[(i + 1) % count];
        area += current.u * next.v - next.u * current.v;
    }
    return area;
}

// 2D cross product of vectors (b-a) and (c-a)
// positive = CCW turn at b, negative = CW turn (reflex), zero = collinear
// ex: a=(0,0), b=(1,0), c=(1,1) -> +1.0 (left turn, convex ear candidate)
//     a=(0,0), b=(1,0), c=(0,-1) ->  -1.0 (right turn, reflex vertex, not an ear)
static double cross2D(Vec2 a, Vec2 b, Vec2 c) { return (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u); }

// returns true if point p lies inside (or on the edge of) triangle abc
// uses the sign-consistency test: p is inside iff all three edge cross products share the same sign
// ex: triangle (0,0),(1,0),(0,1), p=(0.2,0.2) -> true
//     same triangle,              p=(0.9,0.9) -> false (outside hypotenuse)
static bool pointIn2DTriangle(Vec2 point, Vec2 a, Vec2 b, Vec2 c)
{
    double cross1 = cross2D(point, a, b), cross2 = cross2D(point, b, c), cross3 = cross2D(point, c, a);
    return !((cross1 < 0 || cross2 < 0 || cross3 < 0) && (cross1 > 0 || cross2 > 0 || cross3 > 0));
}

// ear-clip a simple polygon, returns triangle indices into 'polygon'
static std::vector<std::array<int, 3>> earClip(const std::vector<Vec2>& polygon)
{
    std::vector<std::array<int, 3>> triangles;
    int vertexCount = (int)polygon.size();
    if (vertexCount < 3)
        return triangles;
    // working index ring: we'll remove vertices from this as ears are clipped
    // ex: polygon with 5 verts -> remaining=[0,1,2,3,4]; clip ear at 1 -> remaining=[0,2,3,4]
    std::vector<int> remaining(vertexCount);
    std::iota(remaining.begin(), remaining.end(), 0);
    // ensure CCW winding so the convexity test (cross > 0) is consistent
    if (area2D(polygon) < 0)
        std::reverse(remaining.begin(), remaining.end());

    // vertexCount*vertexCount*2 iterations: more headroom for self-touching bridge-seam polygons
    // a plain convex polygon needs only vertexCount-2 clips; the *2+64 cushions degenerate flat ears
    // ex: vertexCount=10 -> maxIterations=264; vertexCount=100 -> maxIterations=20064
    int maxIterations = vertexCount * vertexCount * 2 + 64, iterationCount = 0;
    while ((int)remaining.size() > 3 && iterationCount++ < maxIterations) {
        int ringSize = (int)remaining.size();
        bool earFound = false;
        for (int i = 0; i < ringSize; i++) {
            int indexA = remaining[(i - 1 + ringSize) % ringSize], indexB = remaining[i], indexC = remaining[(i + 1) % ringSize];
            double crossProduct = cross2D(polygon[indexA], polygon[indexB], polygon[indexC]);
            // skip reflex vertices (CW turn, crossProduct < 0); allow near-zero (flat) ears since
            // bridge seams create collinear (lie on the same straight line) triples that must still be clipped
            // ex: crossProduct = -0.5 -> skip (reflex);  crossProduct = 1e-11 -> accept (bridge seam flat ear)
            if (crossProduct < -1e-10)
                continue;

            bool isEar = true;
            for (int j = 0; j < ringSize && isEar; j++) {
                if (j == (i - 1 + ringSize) % ringSize || j == i || j == (i + 1) % ringSize)
                    continue;
                Vec2 candidate = polygon[remaining[j]];
                // skip vertices geometrically coincident with ear vertices,
                // bridge seams duplicate vertices; coincident points falsely fail the inside-test and block all valid ears
                const double coincidenceEps = 1e-9;
                if ((std::abs(candidate.u - polygon[indexA].u) < coincidenceEps && std::abs(candidate.v - polygon[indexA].v) < coincidenceEps)
                    || (std::abs(candidate.u - polygon[indexB].u) < coincidenceEps && std::abs(candidate.v - polygon[indexB].v) < coincidenceEps)
                    || (std::abs(candidate.u - polygon[indexC].u) < coincidenceEps && std::abs(candidate.v - polygon[indexC].v) < coincidenceEps))
                    continue;
                if (pointIn2DTriangle(candidate, polygon[indexA], polygon[indexB], polygon[indexC]))
                    isEar = false;
            }
            if (isEar) {
                // only emit non-degenerate (non-zero-area) triangles
                // ex: crossProduct=1e-15 (collinear bridge seam triple) -> skip, no triangle emitted
                if (crossProduct > 1e-14) {
                    std::array<int, 3> triangle = { indexA, indexB, indexC };
                    triangles.push_back(triangle);
                }
                remaining.erase(remaining.begin() + i);
                earFound = true;
                break;
            }
        }
        if (!earFound)
            break;
    }
    // last remaining triangle
    if ((int)remaining.size() == 3) {
        double crossProduct = cross2D(polygon[remaining[0]], polygon[remaining[1]], polygon[remaining[2]]);
        if (crossProduct > 1e-14) {
            std::array<int, 3> triangle = { remaining[0], remaining[1], remaining[2] };
            triangles.push_back(triangle);
        }
    }
    return triangles;
}

// O'Rourke visible-vertex bridge algorithm for hole merging:
// we need to connect a hole to the outer polygon without the connection line crossing any existing edge
// - start from the rightmost point of the hole and shoota ray to the right
// - find which outer polygon edge the ray hits first
// - of the two endpoints of that edge, pick the one further right, that vertex is visible from your hole point by construction (the ray guarantees it)
// - draw the bridge there

// when you have a polygon with a hole in it, the ear-clipper can only handle simple polygons (no holes)
// a bridge cut is a straight line segment that connects a point on the hole boundary to a point on the outer boundary,
// now it has no hole, just a slightly more complex outline that the ear-clipper can handle normally
// since it produce degenerate zero-area ears (the two duplicate vertices are collinear), the ear-clipper skips without emitting a triangle for them

// given the current (possibly already-merged) outer polygon and the +X ray origin rayOrigin,
// returns the index of the polygon vertex that is the best bridge target:
// the vertex of the first edge hit by the ray that has the larger X coordinate
// ex: rayOrigin=(3,1), outer square [(0,0),(5,0),(5,3),(0,3)]
//     ray hits edge (5,0)-(5,3) at x=5 -> both endpoints have same U, pick (5,3) via >= check
static int findBridgeVertex(const std::vector<Vec2>& polygon, Vec2 rayOrigin)
{
    int vertexCount = (int)polygon.size();
    double closestDistance = 1e18;
    int bestEdgeIndex = -1;

    for (int i = 0; i < vertexCount; i++) {
        Vec2 edgeStart = polygon[i], edgeEnd = polygon[(i + 1) % vertexCount];
        // only consider edges that straddle rayOrigin's V coordinate (can be hit by horizontal ray).
        // ex: rayOrigin.v=1, edge from (2,0) to (2,3) -> straddles (0<=1<3), keep
        //     rayOrigin.v=1, edge from (2,2) to (4,3) -> both above rayOrigin.v, skip
        if ((edgeStart.v <= rayOrigin.v && edgeEnd.v <= rayOrigin.v) || (edgeStart.v > rayOrigin.v && edgeEnd.v > rayOrigin.v))
            continue;
        // parametric intersection: find X where the edge crosses V = rayOrigin.v
        // param = (rayOrigin.v - edgeStart.v) / (edgeEnd.v - edgeStart.v),  xHit = edgeStart.u + (edgeEnd.u - edgeStart.u)*param
        double deltaV = edgeEnd.v - edgeStart.v;
        if (std::abs(deltaV) < 1e-12)
            continue;
        double param = (rayOrigin.v - edgeStart.v) / deltaV;
        double xHit = edgeStart.u + (edgeEnd.u - edgeStart.u) * param;
        // distanceAlongRay is the signed distance along +X from rayOrigin to the hit; skip edges to the left
        // ex: rayOrigin.u=3, xHit=5 -> distanceAlongRay=+2 (to the right, valid hit)
        //     rayOrigin.u=3, xHit=1 -> distanceAlongRay=-2 (to the left, skip)
        double distanceAlongRay = xHit - rayOrigin.u;
        if (distanceAlongRay < -1e-9)
            continue;
        if (distanceAlongRay < closestDistance) {
            closestDistance = distanceAlongRay;
            bestEdgeIndex = i;
        }
    }

    if (bestEdgeIndex < 0) {
        // fallback: nearest vertex (handles degenerate geometry where no edge is hit)
        double bestDistSq = 1e18;
        int nearestVertex = 0;
        for (int i = 0; i < vertexCount; i++) {
            double distSq = (polygon[i].u - rayOrigin.u) * (polygon[i].u - rayOrigin.u) + (polygon[i].v - rayOrigin.v) * (polygon[i].v - rayOrigin.v);
            if (distSq < bestDistSq) {
                bestDistSq = distSq;
                nearestVertex = i;
            }
        }
        return nearestVertex;
    }

    // of the two edge endpoints, pick the one with the larger X (more rightward),
    // which is guaranteed to be mutually visible from rayOrigin without crossing the edge
    // ex: edge endpoints (4,0) and (5,2) -> pick index of (5,2) since 5 > 4
    int indexA = bestEdgeIndex, indexB = (bestEdgeIndex + 1) % vertexCount;
    return (polygon[indexA].u >= polygon[indexB].u) ? indexA : indexB;
}

// merge hole polygons into the outer polygon via bridge cuts (O'Rourke method),
// sort holes rightmost-first so each bridge is inserted before any hole to its left
// ex: washer face = outer square + 1 circular hole -> returns one merged polygon with a seam cut
static std::vector<Vec2> buildMergedPolygon(const std::vector<Vec2>& outer, const std::vector<std::vector<Vec2>>& holes)
{
    if (holes.empty())
        return outer;

    // sort holes by their rightmost X coordinate, descending
    // this ensures bridge cuts from right-side holes don't cross left-side holes
    // ex: two holes with rightmost X=8 and X=3 -> process X=8 hole first
    std::vector<int> holeOrder((int)holes.size());
    std::iota(holeOrder.begin(), holeOrder.end(), 0);
    std::sort(holeOrder.begin(), holeOrder.end(), [&](int holeA, int holeB) {
        double rightmostA = -1e18, rightmostB = -1e18;
        for (auto& vertex : holes[holeA])
            rightmostA = std::max(rightmostA, vertex.u);
        for (auto& vertex : holes[holeB])
            rightmostB = std::max(rightmostB, vertex.u);
        return rightmostA > rightmostB;
    });

    std::vector<Vec2> merged = outer;

    for (int holeIndex : holeOrder) {
        const auto& hole = holes[holeIndex];
        if (hole.empty())
            continue;

        // find rightmost point of this hole (the bridge starts here)
        int rightmostIndex = 0;
        for (int i = 1; i < (int)hole.size(); i++)
            if (hole[i].u > hole[rightmostIndex].u)
                rightmostIndex = i;
        Vec2 bridgeSource = hole[rightmostIndex];

        // find visible bridge vertex on the outer polygon using ray casting
        int bridgeTargetIndex = findBridgeVertex(merged, bridgeSource);

        // splice hole into merged polygon at bridgeTargetIndex:
        // outer[0 to bridgeTargetIndex] -> hole[rightmostIndex to rightmostIndex-1] -> hole[rightmostIndex] (seam) -> outer[bridgeTargetIndex to end]
        // the duplicated vertices are what create the "bridge cut"
        // ex: outer=[A,B,C,D], bridgeTargetIndex=1 (=B), hole=[H0,H1,H2], rightmostIndex=0
        //  -> [A, B, H0,H1,H2, H0, B, C, D]  (B and H0 each appear twice = the two seam edges)
        std::vector<Vec2> spliced;
        spliced.reserve(merged.size() + hole.size() + 2);
        for (int i = 0; i <= bridgeTargetIndex; i++)
            spliced.push_back(merged[i]);
        for (int i = 0; i < (int)hole.size(); i++)
            spliced.push_back(hole[(rightmostIndex + i) % hole.size()]);
        spliced.push_back(hole[rightmostIndex]); // close hole seam
        spliced.push_back(merged[bridgeTargetIndex]); // bridge back to outer
        for (int i = bridgeTargetIndex + 1; i < (int)merged.size(); i++)
            spliced.push_back(merged[i]);
        merged = std::move(spliced);
    }

    // remove consecutive near-duplicate vertices (bridge seam creates pairs like [A, A]),
    // keeping them causes zero-area ears that stall the ear-clipper
    // ex: [..., B, B, ...] -> [..., B, ...]  (one of the two bridge-seam duplicates removed)
    std::vector<Vec2> deduped;
    deduped.reserve(merged.size());
    for (int i = 0; i < (int)merged.size(); i++) {
        Vec2& current = merged[i];
        Vec2& previous = merged[(i - 1 + (int)merged.size()) % (int)merged.size()];
        if (std::abs(current.u - previous.u) > 1e-10 || std::abs(current.v - previous.v) > 1e-10)
            deduped.push_back(current);
    }
    return deduped;
}

#pragma region uv grid
// U and V are just parameter names for the two axes of a surface's own coordinate system, the same way X/Y are axes in world space,
// a cylinder has U = angle around the axis (0 to 2π for a full revolution) and V = height along it (heightMin to heightMax),
// imagine you need to wallpaper a cylindrical pillar, just unroll a rectangular sheet and wrap it around, that's the UV grid
// projecting a cylinder boundary onto a flat plane would give two overlapping circles, the ear-clipper would produce garbage,
// instead we tessellate the surface analytically in (u,v) parameter space and emit a clean quad grid
// "analytically" here means the normal or position is computed from the closed-form mathematical definition of the surface
// rather than estimated by, say, averaging the normals of surrounding triangles (tldr exact formula, no approximation)
// inversely, we couldn't use this amazing UV grid for planes because it does not have a natural parameter space like cylinder's and tori's
// angle and height, a plane is infinite and has no intrinsic coordinate system (you have to invent one, tangentX/tangentY) and
// its boundary shape can be absolutely anything, hence why a recursive exploratory algorithm like ear-clipper is better

// appends a single vertex (position + normal) to the flat float arrays of a TessellatedFace
// ex: pos=(1,0,0), nor=(1,0,0) -> pushes 6 floats: vertices=[...,1,0,0], normals=[...,1,0,0]
static void appendVertex(TessellatedFace& face, Vec3 pos, Vec3 nor)
{
    face.vertices.push_back((float)pos.x);
    face.vertices.push_back((float)pos.y);
    face.vertices.push_back((float)pos.z);
    face.normals.push_back((float)nor.x);
    face.normals.push_back((float)nor.y);
    face.normals.push_back((float)nor.z);
}

// appends a single triangle (three vertex indices) to the index list, ex: appendTri(face, 0, 1, 2) -> face.indices = [..., 0, 1, 2]
static void appendTri(TessellatedFace& face, int a, int b, int c)
{
    face.indices.push_back(a);
    face.indices.push_back(b);
    face.indices.push_back(c);
}

// generic UV grid: positionFn(u,v) and normalFn(u,v) lambdas, u in [u0,u1] x v in [v0,v1]
// emits front-facing triangles (CCW from outside), backface culling disabled at draw time
TessellatedFace tessGrid(SurfaceKind kind, std::function<Vec3(double u, double v)> positionFn, std::function<Vec3(double u, double v)> normalFn,
    double u0, double u1, int uSteps, double v0, double v1, int vSteps)
{
    TessellatedFace face;
    face.kind = kind;
    // row stride in the flat vertex array: one row = uSteps+1 vertices
    // ex: uSteps=3 -> stride=4; vertex at grid cell (ui=2, vi=1) is at flat index 1*4+2 = 6
    int stride = uSteps + 1;
    // front vertices: grid of (uSteps+1)*(vSteps+1) points sampled at each (u,v)
    for (int vi = 0; vi <= vSteps; vi++)
        for (int ui = 0; ui <= uSteps; ui++) {
            double u = u0 + (u1 - u0) * (double)ui / uSteps, v = v0 + (v1 - v0) * (double)vi / vSteps;
            appendVertex(face, positionFn(u, v), normalFn(u, v));
        }
    // frontVertexCount = total front vertex count; back vertices start at index frontVertexCount
    // ex: uSteps=3, vSteps=2 -> frontVertexCount=4*3=12; back verts are indices 12 to 23
    int frontVertexCount = (uSteps + 1) * (vSteps + 1);
    // back vertices: same positions as front but normals flipped for inside rendering
    for (int vi = 0; vi <= vSteps; vi++)
        for (int ui = 0; ui <= uSteps; ui++) {
            double u = u0 + (u1 - u0) * (double)ui / uSteps, v = v0 + (v1 - v0) * (double)vi / vSteps;
            appendVertex(face, positionFn(u, v), normalFn(u, v) * -1.0);
        }
    // emit two triangles per quad cell (CCW front, CW back)
    // quad corners: bottomLeft, bottomRight, topLeft, topRight
    // ex: vi=0, ui=0, stride=4 -> bottomLeft=0, bottomRight=1, topLeft=4, topRight=5
    //     front tris: (0,4,5) and (0,5,1)   back tris: (12,17,16) and (12,13,17)
    for (int vi = 0; vi < vSteps; vi++)
        for (int ui = 0; ui < uSteps; ui++) {
            int bottomLeft = vi * stride + ui, bottomRight = bottomLeft + 1, topLeft = (vi + 1) * stride + ui, topRight = topLeft + 1;
            appendTri(face, bottomLeft, topLeft, topRight);
            appendTri(face, bottomLeft, topRight, bottomRight); // front (CCW)
            appendTri(face, bottomLeft + frontVertexCount, topRight + frontVertexCount, topLeft + frontVertexCount);
            appendTri(face, bottomLeft + frontVertexCount, bottomRight + frontVertexCount, topRight + frontVertexCount); // back (reversed)
        }
    return face;
}

static TessellatedFace tessCylinder(const Surface& surface, const std::vector<BoundaryLoop>& loops, int uSegments)
{
    Vec3 origin = surface.axis.origin, Z = surface.axis.zDir.norm(), X = surface.axis.xDir.norm();
    // Y = Z x X gives the third axis of the local coordinate frame (right-hand rule)
    // ex: Z=(0,0,1), X=(1,0,0) -> Y=(0,1,0)
    Vec3 Y = Z.cross(X).norm();
    double radius = surface.majorRadius;

    // compute height range [heightMin, heightMax] by projecting all boundary points onto the cylinder axis
    // ex: screw shank with top boundary at h=10 and bottom at h=0 -> heightMin=0, heightMax=10
    double heightMin = 1e18, heightMax = -1e18;
    bool fullRevolution = false;
    for (auto& loop : loops) {
        if (loop.hasFullCircle)
            fullRevolution = true;
        for (auto& point : loop.points) {
            double height = (point - origin).dot(Z);
            heightMin = std::min(heightMin, height);
            heightMax = std::max(heightMax, height);
        }
    }
    if (heightMax - heightMin < 1e-8)
        return {};

    // angle range: full revolution or derive from boundary points
    double angleMin = 0, angleMax = 2 * M_PI;
    int uCount = uSegments;
    if (!fullRevolution) {
        // project each boundary point onto the XY plane and compute its angle
        // ex: point at (radius,0,h) -> angle=0;  point at (0,radius,h) -> angle=pi/2
        double rawAngleMin = 1e18, rawAngleMax = -1e18;
        for (auto& loop : loops)
            for (auto& point : loop.points) {
                Vec3 delta = point - origin;
                double angle = std::atan2(delta.dot(Y), delta.dot(X));
                rawAngleMin = std::min(rawAngleMin, angle);
                rawAngleMax = std::max(rawAngleMax, angle);
            }
        // if the arc span covers almost the full circle, treat as full revolution
        // to avoid the gap caused by atan2 wrapping near +-pi
        // ex: rawAngleMin=-3.1, rawAngleMax=3.1 -> span=6.2 > 1.9*pi=5.97 -> full revolution
        if (rawAngleMax - rawAngleMin > 1.9 * M_PI) {
            angleMin = 0;
            angleMax = 2 * M_PI;
        } else {
            angleMin = rawAngleMin;
            angleMax = rawAngleMax;
        }
        // scale segment count to the actual arc fraction of a full circle.
        // ex: uSegments=48, half-circle arc (pi out of 2pi) -> uCount = max(2, 48*0.5) = 24
        uCount = std::max(2, (int)(uSegments * (angleMax - angleMin) / (2 * M_PI)));
    }

    // cylinder surface: positionFn(u,v) = origin + radius*(cos(u)*X + sin(u)*Y) + v*Z
    // u = angle around axis [angleMin to angleMax], v = height along axis [heightMin to heightMax]
    auto positionFn = [&](double u, double v) -> Vec3 {
        Vec3 radialDir = X * std::cos(u) + Y * std::sin(u);
        return origin + radialDir * radius + Z * v;
    };
    // outward radial direction (same for all v at a given u),  ex: u=0 -> normal=(1,0,0);  u=pi/2 -> normal=(0,1,0)
    auto normalFn = [&](double u, double v) -> Vec3 {
        (void)v;
        return (X * std::cos(u) + Y * std::sin(u));
    };
    return tessGrid(SurfaceKind::Cylinder, positionFn, normalFn, angleMin, angleMax, uCount, heightMin, heightMax, 1);
}

static TessellatedFace tessTorus(const Surface& surface, int uSegments, int vSegments = 24)
{
    Vec3 origin = surface.axis.origin, Z = surface.axis.zDir.norm(), X = surface.axis.xDir.norm();
    Vec3 Y = Z.cross(X).norm();
    double majorRadius = surface.majorRadius, minorRadius = surface.minorRadius;

    // torus surface: positionFn(u,v) = origin + (majorRadius + minorRadius*cos(v))*(cos(u)*X + sin(u)*Y) + minorRadius*sin(v)*Z
    // u = angle around the major circle [0 to 2pi], v = angle around the tube cross-section [0 to 2pi]
    // ex: u=0, v=0 -> outermost equator point at origin+(majorRadius+minorRadius)*X
    //     u=0, v=pi ->  innermost equator point at origin+(majorRadius-minorRadius)*X
    auto positionFn = [&](double u, double v) -> Vec3 {
        Vec3 radialDir = X * std::cos(u) + Y * std::sin(u);
        return origin + radialDir * (majorRadius + minorRadius * std::cos(v)) + Z * (minorRadius * std::sin(v));
    };
    // outward normal: radial component in XY plane scaled by cos(v), plus axial Z*sin(v)
    // ex: v=0 (outer equator) -> normal = radialDir (purely radial outward)
    //     v=pi/2 (top of tube) ->  normal = Z         (purely axial upward)
    auto normalFn = [&](double u, double v) -> Vec3 {
        Vec3 radialDir = X * std::cos(u) + Y * std::sin(u);
        return (radialDir * std::cos(v) + Z * std::sin(v)).norm();
    };
    return tessGrid(SurfaceKind::Torus, positionFn, normalFn, 0, 2 * M_PI, uSegments, 0, 2 * M_PI, vSegments);
}

#pragma region plane tessellation
// boundary-driven + ear-clip with holes
static TessellatedFace tessPlane(const Surface& surface, std::vector<BoundaryLoop>& loops, bool faceReversed)
{
    TessellatedFace face;
    face.kind = SurfaceKind::Plane;

    Vec3 normal = surface.axis.zDir.norm();
    // STEP's FACE orientation flag: reversed face means the surface normal points inward
    // ex: bottom face of a box has zDir pointing up, but face is reversed -> normal flipped to point down
    if (faceReversed)
        normal = normal * -1.0;
    Vec3 tangentX = surface.axis.xDir.norm();
    // if xDir is degenerate (near-zero), build an arbitrary tangent frame from normal
    // ex: normal=(0,1,0) -> pick tangentX=(1,0,0), Gram-Schmidt -> tangentX stays (1,0,0)
    //     normal=(1,0,0) -> pick tangentX=(0,1,0), Gram-Schmidt -> tangentX stays (0,1,0)
    if (tangentX.len() < 0.5) {
        tangentX = (std::abs(normal.x) < 0.9) ? Vec3 { 1, 0, 0 } : Vec3 { 0, 1, 0 };
        // Gram-Schmidt: remove the component of tangentX along normal to guarantee perpendicularity
        // ex: tangentX=(1,0,0), normal=(0.6,0.8,0) -> tangentX -= 0.6*(0.6,0.8,0) = (0.64,-0.48,0), then normalized
        tangentX = (tangentX - normal * tangentX.dot(normal)).norm();
    }
    // tangentY completes the right-hand orthonormal frame (tangentX, tangentY, normal)
    // ex: normal=(0,0,1), tangentX=(1,0,0) -> tangentY = (0,0,1) x (1,0,0) = (0,1,0)
    Vec3 tangentY = normal.cross(tangentX).norm();

    // separate outer and inner (hole) loops
    BoundaryLoop* outerLoop = nullptr;
    std::vector<BoundaryLoop*> innerLoops;
    for (auto& loop : loops) {
        if (loop.isOuter)
            outerLoop = &loop;
        else
            innerLoops.push_back(&loop);
    }
    if (!outerLoop && !loops.empty()) {
        // fallback: treat largest-area loop as outer (STEP files occasionally mislabel)
        double bestArea = -1e18;
        for (auto& loop : loops) {
            std::vector<Vec2> projected;
            Vec3 centroid = { 0, 0, 0 };
            for (auto& point : loop.points)
                centroid = centroid + point;
            centroid = centroid * (1.0 / loop.points.size());
            for (auto& point : loop.points) {
                Vec3 delta = point - centroid;
                projected.push_back({ delta.dot(tangentX), delta.dot(tangentY) });
            }
            double loopArea = std::abs(area2D(projected));
            if (loopArea > bestArea) {
                bestArea = loopArea;
                outerLoop = &loop;
            }
        }
    }
    if (!outerLoop)
        return face;

    // compute centroid of all loop points as the 2D projection origin
    // centering the 2D coords reduces floating-point error in the ear-clipper
    // (coords near zero rather than at large absolute positions like 150.003)
    Vec3 centroid = { 0, 0, 0 };
    int pointCount = 0;
    for (auto& loop : loops)
        for (auto& point : loop.points) {
            centroid = centroid + point;
            pointCount++;
        }
    if (pointCount)
        centroid = centroid * (1.0 / pointCount);

    // project 3D loop points onto the plane's local (tangentX, tangentY) tangent frame
    // ex: point=(3,0,1), centroid=(1,0,1), tangentX=(1,0,0), tangentY=(0,1,0) -> u=2, v=0
    auto projectTo2D = [&](std::vector<Vec3>& points3D) -> std::vector<Vec2> {
        std::vector<Vec2> projected;
        for (auto& point : points3D) {
            Vec3 delta = point - centroid;
            projected.push_back({ delta.dot(tangentX), delta.dot(tangentY) });
        }
        return projected;
    };

    // build outer and hole 2D polygons
    std::vector<Vec2> outerPoly = projectTo2D(outerLoop->points);
    // ensure outer polygon is CCW, the GPU uses this to determine which side of a triangle is the front face,
    // if you mix them up, normals point the wrong way and faces disappear or are lit backwards
    // the ear-clipper must work in a consistent winding, and hole boundaries must be wound opposite to the outer boundary so the algorithm can tell them apart
    // ex: area2D = -5 -> polygon is CW -> reverse to make it CCW
    if (area2D(outerPoly) < 0) {
        std::reverse(outerPoly.begin(), outerPoly.end());
        std::reverse(outerLoop->points.begin(), outerLoop->points.end());
        outerPoly = projectTo2D(outerLoop->points);
    }

    std::vector<std::vector<Vec2>> holePolygons;
    for (auto* innerLoop : innerLoops) {
        auto holePoly = projectTo2D(innerLoop->points);
        // holes must be CW (opposite winding to outer)
        // ex: bolt hole boundary sampled CCW (area > 0) -> reverse to CW so bridge merge works correctly
        if (area2D(holePoly) > 0) {
            std::reverse(holePoly.begin(), holePoly.end());
            std::reverse(innerLoop->points.begin(), innerLoop->points.end());
            holePoly = projectTo2D(innerLoop->points);
        }
        holePolygons.push_back(holePoly);
    }

    // build merged polygon (O'Rourke bridge cuts, sorted rightmost-first)
    // use merged polygon vertices DIRECTLY as GPU verts, reconstruct 3D from 2D
    std::vector<Vec2> mergedPoly = buildMergedPolygon(outerPoly, holePolygons);
    auto triangles = earClip(mergedPoly);
    if (triangles.empty())
        return face;

    int mergedVertexCount = (int)mergedPoly.size();
    // small offset along the normal to avoid z-fighting when two coplanar faces overlap
    // ex: top cap of a cylinder sits flush against a flat face at the same Z -> offset by 0.0005 units
    Vec3 offsetVec = normal * 5e-4;

    // reconstruct 3D positions from 2D: pos3D = centroid + tangentX*u + tangentY*v
    // ex: u=2, v=1, centroid=(0,0,5), tangentX=(1,0,0), tangentY=(0,1,0) -> pos3D=(2,1,5)
    // emit front and back vertex sets with opposite normals for double-sided rendering
    for (auto& vertex2D : mergedPoly) {
        Vec3 pos3D = centroid + tangentX * vertex2D.u + tangentY * vertex2D.v;
        appendVertex(face, pos3D + offsetVec, normal);
    }
    for (auto& vertex2D : mergedPoly) {
        Vec3 pos3D = centroid + tangentX * vertex2D.u + tangentY * vertex2D.v;
        appendVertex(face, pos3D + offsetVec, normal * -1.0);
    }
    for (auto& triangle : triangles)
        appendTri(face, triangle[0], triangle[1], triangle[2]);
    // reversed winding for back face
    for (auto& triangle : triangles)
        appendTri(face, triangle[0] + mergedVertexCount, triangle[2] + mergedVertexCount, triangle[1] + mergedVertexCount);
    return face;
}

#pragma region advanced_face
//  if outSurface non-null, the resolved surface is written out for callers that need the analytical definition after tessellation
TessellatedFace tessellateAdvancedFace(int faceId, const StepMap& map, int arcSegs, Surface* outSurface)
{
    auto faceIt = map.find(faceId);
    if (faceIt == map.end())
        return {};
    // ADVANCED_FACE params: (name, (bound_refs...), surface_ref, orientation_flag)
    // ex: "('', (#200,#201), #202, .T.)" means no name, boundary is described by two loops (the outer rim and an inner hole, entities #200 and #201),
    // underlying surface geometry is entity #202 (which might be a CYLINDRICAL_SURFACE)
    // .T. (true) means the normal direction matches the surface's own normal (not flipped)
    auto faceParams = splitTopLevel(faceIt->second.params);
    if (faceParams.size() < 3)
        return {};

    // "what kind of surface is this and where is it in space"
    Surface surface = resolveSurface(stepRef(faceParams[2]), map);
    // orientation_flag ".F." means the face normal is opposite to the surface normal
    // ex: inner wall of a hollow cylinder has ".F." so the normal points inward
    bool faceReversed = (faceParams.size() >= 4 && trimWS(faceParams[3]) == ".F.");

    if (outSurface)
        *outSurface = surface;

    // sample all boundary loops, converts the topological boundary description (a chain of edge references)
    // into an actual ordered list of 3D points forming a closed polygon aka "what is the outline of this face"
    std::vector<BoundaryLoop> loops;
    for (auto& boundRef : splitTopLevel(unwrap(trimWS(faceParams[1])))) {
        int boundId = stepRef(trimWS(boundRef));
        BoundaryLoop loop = sampleLoop(boundId, map, arcSegs);
        if (loop.points.size() >= 2)
            loops.push_back(std::move(loop));
    }

    // takes the surface type + that outline and produces triangles = "how do I fill this face with geometry the GPU can render"
    switch (surface.kind) {
    case SurfaceKind::Cylinder:
        return tessCylinder(surface, loops, arcSegs);
    case SurfaceKind::Torus:
        return tessTorus(surface, arcSegs, 24); // 24 tube segments is sufficient for fillet quality
    default:
        return tessPlane(surface, loops, faceReversed);
    }
}
#pragma region gpu upload
// sums the area of all front-face triangles (first half of the index list, back-face duplicates are the second half)
// exact for planar faces (ear-clip triangulates the exact boundary), approximate for curved surfaces (depends on arcSegs)
float computeFaceArea(const TessellatedFace& face)
{
    float area = 0.0f;
    // total triangles / 2 = front-face triangles only (back faces are duplicated with reversed winding in the second half)
    int frontTriCount = (int)face.indices.size() / 6;
    for (int i = 0; i < frontTriCount; i++) {
        int base = i * 3;
        int i0 = face.indices[base] * 3, i1 = face.indices[base + 1] * 3, i2 = face.indices[base + 2] * 3;
        Vec3 v0 = { face.vertices[i0], face.vertices[i0 + 1], face.vertices[i0 + 2] };
        Vec3 v1 = { face.vertices[i1], face.vertices[i1 + 1], face.vertices[i1 + 2] };
        Vec3 v2 = { face.vertices[i2], face.vertices[i2 + 1], face.vertices[i2 + 2] };
        area += 0.5f * (float)(v1 - v0).cross(v2 - v0).len(); // triangle area formula
    }
    return area;
}

// allocates and fills a Raylib Mesh from a TessellatedFace, then uploads it from CPU RAM to the GPU's to draw it later
// ex: face with 6 verts and 4 tris -> mesh.vertices=float[18], mesh.indices=ushort[12]
Mesh uploadMesh(const TessellatedFace& tessellatedFace)
{
    Mesh mesh = {};
    if (tessellatedFace.indices.empty())
        return mesh;
    int vertexCount = (int)tessellatedFace.vertices.size() / 3, triangleCount = (int)tessellatedFace.indices.size() / 3;
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
    mesh.vertices = (float*)RL_MALLOC(vertexCount * 3 * sizeof(float));
    mesh.normals = (float*)RL_MALLOC(vertexCount * 3 * sizeof(float));
    // raylib uses unsigned short indices: max 65535 vertices per mesh
    // ex: a dense torus with uSegments=48, vSegments=24 -> 2*49*25=2450 verts, well within the limit
    mesh.indices = (unsigned short*)RL_MALLOC(triangleCount * 3 * sizeof(unsigned short));
    memcpy(mesh.vertices, tessellatedFace.vertices.data(), vertexCount * 3 * sizeof(float));
    memcpy(mesh.normals, tessellatedFace.normals.data(), vertexCount * 3 * sizeof(float));
    for (int i = 0; i < (int)tessellatedFace.indices.size(); i++)
        mesh.indices[i] = (unsigned short)tessellatedFace.indices[i];
    // dynamic = false tells the GPU to put the mesh in static memory (VRAM that is optimized for read-many, write-never)
    // true would put it in memory that is cheaper to update each frame, for things like skinned meshes or particle systems that change every frame
    // STEP geometry never changes after load so go false
    UploadMesh(&mesh, false);
    return mesh;
}

#pragma region geometry healing
// retessellates the GPU mesh of a cylinder face with updated [heightMin, heightMax] and re-uploads it
// the CPU pickData is also replaced so ray picking and stats remain accurate
void retessCylinderFace(CadModel& model, int cylIdx, double newHeightMin, double newHeightMax)
{
    const Surface& surf = model.faceSurfaces[cylIdx];
    Vec3 origin = surf.axis.origin, Z = surf.axis.zDir.norm(), X = surf.axis.xDir.norm();
    Vec3 Y = Z.cross(X).norm();
    double radius = surf.majorRadius;

    // reconstruct the angle range from the existing pickData (we cannot re-run sampleLoop without the StepMap)
    // project all front-face vertices onto the XY plane of the cylinder and recover the angle sweep
    const TessellatedFace& oldFace = model.pickData[cylIdx];
    int frontVertexCount = (int)oldFace.vertices.size() / 6;
    double rawAngleMin = 1e18, rawAngleMax = -1e18;
    for (int vi = 0; vi < frontVertexCount; vi++) {
        int base = vi * 3;
        Vec3 pt = { oldFace.vertices[base], oldFace.vertices[base + 1], oldFace.vertices[base + 2] };
        Vec3 delta = pt - origin;
        double angle = std::atan2(delta.dot(Y), delta.dot(X));
        rawAngleMin = std::min(rawAngleMin, angle);
        rawAngleMax = std::max(rawAngleMax, angle);
    }
    double angleMin = rawAngleMin, angleMax = rawAngleMax;
    // same full-revolution detection as tessCylinder, span > 1.9*pi -> treat as full circle
    if (rawAngleMax - rawAngleMin > 1.9 * M_PI) {
        angleMin = 0;
        angleMax = 2 * M_PI;
    }
    // reconstruct uCount from the original tessellation, vertex count per row = uCount+1, total front rows = 2 (vSteps=1)
    // frontVertexCount = (uCount+1) * (1+1), so uCount = frontVertexCount/2 - 1
    int uCount = std::max(2, frontVertexCount / 2 - 1);

    auto positionFn = [&](double u, double v) -> Vec3 {
        Vec3 radialDir = X * std::cos(u) + Y * std::sin(u);
        return origin + radialDir * radius + Z * v;
    };
    auto normalFn = [&](double u, double v) -> Vec3 {
        (void)v;
        return (X * std::cos(u) + Y * std::sin(u));
    };
    TessellatedFace newFace = tessGrid(SurfaceKind::Cylinder, positionFn, normalFn, angleMin, angleMax, uCount, newHeightMin, newHeightMax, 1);

    // unload the old GPU mesh and upload the new one
    UnloadMesh(model.meshes[cylIdx]);
    model.meshes[cylIdx] = uploadMesh(newFace);
    model.pickData[cylIdx] = newFace;
    model.faceAreas[cylIdx] = computeFaceArea(newFace);
    model.cylHeightRanges[cylIdx] = { newHeightMin, newHeightMax };
}

// checks if two faces share at least one vertex within eps (front-face vertices only)
// used to confirm a plane and a cylinder are topologically connected at a cap, planeOff and cylOff are the current draw-space offsets of each face
// could use our vec3.near but would be slightly worse because would calls sqrt per pair when all we need is to compare distance
// not actually get them (so we dont need to sqrt)
static bool facesShareVertex(const TessellatedFace& planeData, Vector3 planeOff, const TessellatedFace& cylData, Vector3 cylOff, float eps = 0.05f)
{
    int planeCount = (int)planeData.vertices.size() / 6;
    int cylCount = (int)cylData.vertices.size() / 6;
    float epsSq = eps * eps;
    for (int i = 0; i < planeCount; i++) {
        int baseP = i * 3;
        float px = planeData.vertices[baseP] + planeOff.x;
        float py = planeData.vertices[baseP + 1] + planeOff.y;
        float pz = planeData.vertices[baseP + 2] + planeOff.z;
        for (int j = 0; j < cylCount; j++) {
            int baseC = j * 3;
            float cx = cylData.vertices[baseC] + cylOff.x;
            float cy = cylData.vertices[baseC + 1] + cylOff.y;
            float cz = cylData.vertices[baseC + 2] + cylOff.z;
            float dx = px - cx, dy = py - cy, dz = pz - cz;
            if (dx * dx + dy * dy + dz * dz < epsSq)
                return true;
        }
    }
    return false;
}

// builds the heal cache for the selected plane face at gesture start (rising edge of the translation)
// scans all cylinders once, checks axis alignment and vertex adjacency at the current (initial) position,
// records which cap each connected cylinder should extend so per-frame updates need no proximity work at all
std::vector<CylinderHealEntry> buildCylinderHealCache(const CadModel& model, int planeFaceIdx)
{
    std::vector<CylinderHealEntry> cache;
    if (planeFaceIdx < 0 || planeFaceIdx >= (int)model.faceSurfaces.size())
        return cache;
    const Surface& planeSurf = model.faceSurfaces[planeFaceIdx];
    if (planeSurf.kind != SurfaceKind::Plane)
        return cache;

    Vec3 planeNormal = planeSurf.axis.zDir.norm();
    const Vector3& planeOff = model.faceOffsets[planeFaceIdx];

    // compute plane centroid in STEP space once (no offset, raw vertex data)
    Vec3 planeCent = { 0, 0, 0 };
    int planeCount = (int)model.pickData[planeFaceIdx].vertices.size() / 6;
    for (int vi = 0; vi < planeCount; vi++) {
        int base = vi * 3;
        planeCent.x += model.pickData[planeFaceIdx].vertices[base];
        planeCent.y += model.pickData[planeFaceIdx].vertices[base + 1];
        planeCent.z += model.pickData[planeFaceIdx].vertices[base + 2];
    }
    if (planeCount > 0) {
        double inv = 1.0 / planeCount;
        planeCent = { planeCent.x * inv, planeCent.y * inv, planeCent.z * inv };
    }

    for (int ci = 0; ci < (int)model.faceSurfaces.size(); ci++) {
        if (model.faceSurfaces[ci].kind != SurfaceKind::Cylinder)
            continue;
        const Surface& cylSurf = model.faceSurfaces[ci];
        Vec3 cylAxis = cylSurf.axis.zDir.norm();

        // axis alignment, plane normal and cylinder axis must be parallel (same or opposite direction)
        double dotVal = planeNormal.dot(cylAxis);
        if (std::abs(dotVal) < 0.99)
            continue;

        // adjacency check done here once at gesture start, plane is still at its initial position
        if (!facesShareVertex(model.pickData[planeFaceIdx], planeOff, model.pickData[ci], model.faceOffsets[ci]))
            continue;

        // determine cap by projecting the plane centroid (raw STEP position + current draw-space offset) onto the
        // cylinder axis, then comparing to the height midpoint, the offset must be included because a prior heal
        // may have extended heightMax, shifting the midpoint so that the raw centroid alone would pick the wrong cap
        Vec3 toCent = { planeCent.x - cylSurf.axis.origin.x, planeCent.y - cylSurf.axis.origin.y, planeCent.z - cylSurf.axis.origin.z };
        double planeOffDot = planeOff.x * cylAxis.x + planeOff.y * cylAxis.y + planeOff.z * cylAxis.z;
        double planeProjH = toCent.dot(cylAxis) + planeOffDot;
        const CylinderHeightRange& chr = model.cylHeightRanges[ci];
        bool isMaxCap = (planeProjH >= (chr.heightMin + chr.heightMax) * 0.5);

        cache.push_back({ ci, isMaxCap, dotVal });
    }
    return cache;
}

// applies the cached heal entries for one frame, extends each recorded cylinder cap by the axial component of delta
// no proximity scan because the cache is stable for the entire gesture duration
// when the plane crosses through the opposite cap (height range would invert), min/max are swapped and isMaxCap
// is flipped so the cylinder mirrors correctly and subsequent frames continue healing from the new orientation
void applyCylinderHealCache(CadModel& model, std::vector<CylinderHealEntry>& cache, Vec3 planeNormal, Vector3 delta)
{
    double axialDelta = planeNormal.x * delta.x + planeNormal.y * delta.y + planeNormal.z * delta.z;
    if (std::abs(axialDelta) < 1e-8)
        return;
    for (auto& entry : cache) {
        double signedDelta = axialDelta * entry.axisDotNormal;
        CylinderHeightRange& chr = model.cylHeightRanges[entry.cylFaceIdx];
        double newHeightMin = chr.heightMin, newHeightMax = chr.heightMax;
        if (entry.isMaxCap)
            newHeightMax = chr.heightMax + signedDelta;
        else
            newHeightMin = chr.heightMin + signedDelta;
        // when the moving cap crosses through the fixed cap the range inverts:
        // swap min and max so the cylinder reflects through its fixed cap, and flip isMaxCap so the
        // same plane direction continues shrinking/growing the correct (now-opposite) cap next frame
        // ex: plane pushes top cap down past the bottom -> range [0,5] becomes [5,0] -> swap to [0,5] reflected, flip cap
        if (newHeightMax < newHeightMin) {
            std::swap(newHeightMin, newHeightMax);
            entry.isMaxCap = !entry.isMaxCap;
        }
        retessCylinderFace(model, entry.cylFaceIdx, newHeightMin, newHeightMax);
    }
}
