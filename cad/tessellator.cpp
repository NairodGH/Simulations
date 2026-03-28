#include "cad.hpp"

// appends a single vertex (position + normal) to the flat float arrays of a TessellatedFace
// ex: pos=(1,0,0), normal=(1,0,0) -> pushes 6 floats: vertices=[...,1,0,0], normals=[...,1,0,0]
static void appendVertex(TessellatedFace& face, Vec3 pos, Vec3 normal)
{
    face.vertices.push_back((float)pos.x);
    face.vertices.push_back((float)pos.y);
    face.vertices.push_back((float)pos.z);
    face.normals.push_back((float)normal.x);
    face.normals.push_back((float)normal.y);
    face.normals.push_back((float)normal.z);
}

// appends a single triangle (three vertex indices) to the index list, ex: appendTri(face, 0, 1, 2) -> face.indices = [..., 0, 1, 2]
static void appendTri(TessellatedFace& face, int a, int b, int c)
{
    face.indices.push_back(a);
    face.indices.push_back(b);
    face.indices.push_back(c);
}

#pragma region 2D ear-clip
// imagine a polygon drawn on paper, an "ear" is any three consecutive vertices where the middle one can be cut off cleanly
// the triangle formed by those three vertices lies entirely inside the polygon and contains no other vertices, ear-clipping just repeatedly
// - finds one of those ears (3 adjacent points forming a triangle)
// - emits the triangle
// - removes the middle vertex
// - repeats by finding a new triangle using the 2 remaining points and a new adjacent one
// however you gotta watch out for reflex vertices, in a "convex" polygon (all inner angles under 180° aka the classic ones),
// a "reflex vertex" describes the specific vertex where a concavity happens, if it's the middle vertex then the triangle we make will be outside of the
// plane so we skip it and make sure it's never a middle vertex and only clip ears at convex ones it is done in 2D because the faces are flat planes, you
// project the 3D boundary down to a flat coordinate system (the plane's own UV tangent frame), triangulate there where the math is simple, then lift the result
// back to 3D

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
    // shoelace gives 2*area, divide to get the true signed area
    return area * 0.5;
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
    // memcpy into uint64_t is defined behaviour and compiles to a zero-cost register move (no actual memory touch) and IEEE 754 already stores the sign of a
    // double in last bit 63, packing those sign bits into a 3-bit mask lets us just use them at once instead of doing an ugly
    // !((cross1 < 0 || cross2 < 0 || cross3 < 0) && (cross1 > 0 || cross2 > 0 || cross3 > 0))
    // ex:cross1=+1.0 -> b1=0x3FF0..., b1>>63 = 0 (positive)
    //    cross2=-0.5 -> b2=0xBFE0..., b2>>63 = 1 (negative)
    //    mask = 0b010 -> mixed signs -> point is outside
    uint64_t bit1, bit2, bit3;
    memcpy(&bit1, &cross1, 8);
    memcpy(&bit2, &cross2, 8);
    memcpy(&bit3, &cross3, 8);
    int mask = (int)(bit1 >> 63) << 2 | (int)(bit2 >> 63) << 1 | (int)(bit3 >> 63);
    // mask == 0b000 = all positive (CCW winding, point is inside)
    // mask == 0b111 = all negative (CW winding, point is inside)
    // anything else = mixed signs (point is outside)
    return mask == 0b000 || mask == 0b111;
}

// ear-clip a simple polygon, returns triangle indices into 'polygon'
static std::vector<std::array<int, 3>> earClip(const std::vector<Vec2>& polygon)
{
    std::vector<std::array<int, 3>> triangles;
    int vertexCount = (int)polygon.size();
    if (vertexCount < 3)
        return triangles;
    // working index ring: we'll remove vertices from this as ears are clipped
    // ex: polygon with 5 verts -> remaining={0,1,2,3,4}, clip ear at 1 -> remaining={0,2,3,4}
    std::list<int> remaining;
    for (int i = 0; i < vertexCount; i++)
        remaining.push_back(i);
    // ensure CCW winding so the convexity test (cross > 0) is consistent
    std::vector<Vec2> tmp(polygon.begin(), polygon.end());
    if (area2D(tmp) < 0)
        remaining.reverse();

    // vertexCount*vertexCount*2 iterations: more headroom for self-touching bridge-seam polygons
    // a plain convex polygon needs only vertexCount-2 clips; the *2+64 cushions degenerate flat ears
    // ex: vertexCount=10 -> maxIterations=264; vertexCount=100 -> maxIterations=20064
    int maxIterations = vertexCount * vertexCount * 2 + 64, iterationCount = 0;
    while ((int)remaining.size() > 3 && iterationCount++ < maxIterations) {
        int ringSize = (int)remaining.size();
        bool earFound = false;
        for (auto i = remaining.begin(); i != remaining.end(); ++i) {
            // find prev and next iterators with wraparound
            auto prevId = (i == remaining.begin()) ? std::prev(remaining.end()) : std::prev(i);
            auto nextId = std::next(i);
            if (nextId == remaining.end())
                nextId = remaining.begin();
            int idA = *prevId, idB = *i, idC = *nextId;
            double crossProduct = cross2D(polygon[idA], polygon[idB], polygon[idC]);
            // skip reflex vertices (CW turn, crossProduct < 0); allow near-zero (flat) ears since
            // bridge seams create collinear triples that must still be clipped
            if (crossProduct < -1e-10)
                continue;

            bool isEar = true;
            for (auto j = remaining.begin(); j != remaining.end() && isEar; ++j) {
                if (j == prevId || j == i || j == nextId)
                    continue;
                Vec2 candidate = polygon[*j];
                // skip vertices geometrically coincident (same position) with ear vertices using epsilon near check
                const double coincidentEpsilon = 1e-9;
                if ((std::abs(candidate.u - polygon[idA].u) < coincidentEpsilon && std::abs(candidate.v - polygon[idA].v) < coincidentEpsilon)
                    || (std::abs(candidate.u - polygon[idB].u) < coincidentEpsilon && std::abs(candidate.v - polygon[idB].v) < coincidentEpsilon)
                    || (std::abs(candidate.u - polygon[idC].u) < coincidentEpsilon && std::abs(candidate.v - polygon[idC].v) < coincidentEpsilon))
                    continue;
                if (pointIn2DTriangle(candidate, polygon[idA], polygon[idB], polygon[idC]))
                    isEar = false;
            }
            if (isEar) {
                // only emit non-degenerate (non-zero-area) triangles
                // ex: crossProduct=1e-15 (collinear bridge seam triple) -> skip, no triangle emitted
                if (crossProduct > 1e-14) {
                    std::array<int, 3> triangle = { idA, idB, idC };
                    triangles.push_back(triangle);
                }
                // O(1) erase via iterator — the whole reason we use std::list here
                remaining.erase(i);
                earFound = true;
                break;
            }
        }
        if (!earFound)
            break;
    }
    // earclipper clips until this last remaining triangle we add manually
    if ((int)remaining.size() == 3) {
        auto i = remaining.begin();
        int idA = *i++, idB = *i++, idC = *i;
        double crossProduct = cross2D(polygon[idA], polygon[idB], polygon[idC]);
        if (crossProduct > 1e-14)
            triangles.push_back({ idA, idB, idC });
    }
    return triangles;
}

// O'Rourke visible-vertex bridge algorithm for hole merging:
// we need to connect a hole to the outer polygon without the connection line crossing any existing edge
// - start from the rightmost point of the hole and shoot a ray to the right
// - find which outer polygon edge the ray hits first
// - of the two endpoints of that edge, pick the one further right, that vertex is visible from your hole point by construction (the ray guarantees it)
// - draw the bridge there

// ex: outer square [A, B, C, D] and a triangular hole [H0, H1, H2] where B is the chosen bridge target and H0 is the chosen hole start,
// the merged polygon becomes [A, B, H0, H1, H2, H0, B, C, D], B and H0 appear twice (the seams), B->H0 and H0->B are the bridge

// when you have a polygon with a hole in it, the ear-clipper can only handle simple polygons (no holes),
// a bridge cut is a straight line segment that connects a point on the hole boundary to a point on the outer boundary,
// now it has no hole, just a slightly more complex outline that the ear-clipper can handle normally,
// since it produce degenerate zero-area ears (the two duplicate vertices are collinear), the ear-clipper skips without emitting a triangle for them

// given the current (possibly already-merged) outer polygon and the +X ray origin rayOrigin,
// returns the index of the polygon vertex that is the best bridge target (the vertex of the first edge hit by the ray that has the larger X coordinate)
// ex: rayOrigin=(3,1), outer square [(0,0),(5,0),(5,3),(0,3)]
//     ray hits edge (5,0)-(5,3) at x=5 -> both endpoints have same U, pick (5,3) via >= check
static int findBridgeVertex(const std::vector<Vec2>& polygon, Vec2 rayOrigin)
{
    int vertexCount = (int)polygon.size();
    double closestDistance = std::numeric_limits<double>::max();
    int bestEdgeId = -1;

    for (int i = 0; i < vertexCount; i++) {
        Vec2 edgeStart = polygon[i], edgeEnd = polygon[(i + 1) % vertexCount];
        // only consider edges that straddle rayOrigin's V coordinate (can be hit by horizontal ray)
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
        // distanceAlongRay is the signed distance along +X from rayOrigin to the hit, skip edges to the left
        // ex: rayOrigin.u=3, xHit=5 -> distanceAlongRay=+2 (to the right, valid hit)
        //     rayOrigin.u=3, xHit=1 -> distanceAlongRay=-2 (to the left, skip)
        double distanceAlongRay = xHit - rayOrigin.u;
        if (distanceAlongRay < -1e-9)
            continue;
        if (distanceAlongRay < closestDistance) {
            closestDistance = distanceAlongRay;
            bestEdgeId = i;
        }
    }

    if (bestEdgeId < 0) {
        // fallback: nearest vertex (handles degenerate geometry where no edge is hit)
        double bestDistSq = std::numeric_limits<double>::max();
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
    int idA = bestEdgeId, idB = (bestEdgeId + 1) % vertexCount;
    return (polygon[idA].u >= polygon[idB].u) ? idA : idB;
}

// merge hole polygons into the outer polygon via bridge cuts (O'Rourke method),
// sort holes rightmost-first so each bridge is inserted before any hole to its left
// ex: the plane donuts at the start/end of our LeadScrew = outer circle + 1 hole -> returns one merged polygon with a cut
static std::vector<Vec2> buildMergedPolygon(const std::vector<Vec2>& outer, const std::vector<std::vector<Vec2>>& holes)
{
    if (holes.empty())
        return outer;

    // sort holes by their rightmost X coordinate (descending), this ensures bridge cuts from right-side holes don't cross left-side holes
    // ex: two holes with rightmost X=8 and X=3 -> process X=8 hole first
    std::vector<int> holeOrder((int)holes.size());
    std::iota(holeOrder.begin(), holeOrder.end(), 0);
    // pre-compute rightmost U per hole so the comparator doesn't re-walk each hole on every comparison
    std::vector<double> rightmostU(holes.size(), -std::numeric_limits<double>::max());
    for (int holeId = 0; holeId < (int)holes.size(); holeId++)
        for (auto& vertex : holes[holeId])
            rightmostU[holeId] = std::max(rightmostU[holeId], vertex.u);
    std::sort(holeOrder.begin(), holeOrder.end(), [&](int holeA, int holeB) { return rightmostU[holeA] > rightmostU[holeB]; });

    std::vector<Vec2> merged = outer;

    for (int holeId : holeOrder) {
        const auto& hole = holes[holeId];
        if (hole.empty())
            continue;

        // find rightmost point of this hole (the bridge starts here)
        int rightmostId = 0;
        for (int i = 1; i < (int)hole.size(); i++)
            if (hole[i].u > hole[rightmostId].u)
                rightmostId = i;
        Vec2 bridgeSource = hole[rightmostId];

        // O'Rourke, find visible bridge vertex on the outer polygon using ray casting
        int bridgeTargetId = findBridgeVertex(merged, bridgeSource);

        // splice hole into merged polygon at bridgeTargetId:
        // outer[0 to bridgeTargetId] -> hole[rightmostId to rightmostId-1] -> hole[rightmostId] (seam) -> outer[bridgeTargetId to end]
        // the duplicated vertices are what create the "bridge cut", see example comment above findBridgeVertex
        std::vector<Vec2> spliced;
        spliced.reserve(merged.size() + hole.size() + 2);
        for (int i = 0; i <= bridgeTargetId; i++)
            spliced.push_back(merged[i]);
        for (int i = 0; i < (int)hole.size(); i++)
            spliced.push_back(hole[(rightmostId + i) % hole.size()]);
        spliced.push_back(hole[rightmostId]); // close hole seam
        spliced.push_back(merged[bridgeTargetId]); // bridge back to outer
        for (int i = bridgeTargetId + 1; i < (int)merged.size(); i++)
            spliced.push_back(merged[i]);
        merged = std::move(spliced);
    }

    // remove consecutive near-duplicate vertices (bridge seam creates pairs like [A, A]), keeping them causes zero-area ears that stall the ear-clipper
    // ex: [..., B, B, ...] -> [..., B, ...] (one of the two bridge-seam duplicates removed)
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

#pragma region plane tessellation
// boundary-driven + ear-clip with holes
static TessellatedFace tessPlane(const Surface& surface, std::vector<BoundaryLoop>& loops, bool faceReversed)
{
    TessellatedFace face;
    face.kind = SurfaceKind::Plane;

    Vec3 normal = surface.axis.zDirection.norm();
    // STEP's FACE orientation flag: reversed face means the surface normal points inward
    // ex: bottom face of a box has zDirection pointing up, but face is reversed -> normal flipped to point down
    if (faceReversed)
        normal = normal * -1.0;
    Vec3 tangentX = surface.axis.xDirection.norm();
    // if xDirection is degenerate (near-zero), build an arbitrary tangent frame from normal
    // ex: normal=(0,1,0) -> pick tangentX=(1,0,0), Gram-Schmidt -> tangentX stays (1,0,0)
    //     normal=(1,0,0) -> pick tangentX=(0,1,0), Gram-Schmidt -> tangentX stays (0,1,0)
    if (tangentX.len() < 0.5) {
        tangentX = (std::abs(normal.x) < 0.9) ? Vec3 { 1, 0, 0 } : Vec3 { 0, 1, 0 };
        // remove the component of tangentX along normal to guarantee perpendicularity (Gram-Schmidt, guarantee 90 degree angle)
        // ex: tangentX=(1,0,0), normal=(0.6,0.8,0) -> tangentX -= 0.6*(0.6,0.8,0) = (0.64,-0.48,0), then normalized
        tangentX = (tangentX - normal * tangentX.dot(normal)).norm();
    }
    // tangentY completes our coordinate system (tangentX, tangentY, normal)
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
    // imagine you have a set of fence outlines on a map but none of them are labeled "outer boundary", you can find the outer fence by measuring
    // which enclosed area is the largest, assuming the property boundary is always bigger than any holes it contains
    if (!outerLoop && !loops.empty()) {
        // fallback: no fence was labeled as the property boundary, find it by picking the largest enclosed area
        double bestArea = -std::numeric_limits<double>::max();
        for (auto& loop : loops) {
            // project loop points onto the face plane to measure 2D area
            std::vector<Vec2> projected;
            Vec3 centroid = { 0, 0, 0 };
            for (auto& point : loop.points)
                centroid = centroid + point;
            centroid = centroid * (1.0 / loop.points.size());
            for (auto& point : loop.points) {
                Vec3 delta = point - centroid;
                projected.push_back({ delta.dot(tangentX), delta.dot(tangentY) });
            }
            // measure how much land this fence encloses
            double loopArea = std::abs(area2D(projected));
            // largest fence so far = most likely the property boundary
            if (loopArea > bestArea) {
                bestArea = loopArea;
                outerLoop = &loop;
            }
        }
    }
    if (!outerLoop)
        return face;

    // computes the average position of all points across all loops (centroid of the face), then uses that as the origin for the 2D projection,
    // instead of projecting raw STEP coordinates like 150.003 200.007 you project(0.003 0.007 relative to the centroid (the ear clipper does a lot of
    // subtractions and cross products on those coordinates and working near zero avoids losing all precision when dealing with big numbers)
    Vec3 centroid = { 0, 0, 0 };
    int pointCount = 0;
    for (auto& loop : loops)
        for (auto& point : loop.points) {
            centroid = centroid + point;
            pointCount++;
        }
    if (pointCount)
        centroid = centroid * (1.0 / pointCount);

    // project 3D loop points onto the plane's local coordinate system
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
    // the ear-clipper must work in a consistent winding, and holes boundaries must be wound opposite to the outer boundary so the algorithm can tell them apart
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

    // build merged polygon (O'Rourke bridge cuts, sorted rightmost-first), use merged polygon vertices directly as GPU verts, reconstruct 3D from 2D
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
    for (auto& vertex2D : mergedPoly) {
        Vec3 pos3D = centroid + tangentX * vertex2D.u + tangentY * vertex2D.v;
        appendVertex(face, pos3D + offsetVec, normal);
    }
    for (auto& vertex2D : mergedPoly) {
        Vec3 pos3D = centroid + tangentX * vertex2D.u + tangentY * vertex2D.v;
        appendVertex(face, pos3D + offsetVec, normal * -1.0);
    }
    // emit front and back vertex sets with opposite normals for double-sided rendering
    for (auto& triangle : triangles)
        appendTri(face, triangle[0], triangle[1], triangle[2]);
    // reversed winding for back face ()
    for (auto& triangle : triangles)
        appendTri(face, triangle[0] + mergedVertexCount, triangle[2] + mergedVertexCount, triangle[1] + mergedVertexCount);
    return face;
}

#pragma region uv grid
// U and V are just parameter names for the two axes of a surface's own coordinate system, the same way X/Y are axes in world space,
// a cylinder has U = angle around the axis (0 to 2π for a full revolution) and V = height along it (heightMin to heightMax),
// imagine you need to wallpaper a cylindrical pillar, just unroll a rectangular sheet and wrap it around, that's the UV grid
// projecting a cylinder boundary onto a flat plane would give two overlapping circles, the ear-clipper would produce garbage,
// instead we tessellate the surface analytically in (u,v) parameter space and emit a clean quad grid
// "analytically" here means the normal or position is computed from the closed-form mathematical definition of the surface
// rather than estimated by, say, averaging the normals of surrounding triangles (tldr exact formula, no approximation)
// inversely, we couldn't use this amazing UV grid for planes because it does not have a natural parameter space like cylinder's and toruses's
// angle and height, a plane is infinite and has no intrinsic coordinate system (you have to invent one, tangentX/tangentY) and
// its boundary shape can be absolutely anything, hence why a recursive exploratory algorithm like ear-clipper is better

// generic UV grid: positionFn(u,v) and normalFn(u,v) lambdas, u in [u0,u1] x v in [v0,v1]
// emits front-facing triangles (CCW from outside), backface culling disabled at draw time
TessellatedFace tessGrid(SurfaceKind kind, std::function<Vec3(double u, double v)> positionFn, std::function<Vec3(double u, double v)> normalFn, double u0,
    double u1, int uSteps, double v0, double v1, int vSteps)
{
    TessellatedFace face;
    face.kind = kind;
    assert(uSteps > 0 && vSteps > 0); // so that we never ever divide by 0 later
    // row stride in the flat vertex array: one row = uSteps+1 vertices
    // ex: uSteps=3 -> stride=4; vertex at grid cell (ui=2, vId=1) is at flat index 1*4+2 = 6
    int stride = uSteps + 1;

    // cache front-vertex positions and normals so the back-vertex pass can reuse them without calling positionFn/normalFn again
    // (avoids duplicate sin/cos for cylinders and toruses which compute radial direction inside both fns)
    std::vector<Vec3> frontPos;
    frontPos.reserve((uSteps + 1) * (vSteps + 1));
    std::vector<Vec3> frontNormals;
    frontNormals.reserve((uSteps + 1) * (vSteps + 1));

    // front vertices: grid of (uSteps+1)*(vSteps+1) points sampled at each (u,v)
    for (int vId = 0; vId <= vSteps; vId++)
        for (int uId = 0; uId <= uSteps; uId++) {
            double u = u0 + (u1 - u0) * (double)uId / uSteps, v = v0 + (v1 - v0) * (double)vId / vSteps;
            Vec3 pos = positionFn(u, v);
            Vec3 normal = normalFn(u, v);
            frontPos.push_back(pos);
            frontNormals.push_back(normal);
            appendVertex(face, pos, normal);
        }
    // frontVertexCount = total front vertex count; back vertices start at index frontVertexCount
    // ex: uSteps=3, vSteps=2 -> frontVertexCount=4*3=12; back verts are indices 12 to 23
    int frontVertexCount = (uSteps + 1) * (vSteps + 1);
    // back vertices reuse cached front positions and normals, only flip the normal for inside rendering
    for (int vId = 0; vId <= vSteps; vId++)
        for (int uId = 0; uId <= uSteps; uId++)
            appendVertex(face, frontPos[vId * stride + uId], frontNormals[vId * stride + uId] * -1.0);
    // emit two triangles per quad cell (CCW front, CW back)
    // quad corners: bottomLeft, bottomRight, topLeft, topRight
    // ex: vId=0, uId=0, stride=4 -> bottomLeft=0, bottomRight=1, topLeft=4, topRight=5
    //     front tris: (0,4,5) and (0,5,1)   back tris: (12,17,16) and (12,13,17)
    for (int vId = 0; vId < vSteps; vId++)
        for (int uId = 0; uId < uSteps; uId++) {
            int bottomLeft = vId * stride + uId, bottomRight = bottomLeft + 1, topLeft = (vId + 1) * stride + uId, topRight = topLeft + 1;
            appendTri(face, bottomLeft, topLeft, topRight);
            appendTri(face, bottomLeft, topRight, bottomRight); // front (CCW)
            appendTri(face, bottomLeft + frontVertexCount, topRight + frontVertexCount, topLeft + frontVertexCount);
            appendTri(face, bottomLeft + frontVertexCount, bottomRight + frontVertexCount, topRight + frontVertexCount); // back (reversed)
        }
    return face;
}

// shared cylinder surface evaluation, reused by both tessCylinder and retessCylinderFace so the math lives in one place
// positionFn: origin + radius*(cos(u)*X + sin(u)*Y) + v*Z, same formula sampleCircle uses for boundary arcs
// normalFn: outward radial direction (same for all v at a given u), ex: u=0 -> (1,0,0), u=pi/2 -> (0,1,0)
static auto cylPositionFn(Vec3 origin, Vec3 X, Vec3 Y, Vec3 Z, double radius)
{
    return [=](double u, double v) -> Vec3 { return origin + (X * std::cos(u) + Y * std::sin(u)) * radius + Z * v; };
}
static auto cylNormalFn(Vec3 X, Vec3 Y)
{
    return [=](double u, [[maybe_unused]] double v) -> Vec3 { return X * std::cos(u) + Y * std::sin(u); };
}

static TessellatedFace tessCylinder(
    const Surface& surface, const std::vector<BoundaryLoop>& loops, int uSegments, CylinderHeightRange* outHeightRange = nullptr)
{
    Vec3 origin = surface.axis.origin, Z = surface.axis.zDirection.norm(), X = surface.axis.xDirection.norm();
    // Y = Z x X gives the third axis of the local coordinate frame (right-hand rule)
    // ex: Z=(0,0,1), X=(1,0,0) -> Y=(0,1,0)
    Vec3 Y = Z.cross(X).norm();
    double radius = surface.majorRadius;

    // compute height range [heightMin, heightMax] by projecting all boundary points onto the cylinder axis
    // ex: screw shank with top boundary at h=10 and bottom at h=0 -> heightMin=0, heightMax=10
    double heightMin = std::numeric_limits<double>::max(), heightMax = -std::numeric_limits<double>::max();
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
        return { };
    // pass height range back to the caller so loadStep doesn't need to recompute it from vertices
    if (outHeightRange)
        *outHeightRange = { heightMin, heightMax };

    // angle range, full revolution or derive from boundary points
    double angleMin = 0, angleMax = 2 * M_PI;
    int uCount = uSegments;
    if (!fullRevolution) {
        // project each boundary point onto the XY plane and compute its angle
        // ex: point at (radius,0,h) -> angle=0;  point at (0,radius,h) -> angle=pi/2
        double rawAngleMin = std::numeric_limits<double>::max(), rawAngleMax = -std::numeric_limits<double>::max();
        for (auto& loop : loops)
            for (auto& point : loop.points) {
                Vec3 delta = point - origin;
                double angle = std::atan2(delta.dot(Y), delta.dot(X));
                rawAngleMin = std::min(rawAngleMin, angle);
                rawAngleMax = std::max(rawAngleMax, angle);
            }
        // if the arc span covers almost the full circle, treat as full revolution to avoid the gap caused by atan2 wrapping near +-pi
        // ex: rawAngleMin=-3.1, rawAngleMax=3.1 -> span=6.2 > 1.9*pi=5.97 -> full revolution
        if (rawAngleMax - rawAngleMin > 1.9 * M_PI) {
            angleMin = 0;
            angleMax = 2 * M_PI;
        } else {
            angleMin = rawAngleMin;
            angleMax = rawAngleMax;
        }
        // scale segment count to the actual arc fraction of a full circle
        // ex: uSegments=48, half-circle arc (pi out of 2pi) -> uCount = max(2, 48*0.5) = 24
        uCount = std::max(2, (int)(uSegments * (angleMax - angleMin) / (2 * M_PI)));
    }

    // cylinder surface: u = angle around axis, v = height along axis
    return tessGrid(SurfaceKind::Cylinder, cylPositionFn(origin, X, Y, Z, radius), cylNormalFn(X, Y), angleMin, angleMax, uCount, heightMin, heightMax, 1);
}

static TessellatedFace tessTorus(const Surface& surface, int uSegments, int vSegments = 24)
{
    Vec3 origin = surface.axis.origin, Z = surface.axis.zDirection.norm(), X = surface.axis.xDirection.norm();
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
    //     v=pi/2 (top of tube) -> normal = Z (purely axial upward)
    auto normalFn = [&](double u, double v) -> Vec3 {
        Vec3 radialDir = X * std::cos(u) + Y * std::sin(u);
        return (radialDir * std::cos(v) + Z * std::sin(v)).norm();
    };
    return tessGrid(SurfaceKind::Torus, positionFn, normalFn, 0, 2 * M_PI, uSegments, 0, 2 * M_PI, vSegments);
}

// allocates and fills a Raylib Mesh from a TessellatedFace, then uploads it from CPU RAM to the GPU's to draw it later
// ex: face with 6 verts and 4 tris -> mesh.vertices=float[18], mesh.indices=ushort[12]
Mesh uploadMesh(const TessellatedFace& tessellatedFace)
{
    Mesh mesh = { };
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
// retessellates the GPU mesh of a cylinder face with updated [heightMin, heightMax] and re-uploads it,
// the CPU cpuFaceData is also replaced so ray picking and stats remain accurate
void retessCylinderFace(CadModel& model, int cylId, double newHeightMin, double newHeightMax)
{
    // zero or negative height produces coplanar triangles with undefined normals (degenerate mesh),
    // this can happen when simultaneous two-sided compression (symetry constraint) meets in a single step, clamp before any geometry work
    if (newHeightMax - newHeightMin < 1e-3)
        newHeightMax = newHeightMin + 1e-3;
    const Surface& surface = model.faceSurfaces[cylId];
    Vec3 origin = surface.axis.origin, Z = surface.axis.zDirection.norm(), X = surface.axis.xDirection.norm();
    Vec3 Y = Z.cross(X).norm();
    double radius = surface.majorRadius;

    // imagine a manual clock, each vertex is a clock stick, delta.dot(X)/delta.dot(Y) are already its (X,Y)
    // position on the clock face (= cos/sin of its angle), we never need to label it as "37 degrees"
    // just to sort it, we sort hands by position directly and only produce degree-labels at the very end
    const TessellatedFace& oldFace = model.cpuFaceData[cylId];
    int frontVertexCount = (int)oldFace.vertices.size() / 6;
    // read the clock stick, record each vertex as its (X,Y) position on the unit circle
    // (delta.dot(X)*invRadius, delta.dot(Y)*invRadius) = where the stick points on the clock face, no label (atan2) needed yet
    std::vector<std::pair<double, double>> projections;
    projections.reserve(frontVertexCount);
    double invRadius = (radius > 1e-14) ? 1.0 / radius : 1.0;
    for (int vId = 0; vId < frontVertexCount; vId++) {
        int base = vId * 3;
        Vec3 pt = { oldFace.vertices[base], oldFace.vertices[base + 1], oldFace.vertices[base + 2] };
        Vec3 delta = pt - origin;
        // the stick's position on the clock face, dot(X)*invRadius is cos(theta), dot(Y)*invRadius is sin(theta), no atan2 required
        projections.push_back({ delta.dot(X) * invRadius, delta.dot(Y) * invRadius });
    }
    // remove duplicate sticks pointing at the same spot before sorting
    std::sort(projections.begin(), projections.end());
    projections.erase(std::unique(projections.begin(), projections.end(),
                          [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
                              return std::abs(a.first - b.first) < 1e-9 && std::abs(a.second - b.second) < 1e-9;
                          }),
        projections.end());
    // sort sticks by position without labelling them, upper half of the clock face (second >= 0, ex: 12 o'clock through 3 to 6) comes before lower half
    // (6 through 9 to 12), and within each half sticks are ordered by their X position so the full sequence sweeps CCW around the face
    // ex: (1,0)=3 o'clock, (0,1)=12 o'clock -> both upper half, X=1 > X=0 -> 3 o'clock before 12 (CCW: 0->90deg)
    std::sort(projections.begin(), projections.end(), [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
        bool aUpper = a.second >= 0, bUpper = b.second >= 0;
        if (aUpper != bUpper)
            return aUpper; // upper half before lower
        return aUpper ? (a.first > b.first) : (a.first < b.first);
    });
    double angleMin = 0, angleMax = 2 * M_PI;
    if (!projections.empty()) {
        double biggestGap = 0;
        int gapAfter = 0;
        for (int i = 0; i < (int)projections.size(); i++) {
            // find the biggest empty stretch of clock face (the arc that has no sticks in it),
            // atan2 is called here on adjacent pairs only to measure gap sizes, not on every vertex
            auto& cur = projections[i];
            auto& next = projections[(i + 1) % (int)projections.size()];
            double aCur = std::atan2(cur.second, cur.first);
            double aNext = std::atan2(next.second, next.first);
            double gap = aNext - aCur;
            if (gap <= 0)
                gap += 2 * M_PI; // wrap around midnight
            if (gap > biggestGap) {
                biggestGap = gap;
                gapAfter = i;
            }
        }
        if (biggestGap > 1.9 * M_PI) {
            angleMin = 0;
            angleMax = 2 * M_PI;
        } else {
            // label the two sticks that border the empty stretch, those are the arc endpoints, atan2 is called twice (gapEnd and gapStart) to produce the
            // degree-labels tessGrid needs, the whole sort above never needed labels at all
            auto& gapEnd = projections[(gapAfter + 1) % (int)projections.size()];
            auto& gapStart = projections[gapAfter];
            angleMin = std::atan2(gapEnd.second, gapEnd.first);
            angleMax = std::atan2(gapStart.second, gapStart.first);
            while (angleMax <= angleMin)
                angleMax += 2 * M_PI;
        }
    }
    // reconstruct uCount from the original tessellation, vertex count per row = uCount+1, total front rows = 2 (vSteps=1)
    // frontVertexCount = (uCount+1) * (1+1), so uCount = frontVertexCount/2 - 1
    int uCount = std::max(2, frontVertexCount / 2 - 1);

    auto positionFn = cylPositionFn(origin, X, Y, Z, radius);
    auto normalFn = cylNormalFn(X, Y);
    TessellatedFace newFace = tessGrid(SurfaceKind::Cylinder, positionFn, normalFn, angleMin, angleMax, uCount, newHeightMin, newHeightMax, 1);

    // unload the old GPU mesh and upload the new one
    UnloadMesh(model.meshes[cylId]);
    model.meshes[cylId] = uploadMesh(newFace);
    model.cpuFaceData[cylId] = newFace;
    model.faceAreas[cylId] = computeFaceArea(newFace);
    model.rawFaceCentroids[cylId] = computeRawFaceCentroid(newFace);
    model.cylHeightRanges[cylId] = { newHeightMin, newHeightMax };
}

// checks if two faces share at least one vertex within eps (front-face vertices only)
// used to confirm a plane and a cylinder are topologically connected at a cap, planeOff and cylOff are the current draw-space offsets of each face
// could use our vec3.near but would be slightly worse because would calls sqrt per pair when all we need is to compare distance
// not actually get them (so we dont need to sqrt)
// back vertices are always exactly as many as front and stored contiguously after them (tessGrid and tessPlane both emit front then back),
// so front_vertex_count = vertices.size()/3 / 2 = vertices.size()/6, base = i*3 then addresses XYZ within the front half only
// if this invariant ever breaks, planeCount/cylCount will be wrong
static bool facesShareVertex(const TessellatedFace& planeData, Vector3 planeOff, const TessellatedFace& cylData, Vector3 cylOff, float eps = 0.05f)
{
    int planeCount = (int)planeData.vertices.size() / 6;
    int cylCount = (int)cylData.vertices.size() / 6;
    float epsilon = eps * eps;
    for (int i = 0; i < planeCount; i++) {
        int baseP = i * 3;
        float planeX = planeData.vertices[baseP] + planeOff.x;
        float planeY = planeData.vertices[baseP + 1] + planeOff.y;
        float planeZ = planeData.vertices[baseP + 2] + planeOff.z;
        for (int j = 0; j < cylCount; j++) {
            int baseC = j * 3;
            float distanceX = planeX - (cylData.vertices[baseC] + cylOff.x);
            float distanceY = planeY - (cylData.vertices[baseC + 1] + cylOff.y);
            float distanceZ = planeZ - (cylData.vertices[baseC + 2] + cylOff.z);
            // the sphere (your current best distance) fits inside a cube whose side equals the diameter, if you're already outside the cube on any single wall
            // then you're outside the sphere, checking one wall costs one abs
            // (which is cheaper than even a multiplication because it's just zeroing one bit of the float)
            if (std::abs(distanceX) >= eps || std::abs(distanceY) >= eps || std::abs(distanceZ) >= eps)
                continue;
            if (distanceX * distanceX + distanceY * distanceY + distanceZ * distanceZ < epsilon)
                return true;
        }
    }
    return false;
}

// builds the heal cache for the selected plane face at gesture start, scans all cylinders once, checks axis alignment and vertex adjacency at the current
// (initial) position, records which cap each connected cylinder should extend so per-frame updates need no proximity work at all
std::vector<CylinderHealEntry> buildCylinderHealCache(const CadModel& model, int planeFaceId)
{
    std::vector<CylinderHealEntry> cache;
    if (planeFaceId < 0 || planeFaceId >= (int)model.faceSurfaces.size())
        return cache;
    const Surface& planeSurf = model.faceSurfaces[planeFaceId];
    if (planeSurf.kind != SurfaceKind::Plane)
        return cache;

    Vec3 planeNormal = planeSurf.axis.zDirection.norm();
    const Vector3& planeOff = model.faceOffsets[planeFaceId];

    // compute plane centroid in STEP space once (no offset, raw vertex data)
    Vec3 planeCent = { 0, 0, 0 };
    int planeCount = (int)model.cpuFaceData[planeFaceId].vertices.size() / 6;
    for (int vId = 0; vId < planeCount; vId++) {
        int base = vId * 3;
        planeCent.x += model.cpuFaceData[planeFaceId].vertices[base];
        planeCent.y += model.cpuFaceData[planeFaceId].vertices[base + 1];
        planeCent.z += model.cpuFaceData[planeFaceId].vertices[base + 2];
    }
    if (planeCount > 0) {
        double inv = 1.0 / planeCount;
        planeCent = { planeCent.x * inv, planeCent.y * inv, planeCent.z * inv };
    }

    for (int cylId = 0; cylId < (int)model.faceSurfaces.size(); cylId++) {
        if (model.faceSurfaces[cylId].kind != SurfaceKind::Cylinder)
            continue;
        const Surface& cylSurf = model.faceSurfaces[cylId];
        Vec3 cylAxis = cylSurf.axis.zDirection.norm();

        // axis alignment, plane normal and cylinder axis must be parallel (same or opposite direction)
        double axisAlignment = planeNormal.dot(cylAxis);
        if (std::abs(axisAlignment) < 0.99)
            continue;

        // project the plane centroid (raw STEP position + current draw-space offset) onto the cylinder axis,
        // the offset must be included because a prior heal may have extended heightMax, shifting the midpoint
        Vec3 toCentroid = { planeCent.x - cylSurf.axis.origin.x, planeCent.y - cylSurf.axis.origin.y, planeCent.z - cylSurf.axis.origin.z };
        double planeOffset = cylAxis.dot(planeOff);
        double planeProjectionHeight = toCentroid.dot(cylAxis) + planeOffset;
        const CylinderHeightRange& cylHeightRange = model.cylHeightRanges[cylId];

        // adjacency: plane centroid must project onto the cylinder axis at (or very near) one of the two caps,
        // this is robust to retessellation because it uses the analytical axis projection rather than vertex
        // proximity, facesShareVertex fails after a phase-through because retessCylinderFace emits uniformly-
        // sampled cap ring vertices that no longer coincide with the STEP-sampled plane boundary vertices,
        // axialEps is generous to survive float accumulation over many heal frames (cap height drifts slightly each frame)
        const double axialEps = 0.5; // in model units
        if (std::abs(planeProjectionHeight - cylHeightRange.heightMin) >= axialEps && std::abs(planeProjectionHeight - cylHeightRange.heightMax) >= axialEps)
            continue;

        // at least one vertex of the plane must lie within the cylinder's cross-section, per-vertex is safe here because this runs only
        // once per gesture start and planeOff is purely axial so STEP-space vertex positions are sufficient,
        // only accept vertices whose radial distance from the cylinder axis is near the cylinder's own surface
        const double radialUpperEps = 0.05; // in model units
        const double radialLowerEps = 0.10; // slightly wider lower margin to tolerate retessellation snap on the inner edge
        bool anyVertexInCrossSection = false;
        for (int vId = 0; vId < planeCount && !anyVertexInCrossSection; vId++) {
            int base = vId * 3;
            Vec3 toVert = { (double)model.cpuFaceData[planeFaceId].vertices[base] - cylSurf.axis.origin.x,
                (double)model.cpuFaceData[planeFaceId].vertices[base + 1] - cylSurf.axis.origin.y,
                (double)model.cpuFaceData[planeFaceId].vertices[base + 2] - cylSurf.axis.origin.z };
            Vec3 radialVert = toVert - cylAxis * toVert.dot(cylAxis);
            double radialDist = radialVert.len();
            if (radialDist >= cylSurf.majorRadius - radialLowerEps && radialDist <= cylSurf.majorRadius + radialUpperEps)
                anyVertexInCrossSection = true;
        }
        if (!anyVertexInCrossSection)
            continue;

        // pick the cap whose current axial position is closer to the plane's current projection,
        // closer-cap is unambiguous even when both caps have drifted asymmetrically after healing or a phase-through,
        // midpoint comparison breaks when one cap has moved far and the midpoint no longer sits between the two planes
        bool isMaxCap = (std::abs(planeProjectionHeight - cylHeightRange.heightMax) <= std::abs(planeProjectionHeight - cylHeightRange.heightMin));

        cache.push_back({ cylId, isMaxCap, axisAlignment });
    }
    return cache;
}

// resets the axial drift that accumulates between gestures, writes the plane's exact current projection directly into cylHeightRanges for every cylinder
// in the cache, so the next buildCylinderHealCache starts from zero drift regardless of how many gestures preceded this one, called once at gesture start after
// buildCylinderHealCache
void snapCylinderHealCache(CadModel& model, const std::vector<CylinderHealEntry>& cache, int planeFaceId)
{
    if (planeFaceId < 0 || planeFaceId >= (int)model.faceSurfaces.size())
        return;
    const Surface& planeSurf = model.faceSurfaces[planeFaceId];
    if (planeSurf.kind != SurfaceKind::Plane)
        return;

    Vec3 planeNormal = planeSurf.axis.zDirection.norm();
    const Vector3& planeOff = model.faceOffsets[planeFaceId];

    // recompute the plane centroid from current CPU vertex data (same as buildCylinderHealCache)
    Vec3 planeCent = { 0, 0, 0 };
    int planeCount = (int)model.cpuFaceData[planeFaceId].vertices.size() / 6;
    for (int vId = 0; vId < planeCount; vId++) {
        int base = vId * 3;
        planeCent.x += model.cpuFaceData[planeFaceId].vertices[base];
        planeCent.y += model.cpuFaceData[planeFaceId].vertices[base + 1];
        planeCent.z += model.cpuFaceData[planeFaceId].vertices[base + 2];
    }
    if (planeCount > 0) {
        double inv = 1.0 / planeCount;
        planeCent = { planeCent.x * inv, planeCent.y * inv, planeCent.z * inv };
    }

    for (const auto& entry : cache) {
        const Surface& cylSurf = model.faceSurfaces[entry.cylFaceId];
        Vec3 cylAxis = cylSurf.axis.zDirection.norm();
        Vec3 toCentroid = { planeCent.x - cylSurf.axis.origin.x, planeCent.y - cylSurf.axis.origin.y, planeCent.z - cylSurf.axis.origin.z };
        double planeOffset = cylAxis.dot(planeOff);
        double exactProjection = toCentroid.dot(cylAxis) + planeOffset;

        CylinderHeightRange& range = model.cylHeightRanges[entry.cylFaceId];
        // overwrite the cap this entry owns with the plane's exact current projection, zeroing any drift
        if (entry.isMaxCap)
            range.heightMax = exactProjection;
        else
            range.heightMin = exactProjection;
    }
}

// applies the cached heal entries for one frame, extends each recorded cylinder cap by the axial component of delta
// no proximity scan because the cache is stable for the entire gesture duration
// when the plane crosses through the opposite cap (height range would invert), min/max are swapped and isMaxCap
// is flipped so the cylinder mirrors correctly and subsequent frames continue healing from the new orientation
void applyCylinderHealCache(CadModel& model, std::vector<CylinderHealEntry>& cache, Vec3 planeNormal, Vector3 delta)
{
    double axialDelta = planeNormal.dot(delta);
    if (std::abs(axialDelta) < 1e-8)
        return;
    for (auto& entry : cache) {
        double signedDelta = axialDelta * entry.axisDotNormal;
        CylinderHeightRange& cylHeightRange = model.cylHeightRanges[entry.cylFaceId];
        double newHeightMin = cylHeightRange.heightMin, newHeightMax = cylHeightRange.heightMax;
        if (entry.isMaxCap)
            newHeightMax = cylHeightRange.heightMax + signedDelta;
        else
            newHeightMin = cylHeightRange.heightMin + signedDelta;
        // ex: plane pushes top cap down past the bottom -> range [0,5] becomes [5,0] -> swap to [0,5] reflected, flip cap
        // <= with a small epsilon so the inversion also fires when the caps meet exactly
        if (newHeightMax <= newHeightMin + 1e-6) {
            std::swap(newHeightMin, newHeightMax);
            entry.isMaxCap = !entry.isMaxCap;
        }
        retessCylinderFace(model, entry.cylFaceId, newHeightMin, newHeightMax);
    }
}

#pragma region tessellate face
TessellatedFace tessellateAdvancedFace(int faceId, const StepMap& map, int arcSegs, Surface* outSurface, CylinderHeightRange* outHeightRange)
{
    auto faceIt = map.find(faceId);
    if (faceIt == map.end())
        return { };
    // ADVANCED_FACE params: (name, (bound_refs...), surface_ref, orientation_flag)
    // ex: "( 'NONE', ( #1565, #837, #1004, #1757, #2982, #765 ), #2444, .T. )" means
    // - boundary is described by 6 refs, for our LeadScrew this corresponds to the 2 big faces with 1 outer edge + 4 inner edges for toruses
    // + 1 inner edge for main hole = 6, 2 others have 2 refs because they're the ring at the start/end, and all the others have 1 ref because they're
    // continuous shapes with no hole (only "outer" edge, even if cylinder or torus)
    // - underlying surface geometry is entity #202 (here a plane)
    // - .T. (true) means the normal direction matches the surface's own normal (not flipped)
    auto faceParams = splitTopLevel(faceIt->second.params);
    if (faceParams.size() < 3)
        return { };

    // "what kind of surface is this and where is it in space"
    Surface surface = resolveSurface(stepRef(faceParams[2]), map);
    // orientation_flag ".F." means the face normal is opposite to the surface normal
    // ex: inner wall of a hollow cylinder has ".F." so the normal points inward
    bool faceReversed = (faceParams.size() >= 4 && trimWS(faceParams[3]) == ".F.");

    if (outSurface)
        *outSurface = surface; // for callers thatwant it after

    // sample all boundary loops, converts the topological boundary description (a chain of edge references)
    // into an actual ordered list of 3D points forming a closed polygon aka "what is the outline of this face"
    std::vector<BoundaryLoop> loops;
    for (auto& boundRef : splitTopLevel(unwrap(trimWS(faceParams[1])))) {
        int boundId = stepRef(trimWS(boundRef));
        BoundaryLoop loop = sampleLoop(boundId, map, arcSegs);
        if (loop.points.size() >= 2)
            loops.push_back(std::move(loop));
    }

    // takes the surface type + that outline and produces triangles = "how do I fill this face with triangles the GPU can render"
    switch (surface.kind) {
    case SurfaceKind::Plane:
        return tessPlane(surface, loops, faceReversed);
    case SurfaceKind::Cylinder:
        return tessCylinder(surface, loops, arcSegs, outHeightRange);
    case SurfaceKind::Torus:
        return tessTorus(surface, arcSegs, arcSegs / 2);
    default:
        // unknown surface type, return empty rather than silently tessellating with the wrong model
        return { };
    }
}

#pragma region gpu upload
// sums the area of all front-face triangles (first half of the index list, back-face duplicates are the second half)
// exact for planar faces (ear-clip triangulates the exact boundary), approximate for curved surfaces (depends on arcSegs)
float computeFaceArea(const TessellatedFace& face)
{
    // accumulate in double to reduce error on large faces, cast to float only at the final return
    double area = 0.0;
    // total triangles / 2 = front-face triangles only (back faces are duplicated with reversed winding in the second half)
    int frontTriCount = (int)face.indices.size() / 6;
    for (int i = 0; i < frontTriCount; i++) {
        int base = i * 3;
        int i0 = face.indices[base] * 3, i1 = face.indices[base + 1] * 3, i2 = face.indices[base + 2] * 3;
        Vec3 A = { face.vertices[i0], face.vertices[i0 + 1], face.vertices[i0 + 2] };
        Vec3 B = { face.vertices[i1], face.vertices[i1 + 1], face.vertices[i1 + 2] };
        Vec3 C = { face.vertices[i2], face.vertices[i2 + 1], face.vertices[i2 + 2] };
        area += 0.5 * (B - A).cross(C - A).len(); // shoelace triangle area formula
    }
    return (float)area;
}