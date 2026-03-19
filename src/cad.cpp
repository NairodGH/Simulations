#include "cad.hpp"

#pragma region step parser
// accumulates multi-line entities until ';', extracts #ID = TYPE(params)

// splits a comma-separated parameter string at the top level only, commas inside nested parentheses are ignored, ex: "A,(B,C),D" -> ["A", "(B,C)", "D"]
// needed because STEP params can contain nested lists like coordinate tuples
static std::vector<std::string> splitTopLevel(const std::string& input)
{
    std::vector<std::string> result;
    int depth = 0;
    std::string current;
    for (char c : input) {
        if (c == '(') {
            depth++;
            current += c;
        } else if (c == ')') {
            depth--;
            current += c;
        } else if (c == ',' && !depth) {
            result.push_back(current);
            current.clear();
        } else
            current += c;
    }
    if (!current.empty())
        result.push_back(current);
    return result;
}

// strips leading/trailing whitespace (spaces, tabs, CR, LF) from a string, ex: "  #12 \t" -> "#12"
static std::string trimWS(const std::string& input)
{
    auto first = input.find_first_not_of(" \t\r\n"), last = input.find_last_not_of(" \t\r\n");
    return first == std::string::npos ? "" : input.substr(first, last - first + 1);
}

// strips the outermost parentheses from a string, used to peel the argument list off a STEP entity before splitting it, ex: "(A,B)" -> "A,B"
static std::string unwrap(const std::string& input)
{
    auto open = input.find('('), close = input.rfind(')');
    return (open == std::string::npos || close == std::string::npos) ? input : input.substr(open + 1, close - open - 1);
}

// parses a STEP entity reference to its numeric ID, returns -1 if not a ref, ex: " #123 " -> 123, "0.5" -> -1
static int stepRef(const std::string& input)
{
    auto trimmed = trimWS(input);
    return (!trimmed.empty() && trimmed[0] == '#') ? std::stoi(trimmed.substr(1)) : -1;
}

// converts a trimmed string token to a double, returns 0 on parse failure, ex: " 3.14 " -> 3.14,   ".F." -> 0.0
static double dbl(const std::string& input)
{
    try {
        return std::stod(trimWS(input));
    } catch (...) {
        return 0.0;
    }
}

StepMap parseStepFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot open: " + path);
    StepMap entityMap;
    std::string line, accum;
    bool inData = false;
    // matches a complete STEP entity line after accum flushes at ';':
    //   group 1 = numeric ID after '#', ex: "12" in "#12 = ..."
    //   group 2 = entity type keyword, ex: "CARTESIAN_POINT"
    //   group 3 = raw parameter string,  ex: "'',( 1.0, 2.0, 3.0)"
    // ex full match: "#12 = CARTESIAN_POINT('', (1.0, 2.0, 3.0));"
    static std::regex entityRegex(R"(#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.+)\)\s*;$)");
    while (std::getline(file, line)) {
        std::string trimmedLine = trimWS(line);
        // all we care about is between "DATA;" and "ENDSEC;", skip if not (metadata)
        if (trimmedLine == "DATA;") {
            inData = true;
            continue;
        }
        if (trimmedLine == "ENDSEC;") {
            inData = false;
            continue;
        }
        if (!inData)
            continue;
        // strip /* ... */ block comments
        while (true) {
            auto commentStart = trimmedLine.find("/*"), commentEnd = trimmedLine.find("*/");
            if (commentStart == std::string::npos)
                break;
            trimmedLine.erase(commentStart, (commentEnd == std::string::npos ? trimmedLine.size() : commentEnd + 2) - commentStart);
        }
        // STEP entities can span multiple lines so accumulate until the terminating ';'
        accum += trimmedLine;
        if (!accum.empty() && accum.back() == ';') {
            std::smatch match;
            if (std::regex_search(accum, match, entityRegex))
                entityMap[std::stoi(match[1])] = { match[2], match[3] };
            accum.clear();
        }
    }
    return entityMap;
}

#pragma region step resolver
// STEP stores everything as a flat list of numbered entities that reference each other by ID, "resolving" means following those references to assemble the
// actual geometric data, for example a CYLINDRICAL_SURFACE entity just contains a reference to an AXIS2_PLACEMENT_3D and a radius number, resolving it means
// looking up that placement entity, which in turn references two DIRECTION entities and a CARTESIAN_POINT, each of which you also have to look up,
// by the time you are done you have a proper origin, axis direction, and radius you can actually do math with
Vec3 resolvePoint(int id, const StepMap& map)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return {};
    auto params = splitTopLevel(entityIt->second.params);
    // CARTESIAN_POINT params: (name, (x,y,z))
    // params[0] = name string (ex: "''"), params[1] = coordinate tuple "(1.0,2.0,3.0)"
    if (params.size() < 2)
        return {};
    // unwrap "(1.0,2.0,3.0)" -> "1.0,2.0,3.0", then split to ["1.0","2.0","3.0"]
    auto coords = splitTopLevel(unwrap(trimWS(params[1])));
    if (coords.size() < 3)
        return {};
    return { dbl(coords[0]), dbl(coords[1]), dbl(coords[2]) };
}

AxisPlacement resolveAxis(int id, const StepMap& map)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return {};
    // AXIS2_PLACEMENT_3D params: (name, origin_ref, z_dir_ref, x_dir_ref)
    // z and x direction refs point to DIRECTION entities (which share the CARTESIAN_POINT layout)
    // ex: "('', #10, #11, #12)" -> origin=point[10], zDir=point[11], xDir=point[12]
    auto params = splitTopLevel(entityIt->second.params);
    AxisPlacement axis;
    if (params.size() >= 2)
        axis.origin = resolvePoint(stepRef(params[1]), map);
    if (params.size() >= 3)
        axis.zDir = resolvePoint(stepRef(params[2]), map);
    if (params.size() >= 4)
        axis.xDir = resolvePoint(stepRef(params[3]), map);
    return axis;
}

Surface resolveSurface(int id, const StepMap& map)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return {};
    Surface surface;
    auto params = splitTopLevel(entityIt->second.params);
    if (entityIt->second.type == "PLANE") {
        surface.kind = SurfaceKind::Plane;
        // PLANE params: (name, axis_placement_ref)
        // ex: "('flat_top', #20)" -> axis from entity #20
        if (params.size() >= 2)
            surface.axis = resolveAxis(stepRef(params[1]), map);
    } else if (entityIt->second.type == "CYLINDRICAL_SURFACE") {
        surface.kind = SurfaceKind::Cylinder;
        // CYLINDRICAL_SURFACE params: (name, axis_placement_ref, radius)
        // ex: "('', #30, 5.0)" -> axis from #30, radius = 5.0
        if (params.size() >= 2)
            surface.axis = resolveAxis(stepRef(params[1]), map);
        if (params.size() >= 3)
            surface.majorRadius = dbl(params[2]);
    } else if (entityIt->second.type == "TOROIDAL_SURFACE") {
        surface.kind = SurfaceKind::Torus;
        // TOROIDAL_SURFACE params: (name, axis_placement_ref, major_radius, minor_radius)
        // ex: "('', #40, 10.0, 2.0)" -> fillet ring: center-circle R=10, tube r=2
        if (params.size() >= 2)
            surface.axis = resolveAxis(stepRef(params[1]), map);
        if (params.size() >= 3)
            surface.majorRadius = dbl(params[2]);
        if (params.size() >= 4)
            surface.minorRadius = dbl(params[3]);
    }
    return surface;
}

#pragma region curve
// sampler: a CIRCLE arc from startPt to endPt (or full revolution if isClosedLoop)
// sameSense: effective arc direction = EDGE_CURVE same_sense XOR ORIENTED_EDGE edgeReversed
// true = CCW sweep (angleEnd > angleStart), false = CW sweep (angleEnd < angleStart, short arc the other way)
// reversing an edge traversal also reverses which arc direction is the correct short arc, hence the XOR
// ex: same_sense=.T., edgeReversed=true -> effective false -> CW sweep takes the 75 deg arc, not the 285 deg one
static std::vector<Vec3> sampleCircle(int id, Vec3 startPt, Vec3 endPt, bool isClosedLoop, const StepMap& map, int segs, bool sameSense = true)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return { startPt, endPt };
    // CIRCLE params: (name, axis_placement_ref, radius)
    // ex: "('', #50, 3.0)" -> circle centered at axis[50] origin, radius=3
    auto params = splitTopLevel(entityIt->second.params);
    if (params.size() < 3)
        return { startPt, endPt };
    AxisPlacement placement = resolveAxis(stepRef(params[1]), map);
    double radius = dbl(params[2]);
    Vec3 O = placement.origin, Z = placement.zDir.norm(), X = placement.xDir.norm(), Y = Z.cross(X).norm();

    // returns the angle of 'pt' in the circle's local XY plane (atan2 in [-pi, pi])
    // ex: pt on the +X side -> 0.0,  pt on the +Y side -> pi/2
    auto angleOf = [&](Vec3 pt) {
        Vec3 delta = pt - O;
        return std::atan2(delta.dot(Y), delta.dot(X));
    };

    double angleStart, angleEnd;
    if (isClosedLoop) {
        // full revolution: 0 -> 2pi, sample 'segs' segments
        angleStart = 0;
        angleEnd = 2 * M_PI;
    } else if (sameSense) {
        // CCW sweep: enforce angleEnd > angleStart
        // ex: start=2.9rad, end=-2.9rad -> angleEnd becomes -2.9+2pi=3.38
        angleStart = angleOf(startPt);
        angleEnd = angleOf(endPt);
        while (angleEnd <= angleStart)
            angleEnd += 2 * M_PI;
        // clamp to at most one full revolution (guard against floating-point overshoot)
        if (angleEnd - angleStart > 2 * M_PI + 1e-6)
            angleEnd = angleStart + 2 * M_PI;
    } else {
        // CW sweep: enforce angleEnd < angleStart (short arc in decreasing-angle direction)
        // ex: start=3.14rad, end=-1.83rad -> CW sweep=-1.31 (75 deg correct short arc)
        //     without this: CCW enforcement would give sweep=4.97 (285 deg, the wrong long arc)
        angleStart = angleOf(startPt);
        angleEnd = angleOf(endPt);
        while (angleEnd >= angleStart)
            angleEnd -= 2 * M_PI;
        if (angleStart - angleEnd > 2 * M_PI + 1e-6)
            angleEnd = angleStart - 2 * M_PI;
    }
    // scale segment count proportionally to the arc fraction of a full circle
    // ex: segs=48, quarter arc (pi/2 out of 2pi) -> numSegments = max(2, 48*0.25) = 12 segments
    // always emit at least 2 points (start + end) for degenerate near-zero arcs
    int numSegments = isClosedLoop ? segs : std::max(2, (int)(segs * std::abs(angleEnd - angleStart) / (2 * M_PI)));
    std::vector<Vec3> points;
    points.reserve(numSegments + 1);
    for (int i = 0; i <= numSegments; i++) {
        double angle = angleStart + (angleEnd - angleStart) * (double)i / numSegments;
        // point on circle: O + radius*(cos(angle)*X + sin(angle)*Y)
        points.push_back(O + X * std::cos(angle) * radius + Y * std::sin(angle) * radius);
    }
    return points;
}

// evaluate a B_SPLINE_CURVE_WITH_KNOTS using De Boor's algorithm
// each round takes the current set of local control points and replaces them with a smaller set of blended points, until one point remains
// basically what De Casteljau's algorithm is to Bézier curves, numerically stable and the standard way to evaluate splines
static std::vector<Vec3> sampleBSpline(int id, Vec3 startPt, Vec3 endPt, const StepMap& map, int segs = 16)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return { startPt, endPt };
    // B_SPLINE_CURVE_WITH_KNOTS params:
    //   (name, degree, (ctrl_pts...), curve_form, closed_curve, self_intersect, (knot_multiplicities...), (knots...), knot_spec)
    // ex degree-3 curve: "('', 3, (#60,#61,#62,#63), .UNSPECIFIED., .F., .F., (4,4), (0.,1.), .UNSPECIFIED.)"
    // degree = how smoothly the track bends, 1 = straight line segments between posts, 2 = gentle bends,
    //      3 = standard for CAD, smooth enough that you cannot feel the joints at all
    // ctrl_pts = the posts you place in space that the track is attracted to but does not necessarily touch, move a post and the track bends toward it
    //      4 control points = minimum for a degree-3 curve
    //  curve_form means the overall shape family: .UNSPECIFIED., .CIRCULAR_ARC. or .ELLIPTIC_ARC. (informational only)
    // closed_curve .F. means the track has a start and an end, .T. means it loops back on itself like a circuit (informational only)
    // self_intersect .F. means the track never crosses itself, .T. means it does  (informational only)
    // knot_multiplicities= how many times each knot value is repeated, with (4,4) means the first andl last knots aree repeated 4 times
    //      repeating a knot degree times at each end is what forces the curve to actually start and end at the first and last control points respectively
    //      rather than just being attracted to them, this is called a clamped B-spline
    // knots = the actual knot values, (0., 1.) combined with the multiplicities this expands to (0, 0, 0, 0, 1, 1, 1, 1) which is our
    //      8 values for 4 control points at degree 3, satisfying the n + degree + 1 rule
    // knot_spec = knot distribution type (uniform, quasi-uniform...), .UNSPECIFIED. is almost always what you see in exported STEP files
    auto params = splitTopLevel(entityIt->second.params);
    if (params.size() < 3)
        return { startPt, endPt };
    int degree = (int)dbl(params[1]);
    std::vector<Vec3> controlPoints;
    for (auto& controlRef : splitTopLevel(unwrap(trimWS(params[2])))) {
        int refId = stepRef(trimWS(controlRef));
        if (refId > 0)
            controlPoints.push_back(resolvePoint(refId, map));
    }
    // need at least degree+1 control points to evaluate a valid curve
    // ex: degree 3 needs at least 4 control points
    if ((int)controlPoints.size() <= degree)
        return { startPt, endPt };
    // scan remaining params for the knot vector: first parenthesised list of plain floats with enough entries, must have n+degree+1 knots for n control points
    // go 3 by 3 for coords, tokenize commas, check for no ref but actual convertible doubles (allNumeric false means something wrong)
    // ex: 4 ctrl pts, degree 3 -> need 4+3+1=8 knots, ex: (0,0,0,0,1,1,1,1)
    std::vector<double> knots;
    for (int i = 3; i < (int)params.size() && knots.empty(); i++) {
        std::string token = trimWS(params[i]);
        if (token.empty() || token[0] != '(')
            continue;
        auto values = splitTopLevel(unwrap(token));
        std::vector<double> candidates;
        bool allNumeric = true;
        for (auto& value : values) {
            std::string trimmedValue = trimWS(value);
            if (!trimmedValue.empty() && trimmedValue[0] == '#') {
                allNumeric = false;
                break;
            }
            try {
                candidates.push_back(std::stod(trimmedValue));
            } catch (...) {
                allNumeric = false;
                break;
            }
        }
        if (allNumeric && (int)candidates.size() >= (int)controlPoints.size() + degree + 1)
            knots = candidates;
    }
    if (knots.empty())
        return { startPt, endPt };
    // valid parameter domain: [knots[degree], knots[n-1-degree]] (clamped B-spline convention)
    // ex: knots=(0,0,0,0,1,1,1,1), degree=3 -> tMin=knots[3]=0, tMax=knots[4]=1
    double tMin = knots[degree], tMax = knots[(int)knots.size() - 1 - degree];

    // De Boor's algorithm: evaluate the B-spline at parameter t, finds the knot span k such that knots[k] <= t < knots[k+1]
    // then runs degree rounds of linear interpolation on the local control polygon
    // ex: degree=3, t=0.5 on [0 to 1] -> picks the span containing 0.5, blends 4 local ctrl pts down to 1
    auto deBoor = [&](double t) -> Vec3 {
        // find knot span index (clamped to the last valid span)
        int spanIndex = degree, lastSpan = (int)knots.size() - 2 - degree;
        for (int i = degree; i < (int)knots.size() - 1 - degree; i++) {
            if (t < knots[i + 1]) {
                spanIndex = i;
                break;
            }
            spanIndex = lastSpan;
        }
        // local control points for this span: controlPoints[spanIndex-degree to spanIndex]
        std::vector<Vec3> localPoints(controlPoints.begin() + spanIndex - degree, controlPoints.begin() + spanIndex + 1);
        // degree rounds of De Boor triangular interpolation:
        // each round replaces degree+1-r points with r-blended values
        // ex: degree=3, r=1: blend indices 3,2,1; r=2: blend 3,2; r=3: blend 3 -> final point
        for (int r = 1; r <= degree; r++)
            for (int j = degree; j >= r; j--) {
                double knotSpanWidth = knots[spanIndex - degree + j + r] - knots[spanIndex - degree + j];
                // blend factor alpha, says how far along a knot interval the evaluation point is, used to linearly blend between two control points
                // guard against zero-length knot spans (repeated knots at boundary) which would cause division by 0
                // ex: clamped endpoint where knots[k]=knots[k+1] -> knotSpanWidth=0, use alpha=0 to pin to left control point
                double alpha = (knotSpanWidth < 1e-12) ? 0.0 : (t - knots[spanIndex - degree + j]) / knotSpanWidth;
                localPoints[j] = localPoints[j - 1] * (1 - alpha) + localPoints[j] * alpha;
            }
        return localPoints[degree];
    };
    std::vector<Vec3> points;
    points.reserve(segs + 1);
    for (int i = 0; i <= segs; i++)
        points.push_back(deBoor(tMin + (tMax - tMin) * (double)i / segs));
    return points;
}

#pragma region boundary loop
// solver: FACE_OUTER_BOUND/FACE_BOUND -> EDGE_LOOP -> ORIENTED_EDGE -> EDGE_CURVE -> curve
// FACE_OUTER_BOUND / FACE_BOUND = the declaration that a boundary exists and whether it is the outer rim or a hole,
// it's just a wrapper that says "here comes a loop and it is of this kind", no geometry yet
// EDGE_LOOP = the ordered list of edges that form the closed outline, "go around these edges in this order and you will have
// traced the full boundary", still no geometry, just references
// ORIENTED_EDGE = one edge, plus a flag saying which direction to traverse it, the same underlying edge can be shared by two adjacent faces (share a boundary)
// but each face traverses it in the opposite direction, .F. / .T. flag handles that, still no geometry, just topology
// EDGE_CURVE = the actual geometric edge: which two vertices it connects and which curve lies between them, real 3D positions and curve entity
// curve = geometry itself, LINE or CIRCLE or B_SPLINE_CURVE_WITH_KNOTS, this is what gets sampled into actual 3D points
static BoundaryLoop sampleLoop(int boundId, const StepMap& map, int arcSegs)
{
    BoundaryLoop loop;
    auto boundIt = map.find(boundId);
    if (boundIt == map.end())
        return loop;
    loop.isOuter = (boundIt->second.type == "FACE_OUTER_BOUND");
    // FACE_OUTER_BOUND / FACE_BOUND params: (name, edge_loop_ref, orientation)
    // ex: "('', #100, .T.)" -> edge loop is entity #100
    auto boundParams = splitTopLevel(boundIt->second.params);
    if (boundParams.size() < 2)
        return loop;
    auto loopIt = map.find(stepRef(boundParams[1]));
    if (loopIt == map.end() || loopIt->second.type != "EDGE_LOOP")
        return loop;
    // EDGE_LOOP params: (name, (oriented_edge_ref, ...))
    // ex: "('', (#110,#111,#112,#113))" -> 4 oriented edges forming a closed loop
    auto loopParams = splitTopLevel(loopIt->second.params);
    if (loopParams.size() < 2)
        return loop;

    for (auto& edgeRef : splitTopLevel(unwrap(trimWS(loopParams[1])))) {
        int orientedEdgeId = stepRef(trimWS(edgeRef));
        auto orientedEdgeIt = map.find(orientedEdgeId);
        if (orientedEdgeIt == map.end() || orientedEdgeIt->second.type != "ORIENTED_EDGE")
            continue;
        // ORIENTED_EDGE params: (name, *, *, edge_curve_ref, orientation_flag)
        // orientation_flag ".F." means the edge is traversed in reverse (end->start)
        // ex: "('', *, *, #120, .F.)" -> traverse edge #120 backwards
        auto orientedEdgeParams = splitTopLevel(orientedEdgeIt->second.params);
        if (orientedEdgeParams.size() < 5)
            continue;
        bool edgeReversed = (trimWS(orientedEdgeParams[4]) == ".F.");
        auto edgeCurveIt = map.find(stepRef(orientedEdgeParams[3]));
        if (edgeCurveIt == map.end() || edgeCurveIt->second.type != "EDGE_CURVE")
            continue;
        // EDGE_CURVE params: (name, start_vertex_ref, end_vertex_ref, curve_ref, same_sense)
        // ex: "('', #130, #131, #132, .T.)" -> goes from vtx #130 to vtx #131 along curve #132
        // effective sameSense = rawSameSense XOR edgeReversed: reversing the edge traversal also
        // reverses the arc direction, flipping which arc (short vs long) is the geometrically correct one
        auto edgeCurveParams = splitTopLevel(edgeCurveIt->second.params);
        if (edgeCurveParams.size() < 4)
            continue;
        bool rawSameSense = !(edgeCurveParams.size() >= 5 && trimWS(edgeCurveParams[4]) == ".F.");
        bool sameSense = rawSameSense != edgeReversed; // XOR

        int startVertexId = stepRef(edgeCurveParams[1]), endVertexId = stepRef(edgeCurveParams[2]);
        // if start and end vertex entity are the same ref, this edge is a closed circle (full revolution)
        // ex: "#130 = VERTEX_POINT" for both startVertexId and endVertexId -> full-circle edge (ex: bolt head rim)
        bool fullCircle = (startVertexId == endVertexId && startVertexId > 0);
        if (fullCircle)
            loop.hasFullCircle = true;

        // look up a VERTEX_POINT to get its 3D position
        auto resolveVertex = [&](int vertexId) -> Vec3 {
            auto vertexIt = map.find(vertexId);
            if (vertexIt == map.end() || vertexIt->second.type != "VERTEX_POINT")
                return {};
            // VERTEX_POINT params: (name, cartesian_point_ref), ex: "('', #140)" -> position = point[140]
            auto vertexParams = splitTopLevel(vertexIt->second.params);
            return (vertexParams.size() >= 2) ? resolvePoint(stepRef(vertexParams[1]), map) : Vec3 {};
        };
        Vec3 vertexStart = resolveVertex(startVertexId), vertexEnd = resolveVertex(endVertexId);
        // swap start/end to match the traversal direction of this oriented edge
        if (edgeReversed)
            std::swap(vertexStart, vertexEnd);

        int curveId = stepRef(edgeCurveParams[3]);
        std::vector<Vec3> edgeSamples;
        auto curveIt = map.find(curveId);
        if (curveIt != map.end()) {
            if (curveIt->second.type == "CIRCLE")
                edgeSamples = sampleCircle(curveId, vertexStart, vertexEnd, fullCircle, map, arcSegs, sameSense);
            else if (curveIt->second.type == "LINE")
                edgeSamples = { vertexStart, vertexEnd };
            else if (curveIt->second.type == "B_SPLINE_CURVE_WITH_KNOTS")
                edgeSamples = sampleBSpline(curveId, vertexStart, vertexEnd, map, 16);
            else
                edgeSamples = { vertexStart, vertexEnd };
        } else
            edgeSamples = { vertexStart, vertexEnd };

        // append edge samples, skipping the junction duplicate with the previous edge
        // ex: prev edge ended at P, this edge starts at P -> skip index 0 to avoid [P,P] in the list
        int startIndex = loop.points.empty() ? 0 : 1;
        for (int i = startIndex; i < (int)edgeSamples.size(); i++)
            loop.points.push_back(edgeSamples[i]);
    }
    // remove closing duplicate: STEP loops are closed, so the last point often coincides with the first, drop it so the polygon is a proper open ring
    // ex: [A,B,C,D,A] -> [A,B,C,D]
    if (loop.points.size() > 1 && loop.points.front().near(loop.points.back()))
        loop.points.pop_back();
    return loop;
}

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
static TessellatedFace tessGrid(SurfaceKind kind, std::function<Vec3(double u, double v)> positionFn, std::function<Vec3(double u, double v)> normalFn,
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

#pragma region plane tesselation
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
static TessellatedFace tessellateAdvancedFace(int faceId, const StepMap& map, int arcSegs, Surface* outSurface = nullptr)
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

// sums the area of all front-face triangles (first half of the index list, back-face duplicates are the second half)
// exact for planar faces (ear-clip triangulates the exact boundary), approximate for curved surfaces (depends on arcSegs)
static float computeFaceArea(const TessellatedFace& face)
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
static Mesh uploadMesh(const TessellatedFace& tessellatedFace)
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

CadModel::~CadModel()
{
    for (auto& mesh : meshes)
        UnloadMesh(mesh);
}

// arcSegs: arc subdivision count (circles, cylinders, tori), essentially LOD
// 48 segments is an arbitrary feel-tuned value for smooth circles, I'll also use half of it for smaller torus tubes later
CadModel loadStep(const std::string& path, int arcSegs = 48)
{
    CadModel model;
    // initialize bbox inverted so the first real vertex always wins both min and max comparisons
    // ex: first vertex at (3,1,2) -> min becomes (3,1,2), max becomes (3,1,2)
    model.bbox = { { 1e18f, 1e18f, 1e18f }, { -1e18f, -1e18f, -1e18f } };
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
        // expand the overall bounding box with all vertices of this face
        // vertices are stored flat as [x0,y0,z0, x1,y1,z1, ...] so step 3 by 3
        for (int i = 0; i + 2 < (int)tessellatedFace.vertices.size(); i += 3) {
            model.bbox.min.x = std::min(model.bbox.min.x, tessellatedFace.vertices[i]);
            model.bbox.min.y = std::min(model.bbox.min.y, tessellatedFace.vertices[i + 1]);
            model.bbox.min.z = std::min(model.bbox.min.z, tessellatedFace.vertices[i + 2]);
            model.bbox.max.x = std::max(model.bbox.max.x, tessellatedFace.vertices[i]);
            model.bbox.max.y = std::max(model.bbox.max.y, tessellatedFace.vertices[i + 1]);
            model.bbox.max.z = std::max(model.bbox.max.z, tessellatedFace.vertices[i + 2]);
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
            chr.heightMin = 1e18;
            chr.heightMax = -1e18;
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
        model.totalTriangleCount += (int)tessellatedFace.indices.size() / 6; // front triangles only
    }
    return model;
}

// returns the bbox center in original STEP space, used to undo the centering applied in drawCadModel
static Vector3 modelCenter(const CadModel& model)
{
    return { (model.bbox.min.x + model.bbox.max.x) * 0.5f, (model.bbox.min.y + model.bbox.max.y) * 0.5f, (model.bbox.min.z + model.bbox.max.z) * 0.5f };
}

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

    float closestT = 1e18f;
    int hitFace = -1;
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

// returns the average position of all front-face vertices in draw space (with centering already applied)
// used to anchor axis/normal arrows at the visual center of the face
static Vector3 computeFaceCentroid(const TessellatedFace& face, Vector3 center, Vector3 offset = { 0.0f, 0.0f, 0.0f })
{
    // front vertices occupy the first half of the flat array (second half is the back-face duplicates)
    int frontVertexCount = (int)face.vertices.size() / 6;
    Vector3 sum = { 0, 0, 0 };
    for (int i = 0; i < frontVertexCount; i++) {
        int base = i * 3;
        sum.x += face.vertices[base] - center.x + offset.x;
        sum.y += face.vertices[base + 1] - center.y + offset.y;
        sum.z += face.vertices[base + 2] - center.z + offset.z;
    }
    if (frontVertexCount > 0) {
        float inv = 1.0f / (float)frontVertexCount;
        sum.x *= inv;
        sum.y *= inv;
        sum.z *= inv;
    }
    return sum;
}

// computes the axis-aligned bounding box of the face in draw space (with centering applied)
static BoundingBox computeFaceBBox(const TessellatedFace& face, Vector3 center, Vector3 offset = { 0.0f, 0.0f, 0.0f })
{
    BoundingBox bbox = { { 1e18f, 1e18f, 1e18f }, { -1e18f, -1e18f, -1e18f } };
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
static float computeFaceMinDistance(
    const TessellatedFace& faceA, const TessellatedFace& faceB, Vector3 offsetA = { 0.0f, 0.0f, 0.0f }, Vector3 offsetB = { 0.0f, 0.0f, 0.0f })
{
    float minDistSq = 1e18f;
    int countA = (int)faceA.vertices.size() / 6; // front vertices only
    int countB = (int)faceB.vertices.size() / 6;
    for (int i = 0; i < countA; i++) {
        int baseA = i * 3;
        for (int j = 0; j < countB; j++) {
            int baseB = j * 3;
            float dx = (faceA.vertices[baseA] + offsetA.x) - (faceB.vertices[baseB] + offsetB.x);
            float dy = (faceA.vertices[baseA + 1] + offsetA.y) - (faceB.vertices[baseB + 1] + offsetB.y);
            float dz = (faceA.vertices[baseA + 2] + offsetA.z) - (faceB.vertices[baseB + 2] + offsetB.z);
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
    BoundingBox dynamicBbox = { { 1e18f, 1e18f, 1e18f }, { -1e18f, -1e18f, -1e18f } };
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
    Vector3 off = model.faceOffsets[model.selectedFace];
    const Surface& surface = model.faceSurfaces[model.selectedFace];

    switch (surface.kind) {
    case SurfaceKind::Plane: {
        // one big normal arrow from the face centroid along zDir
        Vector3 faceCentroid = computeFaceCentroid(model.pickData[model.selectedFace], center, off);
        Vec3 normal = surface.axis.zDir.norm();
        Vector3 tip = { faceCentroid.x + (float)normal.x * scale, faceCentroid.y + (float)normal.y * scale, faceCentroid.z + (float)normal.z * scale };
        DrawLine3D(faceCentroid, tip, SKYBLUE);
        break;
    }
    case SurfaceKind::Cylinder:
    case SurfaceKind::Torus: {
        // axis line extending in both directions from the surface origin (which is a point on the axis)
        // surface origin is in STEP space, apply centering then offset to get draw space
        Vector3 axisOrigin = { (float)surface.axis.origin.x - center.x + off.x, (float)surface.axis.origin.y - center.y + off.y,
            (float)surface.axis.origin.z - center.z + off.z };
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
    BoundingBox bbox = { { 1e18f, 1e18f, 1e18f }, { -1e18f, -1e18f, -1e18f } };
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

#pragma region geometry healing
// retessellates the GPU mesh of a cylinder face with updated [heightMin, heightMax] and re-uploads it
// the CPU pickData is also replaced so ray picking and stats remain accurate
static void retessCylinderFace(CadModel& model, int cylIdx, double newHeightMin, double newHeightMax)
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
static std::vector<CylinderHealEntry> buildCylinderHealCache(const CadModel& model, int planeFaceIdx)
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
static void applyCylinderHealCache(CadModel& model, std::vector<CylinderHealEntry>& cache, Vec3 planeNormal, Vector3 delta)
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

#pragma region main
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
    static bool wasTranslating = false;
    static int cachedHealFace = -1; // selectedFace at the time the heal cache was built
    static std::vector<CylinderHealEntry> healCache; // built once per gesture, reused every frame until key release
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
        if (translating && !wasTranslating) {
            // always rebuild at every gesture rising edge so connectivity is re-evaluated at the plane's current
            // position, a stale cache from a prior gesture must never carry over because the plane may have
            // been moved off-axis (disconnected from the cylinder) between gestures
            healCache = buildCylinderHealCache(model, model.selectedFace);
            cachedHealFace = model.selectedFace;
            // snapshot the pre-gesture cylinder height ranges for every cylinder this gesture will heal so
            // undo/redo can retessellate them back to the correct extent, not just the plane offset
            UndoEntry entry;
            entry.faceIndex = model.selectedFace;
            entry.offsetBefore = off;
            for (const auto& healEntry : healCache)
                entry.cylSnapshots.push_back({ healEntry.cylFaceIdx, model.cylHeightRanges[healEntry.cylFaceIdx] });
            model.undoStack.push_back(std::move(entry));
            model.redoStack.clear();
        }
        // invalidate cache if the selected face changed between gestures
        if (cachedHealFace != model.selectedFace) {
            healCache.clear();
            cachedHealFace = -1;
        }
        wasTranslating = translating;
        Vec3 planeNormal = surf.axis.zDir.norm();
        if (IsKeyDown(KEY_UP)) {
            Vector3 delta = { (float)zDir.x * step, (float)zDir.y * step, (float)zDir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, healCache, planeNormal, delta);
        }
        if (IsKeyDown(KEY_DOWN)) {
            Vector3 delta = { -(float)zDir.x * step, -(float)zDir.y * step, -(float)zDir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, healCache, planeNormal, delta);
        }
        if (IsKeyDown(KEY_RIGHT)) {
            Vector3 delta = { (float)xDir.x * step, (float)xDir.y * step, (float)xDir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, healCache, planeNormal, delta);
        }
        if (IsKeyDown(KEY_LEFT)) {
            Vector3 delta = { -(float)xDir.x * step, -(float)xDir.y * step, -(float)xDir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, healCache, planeNormal, delta);
        }
        if (IsKeyDown(KEY_PAGE_UP)) {
            Vector3 delta = { (float)yDir.x * step, (float)yDir.y * step, (float)yDir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, healCache, planeNormal, delta);
        }
        if (IsKeyDown(KEY_PAGE_DOWN)) {
            Vector3 delta = { -(float)yDir.x * step, -(float)yDir.y * step, -(float)yDir.z * step };
            off.x += delta.x;
            off.y += delta.y;
            off.z += delta.z;
            applyCylinderHealCache(model, healCache, planeNormal, delta);
        }
    } else {
        wasTranslating = false;
        healCache.clear();
        cachedHealFace = -1;
    }

    // (ctrl+z is swallowed by the Win32 message loop as ASCII SUB before Raylib sees the key event XDD)
    // both blocked while any translation key is held to prevent stomping an in-progress gesture
    if (!translating && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_U)) {
        if (!model.undoStack.empty()) {
            UndoEntry entry = model.undoStack.back();
            model.undoStack.pop_back();
            // capture current cylinder height ranges before overwriting them so redo can restore forward state
            UndoEntry redoEntry;
            redoEntry.faceIndex = entry.faceIndex;
            redoEntry.offsetBefore = model.faceOffsets[entry.faceIndex];
            for (const auto& snap : entry.cylSnapshots)
                redoEntry.cylSnapshots.push_back({ snap.cylFaceIdx, model.cylHeightRanges[snap.cylFaceIdx] });
            model.redoStack.push_back(std::move(redoEntry));
            model.faceOffsets[entry.faceIndex] = entry.offsetBefore;
            for (const auto& snap : entry.cylSnapshots) {
                model.cylHeightRanges[snap.cylFaceIdx] = snap.rangeBefore;
                retessCylinderFace(model, snap.cylFaceIdx, snap.rangeBefore.heightMin, snap.rangeBefore.heightMax);
            }
            model.selectedFace = entry.faceIndex;
        }
    }
    if (!translating && (IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL)) && IsKeyPressed(KEY_Y)) {
        if (!model.redoStack.empty()) {
            UndoEntry entry = model.redoStack.back();
            model.redoStack.pop_back();
            // capture current cylinder height ranges before overwriting them so undo can restore backward state
            UndoEntry undoEntry;
            undoEntry.faceIndex = entry.faceIndex;
            undoEntry.offsetBefore = model.faceOffsets[entry.faceIndex];
            for (const auto& snap : entry.cylSnapshots)
                undoEntry.cylSnapshots.push_back({ snap.cylFaceIdx, model.cylHeightRanges[snap.cylFaceIdx] });
            model.undoStack.push_back(std::move(undoEntry));
            model.faceOffsets[entry.faceIndex] = entry.offsetBefore;
            for (const auto& snap : entry.cylSnapshots) {
                model.cylHeightRanges[snap.cylFaceIdx] = snap.rangeBefore;
                retessCylinderFace(model, snap.cylFaceIdx, snap.rangeBefore.heightMin, snap.rangeBefore.heightMax);
            }
            model.selectedFace = entry.faceIndex;
        }
    }

    // convert spherical (yaw, pitch, orbitRadius) to Cartesian camera position (x y z)
    // ex: yaw=90deg, pitch=0 -> camera sits on the +X axis looking toward origin
    float yawRadians = DEG2RAD * yaw, pitchRadians = DEG2RAD * pitch;
    camera.position = { orbitRadius * std::cos(pitchRadians) * std::sin(yawRadians), orbitRadius * std::sin(pitchRadians),
        orbitRadius * std::cos(pitchRadians) * std::cos(yawRadians) };
}

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

    DrawText(TextFormat("Faces: %d | Triangles: %d", (int)model.meshes.size(), model.totalTriangleCount), 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;

    // model bbox dimensions and center recomputed each frame to reflect per-face offsets
    BoundingBox liveBbox = computeLiveModelBBox(model);
    Vector3 mCenter = modelCenter(model);
    DrawText(TextFormat("Width: %.2f | Height: %.2f | Depth: %.2f mm", liveBbox.max.x - liveBbox.min.x, liveBbox.max.y - liveBbox.min.y,
                 liveBbox.max.z - liveBbox.min.z),
        20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText(TextFormat("Position: %.2f %.2f %.2f", (liveBbox.min.x + liveBbox.max.x) * 0.5f - mCenter.x, (liveBbox.min.y + liveBbox.max.y) * 0.5f - mCenter.y,
                 (liveBbox.min.z + liveBbox.max.z) * 0.5f - mCenter.z),
        20, uiY, 16, LIGHTGRAY);
    uiY += uiStep + 4;

    DrawText("Drag = orbit | Scroll = zoom", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("RClick = select | Shift+RClick = get distance", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("N = normals | B = bounding box", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;
    DrawText("Arrows/page keys = translate face | Ctrl+U/Y = undo/redo", 20, uiY, 16, LIGHTGRAY);
    uiY += uiStep;

    if (model.selectedFace < 0)
        return;

    uiY += 6;
    int faceIdx = model.selectedFace;
    int frontTris = (int)model.pickData[faceIdx].indices.size() / 6;
    DrawText(TextFormat("Face #%d [%s] | %d triangles | %.2f mm^2", faceIdx, nameForKind(model.pickData[faceIdx].kind), frontTris, model.faceAreas[faceIdx]),
        20, uiY, 16, YELLOW);
    uiY += uiStep;

    // face bbox dimensions and center (in draw space = relative to model center = world coords when model at origin)
    // offset applied so stats match the visual position of the face after push/pull
    Vector3 center = modelCenter(model);
    BoundingBox faceBbox = computeFaceBBox(model.pickData[faceIdx], center, model.faceOffsets[faceIdx]);
    DrawText(TextFormat("Width: %.2f | Height: %.2f | Depth: %.2f mm", faceBbox.max.x - faceBbox.min.x, faceBbox.max.y - faceBbox.min.y,
                 faceBbox.max.z - faceBbox.min.z),
        20, uiY, 16, YELLOW);
    uiY += uiStep;
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

int main()
{
    InitWindow(1280, 720, "CAD");
    SetTargetFPS(60);

    // executable either at root or in build/Release but assets is always at root
    CadModel model = loadStep(std::filesystem::exists("assets") ? "assets/screw.step" : "../../assets/screw.step");

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