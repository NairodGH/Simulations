#include "cad.hpp"

#pragma region step parser
// accumulates multi-line entities until ';', extracts #ID = TYPE(params)

// splits a comma-separated parameter string at the top level only, commas inside nested parentheses are ignored, ex: "A,(B,C),D" -> ["A", "(B,C)", "D"]
// needed because STEP params can contain nested lists like coordinate tuples
std::vector<std::string> splitTopLevel(const std::string& input)
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
std::string trimWS(const std::string& input)
{
    auto first = input.find_first_not_of(" \t\r\n"), last = input.find_last_not_of(" \t\r\n");
    return first == std::string::npos ? "" : input.substr(first, last - first + 1);
}

// strips the outermost parentheses from a string, used to peel the argument list off a STEP entity before splitting it, ex: "(A,B)" -> "A,B"
std::string unwrap(const std::string& input)
{
    auto open = input.find('('), close = input.rfind(')');
    return (open == std::string::npos || close == std::string::npos) ? input : input.substr(open + 1, close - open - 1);
}

// parses a STEP entity reference to its numeric ID, returns -1 if not a ref, ex: " #123 " -> 123, "0.5" -> -1
int stepRef(const std::string& input)
{
    auto trimmed = trimWS(input);
    return (!trimmed.empty() && trimmed[0] == '#') ? std::stoi(trimmed.substr(1)) : -1;
}

// converts a trimmed string token to a double, returns 0 on parse failure, ex: " 3.14 " -> 3.14,   ".F." -> 0.0
double dbl(const std::string& input)
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

#pragma region sample curves
// a CIRCLE arc from startPt to endPt (or full revolution if isClosedLoop)
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

#pragma region sampleLoop
// boundary loop solver: FACE_OUTER_BOUND/FACE_BOUND -> EDGE_LOOP -> ORIENTED_EDGE -> EDGE_CURVE -> curve
// FACE_OUTER_BOUND / FACE_BOUND = the declaration that a boundary exists and whether it is the outer rim or a hole,
// it's just a wrapper that says "here comes a loop and it is of this kind", no geometry yet
// EDGE_LOOP = the ordered list of edges that form the closed outline, "go around these edges in this order and you will have
// traced the full boundary", still no geometry, just references
// ORIENTED_EDGE = one edge, plus a flag saying which direction to traverse it, the same underlying edge can be shared by two adjacent faces (share a boundary)
// but each face traverses it in the opposite direction, .F. / .T. flag handles that, still no geometry, just topology
// EDGE_CURVE = the actual geometric edge: which two vertices it connects and which curve lies between them, real 3D positions and curve entity
// curve = geometry itself, LINE or CIRCLE or B_SPLINE_CURVE_WITH_KNOTS, this is what gets sampled into actual 3D points
BoundaryLoop sampleLoop(int boundId, const StepMap& map, int arcSegs)
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
