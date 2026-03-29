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
    // reserve up front, current can grow up to input.size() in the worst case (no commas)
    current.reserve(input.size());
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
    } catch (const std::invalid_argument&) {
        return 0.0;
    } catch (const std::out_of_range&) {
        return 0.0;
    }
}

// STEP compound entities use the form #ID =( TYPE1 (params) TYPE2 (params) ... ); where each listed type contributes its own slice of attributes,
// this scans the inner body character-by-character and returns a map of typename -> raw params string for each component found,
// balanced paren tracking ensures inner nested lists (ex: control point refs "(#1,#2,#3)") are captured as one blob and never re-scanned,
// ex: body " BOUNDED_CURVE ( )  B_SPLINE_CURVE ( 2, (#1,#2), .F., .F., .F. )  CURVE ( ) " ->
//     { "BOUNDED_CURVE": "", "B_SPLINE_CURVE": " 2, (#1,#2), .F., .F., .F. ", "CURVE": "" }
static std::unordered_map<std::string, std::string> extractCompoundComponents(const std::string& body)
{
    std::unordered_map<std::string, std::string> result;
    int n = (int)body.size(), i = 0;
    while (i < n) {
        // skip any whitespace between components
        while (i < n && (body[i] == ' ' || body[i] == '\t' || body[i] == '\r' || body[i] == '\n'))
            i++;
        if (i >= n)
            break;
        // an identifier starts with an uppercase letter, then any mix of uppercase letters, digits, and underscores like "AXIS2_PLACEMENT_3D"
        if (!std::isupper((unsigned char)body[i])) {
            i++;
            continue;
        }
        int nameStart = i;
        while (i < n && (std::isupper((unsigned char)body[i]) || std::isdigit((unsigned char)body[i]) || body[i] == '_'))
            i++;
        std::string typeName = body.substr(nameStart, i - nameStart);
        // skip whitespace between the type name and its opening parenthesis, can't use trimWS because we are walking a character index inside a larger string
        while (i < n && (body[i] == ' ' || body[i] == '\t'))
            i++;
        // each valid component is immediately followed by (, anything else (ex: a stray word inside a quoted string that already got consumed) is skipped
        if (i >= n || body[i] != '(') {
            continue;
        }
        // balance-track from ( to find the matching ) for this component's param list, handles nested lists correctly
        int parenStart = i + 1;
        int depth = 1;
        i++;
        while (i < n && depth > 0) {
            if (body[i] == '(')
                depth++;
            else if (body[i] == ')')
                depth--;
            if (depth > 0)
                i++;
        }
        // i is now at the closing ), store this component then step past it
        result[typeName] = body.substr(parenStart, i - parenStart);
        i++;
    }
    return result;
}

StepMap parseStepFile(const std::string& path)
{
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("Cannot open: " + path);
    StepMap entityMap;
    std::string line, accumulator;
    bool inData = false;
    // matches a complete STEP entity line after accumulator flushes at ';':
    //   group 1 = numeric ID after '#', ex: "12" in "#12 = ..."
    //   group 2 = entity type keyword, ex: "CARTESIAN_POINT"
    //   group 3 = raw parameter string,  ex: "'',( 1.0, 2.0, 3.0)"
    // ex full match: "#12 = CARTESIAN_POINT('', (1.0, 2.0, 3.0));"
    static std::regex entityRegex(R"(#(\d+)\s*=\s*([A-Z0-9_]+)\s*\((.+)\)\s*;$)");
    // matches STEP compound entities, which list multiple supertypes inline instead of a single type name:
    //   group 1 = numeric ID, group 2 = everything between the outer ( and ) of the compound
    // ex: "#87 =( BOUNDED_CURVE ( )  B_SPLINE_CURVE ( 2, (#1,#2), .F., .F., .F. )  ... );"
    // the simple regex above fails for these because [A-Z0-9_]+ cannot match the ( immediately after =,
    // so we only try the compound regex when the simple one does not match
    static std::regex compoundEntityRegex(R"(#(\d+)\s*=\s*\((.+)\)\s*;$)");
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
        accumulator += trimmedLine;
        if (!accumulator.empty() && accumulator.back() == ';') {
            std::smatch match;
            if (std::regex_search(accumulator, match, entityRegex))
                entityMap[std::stoi(match[1])] = { match[2], match[3] };
            else if (std::regex_search(accumulator, match, compoundEntityRegex)) {
                // compound entity, decompose into typed components and synthesize a single StepEntity for downstream resolvers
                // currently handled compounds (ordered by priority):
                //   RATIONAL_B_SPLINE_CURVE: merges B_SPLINE_CURVE (degree, ctrl_pts) + B_SPLINE_CURVE_WITH_KNOTS (multiplicities, knots) +
                //       RATIONAL_B_SPLINE_CURVE (weights) into one params string that sampleBSpline can parse without any interface change
                //   B_SPLINE_CURVE_WITH_KNOTS: same merge without weights, stored under that type for non-rational splines
                //   AXIS2_PLACEMENT_3D: compound axis placements (CAD exporters sometimes wrap them in PLACEMENT, REPRESENTATION_ITEM supertypes)
                //       are reduced to their AXIS2_PLACEMENT_3D component which resolveAxis already knows how to read
                // unrecognised compound types land in none of these branches and are silently left out of the map,
                // any face that references them will fall through to SurfaceKind::Unknown and be skipped at tessellation time
                auto components = extractCompoundComponents(match[2]);
                std::string entityType, entityParams;
                if (components.count("B_SPLINE_CURVE") && components.count("B_SPLINE_CURVE_WITH_KNOTS")) {
                    // synthesise unified params: '', <B_SPLINE_CURVE: degree,ctrl_pts,form,closed,self_intersect>,
                    //     <B_SPLINE_CURVE_WITH_KNOTS: multiplicities,unique_knots,knot_spec>[, <RATIONAL_B_SPLINE_CURVE: weights>]
                    // this layout matches the existing sampleBSpline param indexing (degree at [1], ctrl_pts at [2],
                    // then float lists from [3] onward) so the function needs no signature change
                    entityType = components.count("RATIONAL_B_SPLINE_CURVE") ? "RATIONAL_B_SPLINE_CURVE" : "B_SPLINE_CURVE_WITH_KNOTS";
                    entityParams = "'', " + components["B_SPLINE_CURVE"] + ", " + components["B_SPLINE_CURVE_WITH_KNOTS"];
                    if (components.count("RATIONAL_B_SPLINE_CURVE"))
                        entityParams += ", " + components["RATIONAL_B_SPLINE_CURVE"];
                } else if (components.count("AXIS2_PLACEMENT_3D")) {
                    entityType = "AXIS2_PLACEMENT_3D";
                    entityParams = components["AXIS2_PLACEMENT_3D"];
                }
                if (!entityType.empty() && !entityParams.empty())
                    entityMap[std::stoi(match[1])] = { entityType, entityParams };
            }
            accumulator.clear();
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
        throw std::runtime_error("CARTESIAN_POINT/DIRECTION missing at #" + std::to_string(id));
    auto params = splitTopLevel(entityIt->second.params);
    // CARTESIAN_POINT params: (name, (x,y,z))
    // params[0] = name string (ex: "''"), params[1] = coordinate tuple "(1.0,2.0,3.0)"
    // DIRECTION shares the exact same layout (name, (dx,dy,dz)) so this function resolves both without any type check,
    // resolveAxis calls it for both the origin (CARTESIAN_POINT) and the two direction refs (DIRECTION) and it works for all of them
    if (params.size() < 2)
        throw std::runtime_error("CARTESIAN_POINT missing coords at #" + std::to_string(id));
    // unwrap "(1.0,2.0,3.0)" -> "1.0,2.0,3.0", then split to ["1.0","2.0","3.0"]
    auto coords = splitTopLevel(unwrap(trimWS(params[1])));
    if (coords.size() < 3)
        throw std::runtime_error("CARTESIAN_POINT has fewer than 3 coords at #" + std::to_string(id));
    return { dbl(coords[0]), dbl(coords[1]), dbl(coords[2]) };
}

AxisPlacement resolveAxis(int id, const StepMap& map)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        throw std::runtime_error("AXIS2_PLACEMENT_3D missing at #" + std::to_string(id));
    // AXIS2_PLACEMENT_3D params: (name, origin_ref, z_dir_ref, x_dir_ref)
    // z and x direction refs point to DIRECTION entities (which share the CARTESIAN_POINT layout)
    // ex: "('', #10, #11, #12)" -> origin=point[10], zDirection=point[11], xDirection=point[12]
    // entity type is not checked here, so AXIS, AXIS1_PLACEMENT, and compound-wrapped AXIS2_PLACEMENT_3D entities
    // with the same (name, origin, z_dir[, x_dir]) layout are resolved correctly without special-casing,
    // when x_dir_ref is absent (AXIS1_PLACEMENT with only 3 params) xDirection stays zero and tessPlane's
    // Gram-Schmidt fallback builds a valid perpendicular automatically
    auto params = splitTopLevel(entityIt->second.params);
    if (params.size() < 3)
        throw std::runtime_error("AXIS2_PLACEMENT_3D missing params at #" + std::to_string(id));
    AxisPlacement axis;
    axis.origin = resolvePoint(stepRef(params[1]), map);
    axis.zDirection = resolvePoint(stepRef(params[2]), map);
    if (params.size() >= 4)
        axis.xDirection = resolvePoint(stepRef(params[3]), map);
    return axis;
}

Surface resolveSurface(int id, const StepMap& map)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return { };
    Surface surface;
    auto params = splitTopLevel(entityIt->second.params);
    if (entityIt->second.type == "PLANE") {
        surface.kind = SurfaceKind::Plane;
        // PLANE params: (name, axis_placement_ref)
        // ex: "('flat_top', #20)" -> axis from entity #20
        if (params.size() < 2)
            throw std::runtime_error("PLANE missing axis ref at #" + std::to_string(id));
        surface.axis = resolveAxis(stepRef(params[1]), map);
    } else if (entityIt->second.type == "CYLINDRICAL_SURFACE") {
        surface.kind = SurfaceKind::Cylinder;
        // CYLINDRICAL_SURFACE params: (name, axis_placement_ref, radius)
        // ex: "('', #30, 5.0)" -> axis from #30, radius = 5.0
        if (params.size() < 3)
            throw std::runtime_error("CYLINDRICAL_SURFACE missing params at #" + std::to_string(id));
        surface.axis = resolveAxis(stepRef(params[1]), map);
        surface.majorRadius = dbl(params[2]);
    } else if (entityIt->second.type == "TOROIDAL_SURFACE") {
        surface.kind = SurfaceKind::Torus;
        // TOROIDAL_SURFACE params: (name, axis_placement_ref, major_radius, minor_radius)
        // ex: "('', #40, 10.0, 2.0)" -> fillet ring: center-circle R=10, tube r=2
        if (params.size() < 4)
            throw std::runtime_error("TOROIDAL_SURFACE missing params at #" + std::to_string(id));
        surface.axis = resolveAxis(stepRef(params[1]), map);
        surface.majorRadius = dbl(params[2]);
        surface.minorRadius = dbl(params[3]);
    }
    // unsupported surface types (B_SPLINE_SURFACE, ...) fall through with SurfaceKind::Unknown,
    // tessellateAdvancedFace returns {} for unknown kinds and the face is skipped silently
    return surface;
}

#pragma region sample curves
// sample a CIRCLE arc from startPt to endPt (or full revolution if isClosedLoop) with a radius and arcSegs LOD
// sameSense true = CCW sweep (angleEnd > angleStart), false = CW sweep (angleEnd < angleStart, short arc the other way)
// ex: same_sense=.T., edgeReversed=true -> effective false -> CW sweep takes the 75 deg arc, not the 285 deg one
static std::vector<Vec3> sampleCircle(int id, Vec3 startPt, Vec3 endPt, bool isClosedLoop, const StepMap& map, int arcSegs, bool sameSense = true)
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
    Vec3 O = placement.origin, Z = placement.zDirection.norm(), X = placement.xDirection.norm(), Y = Z.cross(X).norm();

    // returns the angle of point in the circle's local XY plane (atan2 in [-pi, pi])
    // ex: point on the +X side -> 0.0,  point on the +Y side -> pi/2
    auto angleOf = [&](Vec3 point) {
        Vec3 delta = point - O;
        return std::atan2(delta.dot(Y), delta.dot(X));
    };

    double angleStart, angleEnd;
    if (isClosedLoop) {
        // full revolution: 0 -> 2pi, sample 'arcSegs' segments
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
    // ex: arcSegs=48, quarter arc (pi/2 out of 2pi) -> numSegments = max(2, 48*0.25) = 12 segments
    // always emit at least 2 points (start + end) for degenerate near-zero arcs
    int numSegments = isClosedLoop ? arcSegs : std::max(2, (int)(arcSegs * std::abs(angleEnd - angleStart) / (2 * M_PI)));
    std::vector<Vec3> points;
    points.reserve(numSegments + 1);
    for (int i = 0; i <= numSegments; i++) {
        double angle = angleStart + (angleEnd - angleStart) * (double)i / numSegments;
        // finally the math curve part, point on circle = O + radius*(cos(angle)*X + sin(angle)*Y)
        points.push_back(O + X * std::cos(angle) * radius + Y * std::sin(angle) * radius);
    }
    // the circle of radius 'radius' evaluated with arcSegs+1 points, tessGrid will copy it along the axis from start to end every vSteps, then connect
    // the copied circles points together via UV grid, in real CAD kernels NURBS can do everything so we wouldn't have such perfect circle only sampling code
    return points;
}

// evaluate a B_SPLINE_CURVE_WITH_KNOTS or RATIONAL_B_SPLINE_CURVE (NURBS) using De Boor's algorithm
// each round takes the current set of local control points and replaces them with a smaller set of blended points, until one point remains
// basically what De Casteljau's algorithm is to Bézier curves, numerically stable and the standard way to evaluate splines
// imagine you're mixing paint colors, you have a row of 4 paint pots, in the first round you blend each adjacent pair together, getting 3 new mixed colors,
// then you blend each adjacent pair of those, getting 2, then again for 1 final color:
//  1    2    3    4
//    12   23   34
//      123  234
//        1234
// where you start blending (the t parameter) determines how much of each neighbor contributes at each round, so starting near pot 1 gives a result close to
// pot 1's color and starting near pot 5 gives a result close to pot 5's color, the final single color is the point on the curve at that parameter value
// RATIONAL_B_SPLINE_CURVE (NURBS) extends the same pyramid with per-point weights: run De Boor in homogeneous 4D (x*w, y*w, z*w, w), divide back to 3D,
// which lets it represent exact conics (circles, ellipses) that a non-rational B-spline can only approximate
static std::vector<Vec3> sampleBSpline(int id, Vec3 startPt, Vec3 endPt, const StepMap& map, int arcSegs)
{
    auto entityIt = map.find(id);
    if (entityIt == map.end())
        return { startPt, endPt };
    // B_SPLINE_CURVE_WITH_KNOTS params:
    //   (name, degree, (ctrl_pts...), curve_form, closed_curve, self_intersect, (knot_multiplicities...), (knots...), knot_spec)
    // ex degree-3 curve: "('', 3, (#60,#61,#62,#63), .UNSPECIFIED., .F., .F., (4,4), (0.,1.), .UNSPECIFIED.)"
    // degree = how many neighboring pots influence each blend round, 1 = only immediate neighbor, 3 = standard for CAD, smooth enough that you cannot feel the
    // joints at all ctrl_pts = the paint pots themselves, the curve is attracted to them but does not necessarily touch them (except at the endpoints)
    // curve_form means the overall shape family: .UNSPECIFIED., .CIRCULAR_ARC. or .ELLIPTIC_ARC. (informational only)
    // closed_curve .F. means the row of pots has a start and an end, .T. means it loops back on itself (informational only)
    // self_intersect .F. means the resulting path never crosses itself (informational only)
    // knot_multiplicities = how many times each knot is repeated, repeating a knot degree+1 times at each end forces the curve to actually
    //      start and end at the first and last pots rather than just being attracted to them, this is called a clamped B-spline
    // knots = the t values where blending behavior changes, (0., 1.) combined with multiplicities (4,4) expands to (0,0,0,0,1,1,1,1),
    //      8 values for 4 pots at degree 3, satisfying the n + degree + 1 rule
    // knot_spec = knot distribution type (uniform, quasi-uniform...), .UNSPECIFIED. is almost always what you see in exported STEP files
    // for RATIONAL_B_SPLINE_CURVE (synthesised from a compound entity), weights are appended as a fourth parenthesised list after knot_spec
    bool isRational = (entityIt->second.type == "RATIONAL_B_SPLINE_CURVE");
    auto params = splitTopLevel(entityIt->second.params);
    if (params.size() < 3)
        return { startPt, endPt };
    int degree = (int)dbl(params[1]);
    std::vector<Vec3> controlPoints;
    for (auto& controlRef : splitTopLevel(unwrap(trimWS(params[2])))) {
        int refId = stepRef(controlRef);
        if (refId > 0)
            controlPoints.push_back(resolvePoint(refId, map));
    }
    // need at least degree+1 pots to have enough neighbors to blend down to 1
    if ((int)controlPoints.size() <= degree)
        return { startPt, endPt };
    // scan remaining params for up to three parenthesised float lists in STEP spec order:
    //   first  = knot multiplicities (how many times each unique knot value repeats)
    //   second = unique knot values (the actual t positions where blending behaviour changes)
    //   third  = weights for rational curves (RATIONAL_B_SPLINE_CURVE only, absent for plain B-splines)
    // non-numeric parenthesised lists (ex: ctrl_pt ref list already consumed at params[2]) are detected by the '#' prefix check and skipped
    std::vector<double> multiplicityVector, uniqueKnots, weights;
    int floatListsSeen = 0;
    for (int i = 3; i < (int)params.size(); i++) {
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
            // check the token looks like a number before calling dbl() so we don't confuse a parse failure (returns 0) with a genuine 0 knot
            bool looksNumeric = !trimmedValue.empty()
                && (std::isdigit((unsigned char)trimmedValue[0]) || trimmedValue[0] == '-' || trimmedValue[0] == '+' || trimmedValue[0] == '.');
            if (!looksNumeric) {
                allNumeric = false;
                break;
            }
            candidates.push_back(dbl(trimmedValue));
        }
        if (!allNumeric)
            continue;
        floatListsSeen++;
        // first plain-float list = multiplicities, second = unique knots (STEP spec ordering), third = weights for rational curves
        if (floatListsSeen == 1)
            multiplicityVector = candidates;
        else if (floatListsSeen == 2)
            uniqueKnots = candidates;
        else if (floatListsSeen == 3)
            weights = candidates;
    }
    // expand the knot vector, STEP B_SPLINE_CURVE_WITH_KNOTS stores (unique knot values, multiplicity counts) separately,
    // De Boor needs the full repeated sequence where each unique knot appears multiplicity[k] times consecutively,
    // a multiplicity-d cluster at each clamped endpoint forces the curve to interpolate the first and last control points exactly
    // ex: multiplicities=(3,2,2,3), unique knots=(0.0, 0.5, 0.75, 1.0) -> expanded=(0,0,0, 0.5,0.5, 0.75,0.75, 1,1,1)
    // fallback: if no second list was found but the first list is large enough, treat it as a pre-expanded knot vector
    std::vector<double> knots;
    if (!multiplicityVector.empty() && multiplicityVector.size() == uniqueKnots.size()) {
        for (int k = 0; k < (int)uniqueKnots.size(); k++) {
            int mult = std::max(1, (int)std::round(multiplicityVector[k]));
            for (int j = 0; j < mult; j++)
                knots.push_back(uniqueKnots[k]);
        }
    } else if (uniqueKnots.empty() && (int)multiplicityVector.size() >= (int)controlPoints.size() + degree + 1) {
        // non-standard: first list is already the full expanded knot vector (no multiplicity encoding)
        knots = multiplicityVector;
    }
    if (knots.empty() || (int)knots.size() < (int)controlPoints.size() + degree + 1)
        return { startPt, endPt };
    // valid blending range = [knots[degree], knots[n-1-degree]], outside this range there aren't enough pots on both sides to blend
    // ex: knots=(0,0,0,0,1,1,1,1), degree=3 -> tMin=knots[3]=0, tMax=knots[4]=1
    double tMin = knots[degree], tMax = knots[(int)knots.size() - 1 - degree];

    // De Boor's algorithm: given t, find which group of pots are the local neighbors, then blend them down round by round until 1 color remains
    auto deBoor = [&](double t) -> Vec3 {
        // find which knot span t falls into, i.e. which group of pots are the local neighbors for this t
        int spanId = degree, lastSpan = (int)knots.size() - 2 - degree;
        for (int i = degree; i < (int)knots.size() - 1 - degree; i++) {
            if (t < knots[i + 1]) {
                spanId = i;
                break;
            }
            spanId = lastSpan;
        }
        // grab the local pots for this span, degree+1 pots = one full blending pyramid
        std::vector<Vec3> localPoints(controlPoints.begin() + spanId - degree, controlPoints.begin() + spanId + 1);
        // blend round by round: each round reduces the row of pots by 1 until only 1 remains
        // ex: degree=3 starts with 4 pots -> round 1 gives 3 -> round 2 gives 2 -> round 3 gives the final 1
        for (int r = 1; r <= degree; r++)
            for (int j = degree; j >= r; j--) {
                double knotSpanWidth = knots[spanId - degree + j + r] - knots[spanId - degree + j];
                // alpha says how close t is to the right pot vs the left pot in this pair, 0 = all left, 1 = all right
                // guard against repeated knots at boundaries (knotSpanWidth=0) which would cause division by zero, pin to left pot in that case
                double alpha = (knotSpanWidth < 1e-12) ? 0.0 : (t - knots[spanId - degree + j]) / knotSpanWidth;
                localPoints[j] = localPoints[j - 1] * (1 - alpha) + localPoints[j] * alpha;
            }
        return localPoints[degree];
    };

    if (isRational && weights.size() == controlPoints.size()) {
        // each paint pot gets a stickiness value baked in, a sticky pot (w>1) pulls the final color toward itself more than a normal pot,
        // to make the standard blending pyramid work with stickiness, pre-multiply each pot's color by its stickiness and carry the stickiness
        // itself as a 4th channel (x*w, y*w, z*w, w), the pyramid blends all 4 channels identically to plain De Boor (same span-find and alpha formula),
        // at the end divide the 3 color channels by the accumulated stickiness channel to recover the true color,
        // a pot with w=1 contributes normally so this is a strict generalization (plain De Boor is just the w=1 case),
        // ex: a degree-2 circular arc is encoded with weights (1, cos(half_angle), 1), the middle pot is slightly less sticky (w<1) which relaxes
        // it away from the control polygon just enough to trace the exact circle, the non-rational path can only approximate this
        std::vector<std::array<double, 4>> homogeneousPoints; // the "weighted pots"
        homogeneousPoints.reserve(controlPoints.size());
        for (int i = 0; i < (int)controlPoints.size(); i++) {
            double weight = weights[i];
            homogeneousPoints.push_back({ controlPoints[i].x * weight, controlPoints[i].y * weight, controlPoints[i].z * weight, weight });
        }
        auto deBoorRational = [&](double t) -> Vec3 {
            int spanId = degree, lastSpan = (int)knots.size() - 2 - degree;
            for (int i = degree; i < (int)knots.size() - 1 - degree; i++) {
                if (t < knots[i + 1]) {
                    spanId = i;
                    break;
                }
                spanId = lastSpan;
            }
            std::vector<std::array<double, 4>> localPoints(homogeneousPoints.begin() + spanId - degree, homogeneousPoints.begin() + spanId + 1);
            for (int r = 1; r <= degree; r++)
                for (int j = degree; j >= r; j--) {
                    double knotSpanWidth = knots[spanId - degree + j + r] - knots[spanId - degree + j];
                    double alpha = (knotSpanWidth < 1e-12) ? 0.0 : (t - knots[spanId - degree + j]) / knotSpanWidth;
                    for (int d = 0; d < 4; d++)
                        localPoints[j][d] = localPoints[j - 1][d] * (1.0 - alpha) + localPoints[j][d] * alpha;
                }
            // project (x*w, y*w, z*w, w) back to (x, y, z), same degenerate sentinel guard as Vec3::norm
            double w = localPoints[degree][3];
            return (w > 1e-14) ? Vec3 { localPoints[degree][0] / w, localPoints[degree][1] / w, localPoints[degree][2] / w } : Vec3 { 0, 0, 1 };
        };
        std::vector<Vec3> points;
        points.reserve(arcSegs + 1);
        for (int i = 0; i <= arcSegs; i++)
            points.push_back(deBoorRational(tMin + (tMax - tMin) * (double)i / arcSegs));
        return points;
    }

    // ask for the final mixed color at arcSegs+1 evenly spaced positions along the row of pots, from the first pot to the last,
    // and collecting all those colors into a list, that list is the polyline approximating the curve
    std::vector<Vec3> points;
    points.reserve(arcSegs + 1);
    for (int i = 0; i <= arcSegs; i++)
        points.push_back(deBoor(tMin + (tMax - tMin) * (double)i / arcSegs));
    return points; // same as circle, NURBS would do it all and better since it can do a perfect circle which simple B-spline cannot
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
        throw std::runtime_error("FACE_BOUND missing at #" + std::to_string(boundId));
    loop.isOuter = (boundIt->second.type == "FACE_OUTER_BOUND");
    // FACE_OUTER_BOUND / FACE_BOUND params: (name, edge_loop_ref, orientation)
    // ex: "( 'NONE', #318, .T. )" -> EDGE_LOOP is entity #318 (orientation is "edge loop runs clockwise or counterclockwise" but we don't care yet)
    auto boundParams = splitTopLevel(boundIt->second.params);
    if (boundParams.size() < 2)
        throw std::runtime_error("FACE_BOUND missing params at #" + std::to_string(boundId));
    auto loopIt = map.find(stepRef(boundParams[1]));
    if (loopIt == map.end() || loopIt->second.type != "EDGE_LOOP") // ref should always lead to EDGE_LOOP
        throw std::runtime_error("EDGE_LOOP missing at #" + std::to_string(stepRef(boundParams[1])));
    // EDGE_LOOP params: (name, (oriented_edge_ref, ...))
    // ex: "( 'NONE', ( #2513, #123 ) )" -> 2 oriented edges ("edges" aren't necessarily only 2 points,
    // for a circle we could have 2 loops each handle 180° of it for example)
    auto loopParams = splitTopLevel(loopIt->second.params);
    if (loopParams.size() < 2)
        throw std::runtime_error("EDGE_LOOP missing params at #" + std::to_string(stepRef(boundParams[1])));

    // since EDGE_LOOP has multiple refs, start looping through each
    for (auto& edgeRef : splitTopLevel(unwrap(trimWS(loopParams[1])))) {
        int orientedEdgeId = stepRef(edgeRef);
        auto orientedEdgeIt = map.find(orientedEdgeId);
        if (orientedEdgeIt == map.end() || orientedEdgeIt->second.type != "ORIENTED_EDGE")
            continue;
        // ORIENTED_EDGE params: (name, *, *, edge_curve_ref, orientation_flag)
        // orientation_flag ".F." means the edge is traversed in reverse (end->start)
        // ex: "( 'NONE', *, *, #2872, .T. )" -> traverse edge #2872 forward
        auto orientedEdgeParams = splitTopLevel(orientedEdgeIt->second.params);
        if (orientedEdgeParams.size() < 5)
            continue;
        bool edgeReversed = (trimWS(orientedEdgeParams[4]) == ".F.");
        auto edgeCurveIt = map.find(stepRef(orientedEdgeParams[3]));
        if (edgeCurveIt == map.end() || edgeCurveIt->second.type != "EDGE_CURVE")
            continue;
        // EDGE_CURVE params: (name, start_vertex_ref, end_vertex_ref, curve_ref, same_sense)
        // ex: "( 'NONE', #1950, #238, #2523, .T. )" -> goes from vtx #1950 to vtx #238 along curve #2523
        // when we say "curve" edge of a face, it can be a curve for cylinder or torus but also a line for a plane, it's just a general term
        // same_sense = does the curve follow its parent's (ORIENTED_EDGE) orientation_flag (up there)
        auto edgeCurveParams = splitTopLevel(edgeCurveIt->second.params);
        if (edgeCurveParams.size() < 4)
            continue;
        bool rawSameSense = !(edgeCurveParams.size() >= 5 && trimWS(edgeCurveParams[4]) == ".F.");
        // if both are flipped, they cancel out and you're back to the original arc direction (with 2 points on a circle there's always a shorter and longer arc
        // (or both equal if opposite points), same_sense rawSameSense with edgeReversed tells you which way to go, get it wrong and instead of a
        // 30 degree fillet arc you'd sample the 330 degree arc going the other way around the circle for example)
        // so use XOR for two reversals = no reversal, one reversal = reversed
        bool sameSense = rawSameSense != edgeReversed;

        int startVertexId = stepRef(edgeCurveParams[1]), endVertexId = stepRef(edgeCurveParams[2]);
        // if start and end vertex entity are the same ref, this edge is a closed circle (full revolution)
        // ex: if "#1950 = VERTEX_POINT" for both startVertexId and endVertexId = full-circle edge
        bool fullCircle = (startVertexId == endVertexId && startVertexId > 0);
        if (fullCircle)
            loop.hasFullCircle = true;

        // look up a VERTEX_POINT to get its 3D position
        auto resolveVertex = [&](int vertexId) -> Vec3 {
            auto vertexIt = map.find(vertexId);
            if (vertexIt == map.end() || vertexIt->second.type != "VERTEX_POINT")
                return { };
            // VERTEX_POINT params: (name, cartesian_point_ref), ex: "( 'NONE', #2929 )" -> position = point[2929]
            auto vertexParams = splitTopLevel(vertexIt->second.params);
            return (vertexParams.size() >= 2) ? resolvePoint(stepRef(vertexParams[1]), map) : Vec3 { };
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
                edgeSamples = { vertexStart, vertexEnd }; // step calls lines "curves" (just straight ones)
            else if (curveIt->second.type == "B_SPLINE_CURVE_WITH_KNOTS" || curveIt->second.type == "RATIONAL_B_SPLINE_CURVE")
                edgeSamples = sampleBSpline(curveId, vertexStart, vertexEnd, map, arcSegs);
            else
                edgeSamples = { vertexStart, vertexEnd };
        } else
            edgeSamples = { vertexStart, vertexEnd };

        // append edge samples, skipping the junction duplicate with the previous edge
        // ex: prev edge ended at P, this edge starts at P -> skip index 0 to avoid [P,P] in the list
        int startId = loop.points.empty() ? 0 : 1;
        for (int i = startId; i < (int)edgeSamples.size(); i++)
            loop.points.push_back(edgeSamples[i]);
    }
    // remove closing duplicate: STEP loops are closed, so the last point often coincides with the first, drop it so the polygon is a proper open ring
    // ex: [A,B,C,D,A] -> [A,B,C,D]
    if (loop.points.size() > 1 && loop.points.front().near(loop.points.back()))
        loop.points.pop_back();
    return loop;
}