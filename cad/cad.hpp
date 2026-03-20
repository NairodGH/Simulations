#include "main.hpp"

//  Raylib's Vec3 is float, we use Vec3 with doubles for STEP geometry computation where precision matters (ex: detecting coincident vertices)
struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(Vec3 o) const { return { x + o.x, y + o.y, z + o.z }; }
    Vec3 operator-(Vec3 o) const { return { x - o.x, y - o.y, z - o.z }; }
    Vec3 operator*(double t) const { return { x * t, y * t, z * t }; }
    // measures how much two vectors point in the same direction (1 = same, 0 = perpendicular, -1 = opposite)
    double dot(Vec3 o) const { return x * o.x + y * o.y + z * o.z; }
    // gives a vector perpendicular to both, and its sign (of its Z component in 2D) tells you which side of a line a point is on
    // positive = left turn, negative = right turn
    Vec3 cross(Vec3 o) const { return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x }; }
    double len() const { return std::sqrt(x * x + y * y + z * z); }
    // returns a unit-length copy of this vector, or (0,0,1) if the vector is degenerate (near-zero length)
    // the 1e-14 threshold guards against division by zero from floating-point underflow
    // ex: (3,0,0).norm() -> (1,0,0)
    //     (0,0,0).norm() -> (0,0,1) (fallback sentinel direction)
    Vec3 norm() const
    {
        double length = len();
        return length > 1e-14 ? Vec3 { x / length, y / length, z / length } : Vec3 { 0, 0, 1 };
    }
    // returns true if this vector is within eps distance of 'other' (Euclidean)
    // used to detect duplicate vertices at loop seams and closing points
    // ex: (1,0,0).near((1,0,0.000001)) -> true (within default eps=1e-5)
    //     (1,0,0).near((1,0,0.1)) -> false
    //     (1,0,0).near((1,0,0.000001), 1e-9) -> false (tighter eps)
    bool near(Vec3 other, double eps = 1e-5) const { return (*this - other).len() < eps; }
};

struct Vec2 {
    double u = 0, v = 0;
};

// STEP AXIS2_PLACEMENT_3D
struct AxisPlacement {
    Vec3 origin, zDir, xDir;
};

// surface types we handle
enum class SurfaceKind { Plane, Cylinder, Torus, Unknown };

// holds everything needed to evaluate any of the three supported surface types at any point.
// plane:    no radii, origin = any point on the plane, zDir = surface normal,
//           xDir = one tangent direction (yDir is cross product, making up the full plane coord system)
// cylinder: origin = any point on the axis, zDir = axis direction,
//           xDir = zero-angle reference (where u=0 starts sweeping around)
//           majorRadius = distance from axis to surface
// torus:    picture a donut, origin = center of the hole, zDir = axis through the hole
//           xDir = direction where u=0 starts (irrelevant for a full donut, matters for fillet patches)
//           majorRadius = distance from donut center to tube centerline (size of the hole)
//           minorRadius = radius of the tube itself (thickness of the dough) from where majorRadius ends up
//           torus minorRadius plays the same role as cylinder majorRadius
struct Surface {
    SurfaceKind kind = SurfaceKind::Unknown;
    AxisPlacement axis;
    double majorRadius = 0;
    double minorRadius = 0;
};

// a boundary loop is the outline of one face, a closed chain of edges that says "here is where this surface patch ends",
// a face can have one outer loop and zero or more inner loops (holes)
struct BoundaryLoop {
    std::vector<Vec3> points;
    bool isOuter = false; // FACE_OUTER_BOUND vs FACE_BOUND for ear-clipper, imagine a ring: true means it makes up the outer edge and false the inner one
    bool hasFullCircle = false; // any edge where vertexStartId==vertexEndId means full-revolution arc (true)
};

// one tessellated face ready for GPU upload
struct TessellatedFace {
    SurfaceKind kind = SurfaceKind::Unknown;
    std::vector<float> vertices; // interleaved XYZ
    std::vector<float> normals; // interleaved XYZ
    std::vector<int> indices; // triangle indices
};

struct StepEntity {
    std::string type, params;
};
using StepMap = std::unordered_map<int, StepEntity>;

// mutable height extent of a cylinder face along its axis, used for geometry healing when a connected cap plane is translated,
// stored separately so the healing pass can retessellate the cylinder with the updated range without re-parsing STEP
// for non-cylinder faces both values are 0 and the struct is never read
struct CylinderHeightRange {
    double heightMin = 0, heightMax = 0;
};

// snapshot of one healed cylinder's height range at the moment the undo entry was captured
struct CylinderHeightSnapshot {
    int cylFaceIdx;
    CylinderHeightRange rangeBefore;
};

// one entry on the undo/redo stack
struct UndoEntry {
    int faceIndex; // which face moved
    Vector3 offsetBefore; // what its offset was before the move began
    std::vector<CylinderHeightSnapshot> cylSnapshots; // height ranges of every cylinder healed during this gesture, empty when no heal occurred
};

// one entry in the per-gesture heal cache, which cylinder to extend and which cap to move
struct CylinderHealEntry {
    int cylFaceIdx;
    bool isMaxCap; // true = move heightMax, false = move heightMin
    double axisDotNormal; // dot(cylAxis, planeNormal) = about 1, preserves sign for signedDelta each frame
};

// final GPU model, many are SoA (Struct of Arrays) related index wise for their specific tasks
struct CadModel {
    // SoA per mesh
    std::vector<Mesh> meshes;
    std::vector<Color> colors;
    // SoA per face
    std::vector<TessellatedFace> pickData; // CPU copy kept after GPU upload, for mouse ray picking and analysis
    std::vector<Surface> faceSurfaces; // analytical surface definition per face, for axis/normal display
    std::vector<float> faceAreas;
    std::vector<Vector3> faceOffsets; // per-face translation in draw space, applied on top of the centering transform at draw/query time
    std::vector<CylinderHeightRange> cylHeightRanges; // per-face cylinder axis height range, kept mutable for geometry healing, non-cylinders are zeroed
    // SoA per translation
    std::vector<UndoEntry> undoStack; // pushed once on gesture
    std::vector<UndoEntry> redoStack; // cleared on new translation, populated by undo
    // measurements
    BoundingBox bbox;
    int totalTriangleCount = 0;
    int selectedFace = -1;
    int distFace = -1;
    ~CadModel();
};

// parser utils
std::vector<std::string> splitTopLevel(const std::string& input);
std::string trimWS(const std::string& input);
std::string unwrap(const std::string& input);
int stepRef(const std::string& input);
double dbl(const std::string& input);

// parser.cpp: STEP file parsing, entity resolution, curve sampling, boundary loop resolution
StepMap parseStepFile(const std::string& path);
Vec3 resolvePoint(int id, const StepMap& map);
AxisPlacement resolveAxis(int id, const StepMap& map);
Surface resolveSurface(int id, const StepMap& map);
BoundaryLoop sampleLoop(int boundId, const StepMap& map, int arcSegs);

// tessellator.cpp: UV grid, face tessellation, GPU upload, geometry healing
TessellatedFace tessGrid(SurfaceKind kind, std::function<Vec3(double u, double v)> positionFn, std::function<Vec3(double u, double v)> normalFn, double u0,
    double u1, int uSteps, double v0, double v1, int vSteps);
TessellatedFace tessellateAdvancedFace(int faceId, const StepMap& map, int arcSegs, Surface* outSurface = nullptr);
float computeFaceArea(const TessellatedFace& face);
Mesh uploadMesh(const TessellatedFace& tessellatedFace);
void retessCylinderFace(CadModel& model, int cylIdx, double newHeightMin, double newHeightMax);
std::vector<CylinderHealEntry> buildCylinderHealCache(const CadModel& model, int planeFaceIdx);
void applyCylinderHealCache(CadModel& model, std::vector<CylinderHealEntry>& cache, Vec3 planeNormal, Vector3 delta);

// cad.cpp: model loading and drawing
CadModel loadStep(const std::string& path, int arcSegs = 48);
void drawCadModel(const CadModel& model);