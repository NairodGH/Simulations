#include "main.hpp"

//  Raylib's Vector3 is float, we use Vec3 with doubles for STEP geometry computation where precision matters
struct Vec3 {
    double x = 0, y = 0, z = 0;
    // const refs to avoid a copy
    Vec3 operator+(const Vec3& o) const { return { x + o.x, y + o.y, z + o.z }; }
    Vec3 operator-(const Vec3& o) const { return { x - o.x, y - o.y, z - o.z }; }
    Vec3 operator*(double t) const { return { x * t, y * t, z * t }; }
    // measures how much two vectors point in the same direction (1 = same, 0 = perpendicular, -1 = opposite)
    double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
    // gives a vector perpendicular to both, and its sign (or its Z component in 2D) tells you which side of a line a point is on
    // positive = left turn, negative = right turn
    Vec3 cross(const Vec3& o) const { return { y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x }; }
    double len() const { return std::sqrt(x * x + y * y + z * z); }
    // returns a unit-length (each component between 0-1) copy of this vector, or (0,0,1) if the vector is degenerate (near-zero length)
    // the 1e-14 threshold guards against division by zero from floating-point underflow
    // ex: (3,0,0).norm() -> (1,0,0)
    //     (0,0,0).norm() -> (0,0,1) (fallback sentinel direction, 0 0 0 or NaN NaN NaN would crash, at least here we can see the wrong xd)
    Vec3 norm() const
    {
        double length = len();
        return length > 1e-14 ? Vec3 { x / length, y / length, z / length } : Vec3 { 0, 0, 1 };
    }
    // returns true if this vector is within eps distance of 'other' (Euclidean), used to detect duplicate vertices
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
    Vec3 origin, zDirection, xDirection;
};

// surface types we handle
enum class SurfaceKind { Plane, Cylinder, Torus, Unknown };

// holds everything needed to evaluate any of the three supported surface types at any point.
// plane:    no radii, origin = any point on the plane, zDirection = surface normal,
//           xDirection = one tangent direction (yDir is cross product, making up the full plane coord system)
// cylinder: origin = any point on the axis, zDirection = axis direction,
//           xDirection = zero-angle reference (where u=0 starts sweeping around)
//           majorRadius = distance from axis to surface
// torus:    picture a donut, origin = center of the hole, zDirection = axis through the hole
//           xDirection = direction where u=0 starts (irrelevant for a full donut, matters for fillet patches)
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
    bool isOuter = false; // FACE_OUTER_BOUND vs FACE_BOUND for ear-clipper, imagine a ring where true means it makes up the outer edge and false the inner one
    bool hasFullCircle = false; // any edge where vertexStartId==vertexEndId means full-revolution arc (true)
};

// one tessellated face ready for GPU upload
struct TessellatedFace {
    SurfaceKind kind = SurfaceKind::Unknown;
    std::vector<float> vertices; // interleaved XYZ (x0, y0, z0, x1, y1, z1, ...)
    std::vector<float> normals; // same
    std::vector<int> indices; // triangle indices
};

struct StepEntity {
    std::string type, params;
};
using StepMap = std::unordered_map<int, StepEntity>;

// mutable height extent of a cylinder face along its axis, used for geometry healing when a connected cap plane is translated,
// stored separately so the healing pass can retessellate the cylinder with the updated range without re-parsing STEP,
// for non-cylinder faces both values are 0 and the struct is never read
struct CylinderHeightRange {
    double heightMin = 0, heightMax = 0;
};

// snapshot of one healed cylinder's height range at the moment the undo entry was captured
struct CylinderHeightSnapshot {
    int cylFaceId;
    CylinderHeightRange oldHeight;
};

// one entry on the undo/redo stack
struct UndoEntry {
    int faceId; // which face moved
    Vector3 oldOffset; // what its offset was before the move began
    std::vector<CylinderHeightSnapshot> cylSnapshots; // height ranges of every cylinder healed during this gesture, empty when no heal occurred
    // (faceId, oldOffset) for every face down the constraints chain moved by constraint propagation
    std::vector<std::pair<int, Vector3>> propagatedOffsets;
};

// one entry in the per-gesture heal cache, which cylinder to extend and which cap to move
struct CylinderHealEntry {
    int cylFaceId;
    bool isMaxCap; // true = move heightMax, false = move heightMin
    double axisDotNormal; // dot(cylAxis, planeNormal) = about 1, preserves sign for signedDelta each frame
};

enum class ConstraintKind { Distance, Symmetry };

// one active constraint pairing between two faces
struct ConstraintPair {
    int faceA = -1, faceB = -1;
    bool hasDistance = false; // both faces must stay apart from each other by the declaration time distance
    float targetDistance = 0.0f; // which is stored here
    bool hasSymmetry = false; // both faces must stay equidistant from the declaration time midpoint
    float symmetryMidpoint = 0.0f; // which is stored here
};

// fading notification shown briefly at the bottom-center of the viewport
struct Toast {
    std::string message;
    float timeLeft = 0.0f;
};

// final GPU model (but more of a global bin struct xd), many are SoA (Struct of Arrays) related index wise for their specific tasks
// SoA is the right choice for CAD because operations are almost never "do everything to one face" but instead are
// "do one thing to all faces" or "do one thing to a subset of faces" so it's better to separate concerns than doing a big
// struct Face { Mesh mesh; Color color; ... };
struct CadModel {
    // SoA per mesh
    std::vector<Mesh> meshes;
    std::vector<Color> colors;

    // SoA per face
    std::vector<TessellatedFace> cpuFaceData; // CPU copy kept after GPU upload, for mouse ray picking and analysis
    std::vector<Surface> faceSurfaces; // analytical surface definition per face, for axis/normal display
    std::vector<float> faceAreas;
    std::vector<Vector3> faceOffsets; // per-face translation in draw space, applied on top of the centering transform at draw/query time
    std::vector<CylinderHeightRange> cylHeightRanges; // per-face cylinder axis height range, kept mutable for geometry healing, non-cylinders are zeroed

    // per-gesture working states, stack persists for undo/redo
    std::vector<UndoEntry> undoStack; // pushed once on gesture
    std::vector<UndoEntry> redoStack; // cleared on new translation, populated by undo
    std::vector<CylinderHealEntry> healCache; // built at gesture start, cleared at end
    std::vector<std::pair<int, std::vector<CylinderHealEntry>>> propagatedHealCaches; // same, (faceId, healEntries) per propagated face
    int cachedHealFace = -1; // selectedFace at the time the heal cache was built
    bool translating = false; // on translation key pressed
    bool wasTranslating = false; // every frame after 1rst one while we're pressing
    bool toastPushedThisGesture = false;

    // persistent interaction states
    std::vector<ConstraintPair> constraints;
    std::vector<Toast> toasts;
    int selectedFace = -1;
    int secondFace = -1;
    bool needsReset = false; // "persistent" for 1 frame lmao

    // load-time centering anchor set once during loadStep from raw STEP vertex positions (no offsets),
    // used only to derive modelCenter() which is the fixed origin for all draw-space transforms,
    // for the live extent including current face offsets use computeModelBBox()
    BoundingBox bbox;
    int totalTriangleCount = 0; // will never change, even for geometry healed cylinders because UV grid tessellation doesnt create new triangles

    // ok crazy stuff next (spent way too much searching for the problem): C++ Rule of Five says if you define any one of destructor, copy constructor, copy
    // assignment, move constructor or move assignment, you probably need to define all five because they're all related to the same question of "who owns
    // what and what happens when ownership transfers or ends", here we define destructor to unload our meshes from GPU (which it doesnt do on its own)

    // the compiler's position is "you touched one, so I'm not sure what you want for the others, I'll either delete them or fall back to the old behavior
    // rather than guess wrong", funny thing is since we defined destructor it decides to destroy move assignement whose old behavior
    // (didn't exist before C++11) was copy, it copies IDs instead of moving them -> temporary destructor frees them -> live object holds dead handles -> gg
    // tldr; compiler's caution about not generating something potentially wrong leads it to silently do something definitely wrong, so we have to explicitely
    // define everything (delete copy which we dont want and use default constructor/move)
    CadModel() = default;
    CadModel(const CadModel&) = delete;
    CadModel& operator=(const CadModel&) = delete;
    CadModel(CadModel&&) = default;
    CadModel& operator=(CadModel&&) = default;
    ~CadModel();
};

// cad.cpp
CadModel loadStep(const std::string& path, int arcSegs = 48);
void drawCadModel(const CadModel& model);

// parser.cpp
std::vector<std::string> splitTopLevel(const std::string& input);
std::string trimWS(const std::string& input);
std::string unwrap(const std::string& input);
int stepRef(const std::string& input);
double dbl(const std::string& input);
StepMap parseStepFile(const std::string& path);
Vec3 resolvePoint(int id, const StepMap& map);
AxisPlacement resolveAxis(int id, const StepMap& map);
Surface resolveSurface(int id, const StepMap& map);
BoundaryLoop sampleLoop(int boundId, const StepMap& map, int arcSegs);

// tessellator.cpp
TessellatedFace tessellateAdvancedFace(int faceId, const StepMap& map, int arcSegs, Surface* outSurface = nullptr, CylinderHeightRange* outHeightRange = nullptr);
float computeFaceArea(const TessellatedFace& face);
Mesh uploadMesh(const TessellatedFace& tessellatedFace);
void retessCylinderFace(CadModel& model, int cylId, double newHeightMin, double newHeightMax);
std::vector<CylinderHealEntry> buildCylinderHealCache(const CadModel& model, int planeFaceId);
void applyCylinderHealCache(CadModel& model, std::vector<CylinderHealEntry>& cache, Vec3 planeNormal, Vector3 delta);