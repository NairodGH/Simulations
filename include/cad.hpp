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

struct Surface {
    SurfaceKind kind = SurfaceKind::Unknown;
    AxisPlacement axis;
    double majorRadius = 0; // cylinder: radius/torus: center->tube->center
    double minorRadius = 0; // torus: tube radius
};

// one sampled boundary loop (closed ordered polyline)
struct BoundaryLoop {
    std::vector<Vec3> points;
    bool isOuter = false; // FACE_OUTER_BOUND vs FACE_BOUND (hole)
    bool hasFullCircle = false; // any edge had vsId==veId -> full-revolution arc
};

// one tessellated face ready for GPU upload
struct TessellatedFace {
    SurfaceKind kind = SurfaceKind::Unknown;
    std::vector<float> vertices; // interleaved XYZ
    std::vector<float> normals; // interleaved XYZ, exact analytical normals
    std::vector<int> indices; // triangle indices
};

struct StepEntity {
    std::string type, params;
};
using StepMap = std::unordered_map<int, StepEntity>;

// final GPU model
struct CadModel {
    std::vector<Mesh> meshes;
    std::vector<Color> colors; // one per mesh, color-coded by surface type
    BoundingBox bbox;
    ~CadModel();
};