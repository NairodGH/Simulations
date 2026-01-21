#include <Eigen/Dense>
#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

// This is the original file I was given as a .cpp
// all the comments have been removed so people can't just search them up to cheat using my solution

struct Input
{
    Eigen::MatrixXd V;

    Eigen::MatrixXi F;

    Eigen::Matrix4d mesh_to_world;

    Eigen::VectorXd slicing_plane_normal;

    enum class Kind
    {
        Linear,
        Spiral
    };
    Kind kind;

    double width = 10;
    double height = 2;
    double height_spacing = 1;

    double spiralizing_out_factor = 1;
    double spiral_length = 10;
    double spiral_step = 0.1;
};

struct Output
{
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> orientation;
};

Eigen::Vector3d get_first_point(const Input& input) {
    // turn each vertex from local to world space using homogeneous coords
    Eigen::MatrixXd V_world(input.V.rows(), 3);
    for (int i = 0; i < input.V.rows(); i++) {
        Eigen::Vector4d v_homogenous(input.V(i, 0), input.V(i, 1), input.V(i, 2), 1.0);
        Eigen::Vector4d v_world = input.mesh_to_world * v_homogenous;
        V_world.row(i) = v_world.head<3>();
    }
    
    // find the "lowest" vertex relative to the plane
    Eigen::Vector3d normal = input.slicing_plane_normal.normalized();
    double min_distance = std::numeric_limits<double>::max();
    int min_i = 0;
    for (int i = 0; i < V_world.rows(); i++) {
        Eigen::Vector3d v = V_world.row(i);
        double distance = v.dot(normal);
        if (distance < min_distance) {
            min_distance = distance;
            min_i = i;
        }
    }
    
    // project that lowest vertex onto the plane, it's the starting point (in this case always 0 0 0)
    Eigen::Vector3d lowest_vertex = V_world.row(min_i);
    return lowest_vertex - (lowest_vertex.dot(normal)) * normal;
}

Output ramping(const Input& input) {
    Output output;
    Eigen::Vector3d first_point = get_first_point(input);
    Eigen::Vector3d normal = input.slicing_plane_normal.normalized();
    
    // since I tilted the plane 45° to fit the schemas, drawing on it would have to take it into account everytime...
    // so I instead simply create a new coordinate system that's parallel to the plane, made up of 2 axes perpendicular to themselves and the normal
    // (if I didn't tilt the plane I wouldn't need it but this makes it adaptable to any rotation)
    Eigen::Vector3d plane_x, plane_y;
    if (std::abs(normal.z()) < 0.9) {
        plane_x = Eigen::Vector3d(0, 0, 1).cross(normal).normalized();
    } else {
        plane_x = Eigen::Vector3d(1, 0, 0).cross(normal).normalized();
    }
    plane_y = normal.cross(plane_x).normalized();
    
    if (input.kind == Input::Kind::Linear) {
        // for each stacked line, alternate direction and draw from one end to the other at the current layer height
        for (int layer = 0; layer <= input.height / input.height_spacing; layer++) {
            int sign = (layer % 2 == 0) ? 1 : -1;
            for (int side : {-sign, sign}) {
                output.points.push_back(first_point + (side * 0.5 * input.width) * plane_x + (layer * input.height_spacing) * plane_y);
                output.orientation.push_back(normal);
            }
        }
    } else {
        // for each spiral step, draw the Archimedean spiral by converting polar to cartesian coordinates so we can use the plane axes
        for (int i = 0; i <= input.spiral_length / input.spiral_step; i++) {
            double angle = i * input.spiral_step;
            output.points.push_back(first_point + (input.spiralizing_out_factor * angle) * (std::cos(angle) * plane_x + std::sin(angle) * plane_y));
            output.orientation.push_back(normal);
        }
    }
    // orientation is always the normal since the head should always point perpendicular to the flat plane
    return output;
}