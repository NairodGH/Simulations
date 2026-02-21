#include "main.hpp"

// This is a technical test file I was given as a .cpp, it suffices for my prototype so I built upon it
// all the comments have been removed so people can't just search them up to cheat using my solution

struct Target {
	Eigen::Vector3d position;
	Eigen::Vector3d align;
	double          roll;
};

struct Axis {
	enum class Kind {
		Rotary,
		Linear
	};

	Kind kind;

	Eigen::Vector3d pivot; // position, the point the joint rotates around or slides from (joint's origin)
	Eigen::Vector3d pivotNormal; // the axis direction for rotation (ignored for linear joints)

	Eigen::Vector3d segmentA;
	Eigen::Vector3d segmentB;
};

struct Robot {
	std::array<Axis, 6> axes;

	Eigen::Matrix4d transform;
};

Target forward(const Robot& robot, const Target& tip_target, const std::array<float, 6> &joint_values) {
    // build the cumulative transform by chaining pivot translations
    Eigen::Matrix4d cumulative_transform = robot.transform;
    for (size_t i = 0; i < robot.axes.size(); i++) {
        const auto& axis = robot.axes[i];
        Eigen::Matrix4d local_transform = Eigen::Matrix4d::Identity();
        local_transform.block<3,1>(0,3) = axis.pivot;
        if (axis.kind == Axis::Kind::Rotary) {
            local_transform.block<3,3>(0,0) = Eigen::AngleAxisd(joint_values[i], axis.pivotNormal).toRotationMatrix();
        } else {
            local_transform(0, 3) += joint_values[i];
        }
        cumulative_transform *= local_transform;
    }
    // transform tip_target from end-effector space to robot base space
    Target result = tip_target;
    result.position = (cumulative_transform * tip_target.position.homogeneous()).head<3>();
    result.align = (cumulative_transform.block<3,3>(0,0) * tip_target.align).normalized();
    return result;
}

Eigen::Quaterniond fromAlignRoll(Eigen::Vector3d align, double roll) {
	// rotate Z (convention vector) to point in the align direction then twists around it by roll radians
    align.normalize();
    return Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), align) * Eigen::Quaterniond(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
}

std::pair<Eigen::Vector3d, double> toAlignRoll(const Eigen::Quaterniond& quat) {
	// extract align direction
    Eigen::Vector3d align = quat * Eigen::Vector3d::UnitZ();
    align.normalize();
	// and isolate roll angle, reverse engineering rotations isn't as easy..
    Eigen::Quaterniond align_quat = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), align);
    Eigen::Quaterniond roll_quat = align_quat.inverse() * quat;
    Eigen::AngleAxisd roll_angle(roll_quat);
    double roll = roll_angle.angle();
    if (roll_angle.axis().dot(Eigen::Vector3d::UnitZ()) < 0) {
        roll = -roll;
    }
    return {align, roll};
}