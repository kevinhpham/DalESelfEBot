#include "ik_solver_kdl.hpp"
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <iostream>

IKSolverKDL::IKSolverKDL(const std::string& urdf_path) : initialized_(false) {
    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromFile(urdf_path, kdl_tree)) {
        RCLCPP_ERROR(rclcpp::get_logger("IKSolverKDL"), "Failed to parse URDF file: %s", urdf_path.c_str());
        return;
    }

    std::string base_link = "base_link";  // Ensure this exists in URDF
    std::string end_effector = "tool0"; // Update based on your URDF

    if (!kdl_tree.getChain(base_link, end_effector, kdl_chain_)) {
        RCLCPP_ERROR(rclcpp::get_logger("IKSolverKDL"), "Failed to extract KDL chain from URDF.");
        return;
    }

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);
    ik_solver_vel_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
    ik_solver_pos_ = std::make_unique<KDL::ChainIkSolverPos_NR>(kdl_chain_, *fk_solver_, *ik_solver_vel_, 100, 1e-6);

    // **Initialize last valid joint positions to zeros**
    last_valid_joint_positions_.resize(kdl_chain_.getNrOfJoints(), 0.0);

    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("IKSolverKDL"), "KDL IK Solver initialized successfully.");
}

bool IKSolverKDL::isInitialized() const {
    return initialized_;
}

std::vector<double> IKSolverKDL::computeInverseKinematics(const std::vector<double>& cartesian_position) {
    if (!initialized_) {
        RCLCPP_ERROR(rclcpp::get_logger("IKSolverKDL"), "KDL IK Solver not initialized!");
        return {};
    }

    // Convert input Cartesian position to KDL Frame
    KDL::Frame target_pose(KDL::Vector(cartesian_position[0], cartesian_position[1], cartesian_position[2]));

    // Create zeroed joint array for IK solver
    KDL::JntArray initial_joint_positions(kdl_chain_.getNrOfJoints());
    KDL::JntArray joint_positions(kdl_chain_.getNrOfJoints());

    int result = ik_solver_pos_->CartToJnt(initial_joint_positions, target_pose, joint_positions);

    if (result < 0) {
        RCLCPP_WARN(rclcpp::get_logger("IKSolverKDL"), "Singularity or unreachable point detected.");

        // **Check if it's a singularity or an unreachable point**
        if (isSingularity(cartesian_position)) {
            RCLCPP_WARN(rclcpp::get_logger("IKSolverKDL"), "Singularity detected, returning closest valid solution.");
            return last_valid_joint_positions_;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("IKSolverKDL"), "Unreachable point detected, returning empty solution.");
            return {};  // **Return empty vector for unreachable points**
        }
    }

    std::vector<double> joint_values;
    for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
        // Clamp joint values within [-pi, pi]
        double angle = joint_positions(i);
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;

        joint_values.push_back(angle);
    }

    // **Update last valid joint positions**
    last_valid_joint_positions_ = joint_values;

    return joint_values;
}

bool IKSolverKDL::isSingularity(const std::vector<double>& cartesian_position) {
    // For now, assume any pose near the origin is a singularity
    double threshold = 1e-3;  // Define singularity threshold (adjust as needed)
    
    if (std::abs(cartesian_position[0]) < threshold && 
        std::abs(cartesian_position[1]) < threshold) {
        return true;  // Likely a singularity
    }
    
    return false;  // Not a singularity
}

