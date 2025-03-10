#include <gtest/gtest.h>
#include "../src/ik_solver_kdl.hpp"
#include <vector>
#include <iostream>

// Test fixture for IKSolverKDL
class IKSolverKDLTest : public ::testing::Test {
protected:
    void SetUp() override {
        urdf_path = "/home/jarred/git/DalESelfEBot/ur3_control/models/ur3e.urdf";
        solver = std::make_unique<IKSolverKDL>(urdf_path);  // FIX: Initialize solver
    }

    std::string urdf_path;
    std::unique_ptr<IKSolverKDL> solver;
};

// Test if the solver initializes correctly
TEST_F(IKSolverKDLTest, Initialization) {
    ASSERT_NE(solver, nullptr) << "Solver instance is nullptr!";
    EXPECT_TRUE(solver->isInitialized());
}

// Test inverse kinematics computation
TEST_F(IKSolverKDLTest, ComputeIK) {
    std::vector<double> cartesian_pose = {0.3, 0.2, 0.5};  // Example Cartesian position
    std::vector<double> joint_angles = solver->computeInverseKinematics(cartesian_pose);

    // Check that a valid joint solution is returned
    ASSERT_FALSE(joint_angles.empty()) << "IK solver returned an empty solution";

    // Verify joint limits (optional: adjust limits based on UR3e specifications)
    for (double angle : joint_angles) {
        EXPECT_GE(angle, -3.14);  // Lower bound
        EXPECT_LE(angle, 3.14);   // Upper bound
    }
}

// Test edge cases: Unreachable point
TEST_F(IKSolverKDLTest, UnreachablePoint) {
    std::vector<double> unreachable_pose = {5.0, 5.0, 5.0};  // Clearly out of reach
    std::vector<double> joint_angles = solver->computeInverseKinematics(unreachable_pose);

    EXPECT_TRUE(joint_angles.empty()) << "IK solver should return an empty solution for an unreachable point";
}

// Test edge cases: Singularities
TEST_F(IKSolverKDLTest, Singularities) {
    std::vector<double> singular_pose = {0.0, 0.0, 0.0};  // A known singularity case for UR3e
    std::vector<double> joint_angles = solver->computeInverseKinematics(singular_pose);

    // Verify that the solver does NOT return an empty solution
    ASSERT_FALSE(joint_angles.empty()) << "Solver should return the last known valid joint positions instead of failing.";

    // Print the returned joint angles for debugging
    std::cout << "Returned joint angles (singularity case): ";
    for (double angle : joint_angles) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
}

// Main function for Google Test
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
