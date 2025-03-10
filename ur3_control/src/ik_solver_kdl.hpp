#ifndef IK_SOLVER_KDL_H
#define IK_SOLVER_KDL_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class IKSolverKDL {
public:
    explicit IKSolverKDL(const std::string& urdf_path);
    bool isInitialized() const;
    std::vector<double> computeInverseKinematics(const std::vector<double>& cartesian_position);
    bool isSingularity(const std::vector<double>& cartesian_position);

private:
    bool initialized_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_;
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_pos_;

    std::vector<double> last_valid_joint_positions_;  // Stores last valid IK solution
};

#endif // IK_SOLVER_KDL_H
