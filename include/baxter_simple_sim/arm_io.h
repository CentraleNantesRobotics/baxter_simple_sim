#ifndef BAXTER_SIM_ARM_IK_H
#define BAXTER_SIM_ARM_IK_H

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/node.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_simple_sim/srv/jacobian.hpp>
#include <urdf/model.h>


namespace baxter_simple_sim
{

using namespace baxter_core_msgs;
using srv::Jacobian;

enum class Motion{CMD, MIRROR, PUPPET};

inline size_t findIdx(const std::vector<std::string> &names, const std::string &name)
{
  auto where{std::find(names.begin(), names.end(), name)};
  return std::distance(names.begin(), where);
}

struct Solvers
{
  explicit Solvers(const KDL::Chain &chain);

  KDL::ChainFkSolverPos_recursive fwd;
  KDL::ChainIkSolverVel_pinv ik_v;
  KDL::ChainIkSolverPos_NR ik_p;
  KDL::ChainJntToJacSolver jac;
};

class BaxterArmIO : public rclcpp::Node
{
public:
  BaxterArmIO(rclcpp::Node* node, const urdf::Model &model, std::string limb, Motion motion);

  inline static constexpr auto samplingTime()
  {
    return std::chrono::milliseconds{static_cast<int>(1000*dt)};
  }

  inline static auto zeroJoints()
  {
    zero_joints = true;
  }

  inline auto jointNames() const
  {
    return state.name;
  }

  inline void update(std::vector<double>::iterator in_full_state, double t = 0)
  {
    std::scoped_lock lock(state_mtx, cmd_mtx);

    if(zero_joints && motion == Motion::CMD && (t - last_cmd_t) > 1.0)
    {
     // go back to home
     last_cmd.names = state.name;
     last_cmd.mode = msg::JointCommand::POSITION_MODE;
     last_cmd.command = {0, -0.7, 0, 1.59, 0, -0.7, 0};
    }

    switch (motion)
    {
    case Motion::CMD:
      updateCmd();
      break;
    case Motion::MIRROR:
      updateMirror(t);
      break;
    case Motion::PUPPET:
      updatePuppet(t);
      break;
    }
    std::copy(state.position.begin(), state.position.end(), in_full_state);
  }

private:

  static constexpr double dt{0.02};
  static inline bool zero_joints{false};
  Motion motion;
  std::string limb;
  std::mutex state_mtx, cmd_mtx;

  sensor_msgs::msg::JointState state;
  std::vector<double> lower, upper, vel_max;
  msg::JointCommand last_cmd;
  double last_cmd_t{0};
  rclcpp::Subscription<msg::JointCommand>::SharedPtr cmd_sub;

  KDL::Chain arm_chain;  
  std::unique_ptr<Solvers> solvers;

  rclcpp::Service<baxter_core_msgs::srv::SolvePositionIK>::SharedPtr ik_service;
  void processIK(baxter_core_msgs::srv::SolvePositionIK::Request::SharedPtr req,
                 baxter_core_msgs::srv::SolvePositionIK::Response::SharedPtr res);
  std::vector<double> inverseKinematics(KDL::Vector pos, KDL::Rotation rot, const std::vector<double> &seed);

  rclcpp::Service<Jacobian>::SharedPtr jacobian_service;
  void processJacobian(Jacobian::Request::SharedPtr req, Jacobian::Response::SharedPtr res);

  void updateCmd();
  void updateMirror(double t);
  void updatePuppet(double t);
};

}

#endif
