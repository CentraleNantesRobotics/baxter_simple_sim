#ifndef BAXTER_SIM_ARM_IK_H
#define BAXTER_SIM_ARM_IK_H

#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <rclcpp/node.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <urdf/model.h>

namespace baxter_simple_sim
{

using namespace baxter_core_msgs;

enum class Motion{CMD, MIRROR, PUPPET};

inline size_t findIdx(const std::vector<std::string> &names, const std::string &name)
{
  auto where{std::find(names.begin(), names.end(), name)};
  return std::distance(names.begin(), where);
}

class BaxterArmIO : public rclcpp::Node
{
public:
  BaxterArmIO(rclcpp::Node* node, const urdf::Model &model, std::string limb, Motion motion);

  inline static constexpr auto samplingTime()
  {
    return std::chrono::milliseconds{static_cast<int>(1000*dt)};
  }

  inline auto jointNames() const
  {
    return state.name;
  }

  inline void update(std::vector<double>::iterator in_full_state, double t = 0)
  {
    std::scoped_lock lock(state_mtx, cmd_mtx);
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
  Motion motion;
  std::string limb;
  std::mutex state_mtx, cmd_mtx;

  sensor_msgs::msg::JointState state;
  std::vector<double> lower, upper, vel_max;
  msg::JointCommand last_cmd;
  rclcpp::Subscription<msg::JointCommand>::SharedPtr cmd_sub;

  void processIK(baxter_core_msgs::srv::SolvePositionIK::Request::SharedPtr req,
                 baxter_core_msgs::srv::SolvePositionIK::Response::SharedPtr res);

  rclcpp::Service<baxter_core_msgs::srv::SolvePositionIK>::SharedPtr ik_service;

  KDL::Chain arm_chain;
  std::vector<double> inverseKinematics(KDL::Vector pos, KDL::Rotation rot, const std::vector<double> &seed);

  void updateCmd();
  void updateMirror(double t);
  void updatePuppet(double t);
};

}

#endif
