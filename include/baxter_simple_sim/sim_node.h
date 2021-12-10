#ifndef BAXTER_SIM_NODE
#define BAXTER_SIM_NODE

#include <rclcpp/node.hpp>
#include <baxter_simple_sim/arm_io.h>
#include <robot_state_publisher/robot_state_publisher.hpp>

namespace baxter_simple_sim
{

class BaxterSim : public rclcpp::Node
{

public:
  BaxterSim(Motion motion);
  inline void addNodesTo(rclcpp::Executor &exec)
  {
    exec.add_node(shared_from_this());
    exec.add_node(rsp);
    exec.add_node(left);
    exec.add_node(right);
  }

private:

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub;
  robot_state_publisher::RobotStatePublisher::SharedPtr rsp;
  std::shared_ptr<BaxterArmIO> left, right;

  sensor_msgs::msg::JointState state;
  size_t torso_joints;

  Motion right_motion;

  urdf::Model initRSP();

  rclcpp::TimerBase::SharedPtr sim_timer, pub_timer;
  void updateSim();
};










}

#endif
