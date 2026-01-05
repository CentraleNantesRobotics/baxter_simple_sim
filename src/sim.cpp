#include <baxter_simple_sim/arm_io.h>
#include <baxter_simple_sim/sim_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace baxter_simple_sim;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;

  auto sim{std::make_shared<BaxterSim>()};
  sim->addNodesTo(exec);
  exec.spin();
}
