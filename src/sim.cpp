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

  auto motion{Motion::CMD};
  for(int i = 0; i < argc; ++i)
  {
    const std::string arg{argv[i]};
    if(arg == "mirror") motion = Motion::MIRROR;
    else if(arg == "puppet") motion = Motion::PUPPET;
  }

  auto sim{std::make_shared<BaxterSim>(motion)};
  sim->addNodesTo(exec);
  exec.spin();
}
