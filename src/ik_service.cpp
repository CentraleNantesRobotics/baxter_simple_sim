#include <ros/ros.h>
#include <baxter_simple_sim/baxter_cppkdl.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "baxter_ik_server");
  ros::NodeHandle nh;

  CPPKDL::baxter_kinematics left(nh, "left"), right(nh, "right");

  ros::spin();
}
