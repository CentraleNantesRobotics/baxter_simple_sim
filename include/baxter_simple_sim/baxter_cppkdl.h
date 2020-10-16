#include <ros/ros.h>
#include <std_msgs/String.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <sensor_msgs/JointState.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <baxter_core_msgs/SolvePositionIK.h>

#ifndef CPPKDL_H
#define CPPKDL_H

using namespace KDL;

namespace CPPKDL {
class baxter_kinematics
{
  public:
    baxter_kinematics(ros::NodeHandle &node, std::string limb);

  private:

    static ros::Subscriber js_sub;
    static sensor_msgs::JointState joint_states;
    static void jointStateCallback(const sensor_msgs::JointState &mMsg);



    bool processIK(baxter_core_msgs::SolvePositionIKRequest &req, baxter_core_msgs::SolvePositionIKResponse &res);

    ros::ServiceServer ik_service;

    std::array<std::string, 7> names;
    int num_jnts;
    std::string limb;
    KDL::Tree kdl_tree;
    KDL::Chain arm_chain;
    bool inverse_kinematics_function(std::array<double, 7> &result, double position[3], double orientation[4], double seed[7]);

};
}

#endif
