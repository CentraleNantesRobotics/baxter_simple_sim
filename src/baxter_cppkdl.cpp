#include <baxter_simple_sim/baxter_cppkdl.h>

namespace CPPKDL {

using namespace std;
using namespace KDL;

ros::Subscriber baxter_kinematics::js_sub;
sensor_msgs::JointState baxter_kinematics::joint_states;

baxter_kinematics::baxter_kinematics(ros::NodeHandle&node, std::string _limb)
{
  limb = _limb;

  //Create a subscriber for joint state if not already defined
  if(!joint_states.name.size())
  {
    joint_states.name.resize(1);
    js_sub = node.subscribe("/robot/joint_states", 100, &baxter_kinematics::jointStateCallback);
  }

  ros::Rate wait(1);
  while(!node.hasParam("/robot_description"))
  {
    std::cout << "Waiting for robot description..." << std::endl;
    wait.sleep();
  }
  //Read the URDF from param sever
  const auto urdf = node.param<std::string>("/robot_description", "");
  if (!kdl_parser::treeFromString(urdf, kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree");
  }
  //Get chain from kdl tree
  std::string base_link = "base";
  std::string tip_link = limb + "_gripper";
  kdl_tree.getChain(base_link, tip_link, arm_chain);
  //Get number of Joints
  num_jnts = arm_chain.getNrOfJoints();

  names = {"s0", "s1", "e0", "e1", "w0", "w1", "w2"};
  for(auto &name: names)
    name = limb + "_" + name;
  
  //Wait for first message from joint_state subscriber arrives
  while(ros::ok() && joint_states.name.size() == 1)
  {
    wait.sleep();
    ros::spinOnce();
  }
  ik_service = node.advertiseService("/ExternalTools/" + limb + "/PositionKinematicsNode/IKService", &baxter_kinematics::processIK, this);
}

bool baxter_kinematics::processIK(baxter_core_msgs::SolvePositionIKRequest &req, baxter_core_msgs::SolvePositionIKResponse &res)
{
  if(req.seed_mode != req.SEED_AUTO && req.seed_mode != req.SEED_CURRENT)
    return false;

  // build response and parse current joint state angles
  sensor_msgs::JointState joints;
  double q[7];
  size_t jIdx = 0;
  for(const auto &name: names)
  {
    joints.name.push_back(name);
    // find corresponding angle
    auto idx = std::distance(joint_states.name.begin(),
                             std::find(joint_states.name.begin(), joint_states.name.end(),
                                       name));
    q[jIdx] = joint_states.position[idx];
    jIdx++;
  }
  joints.position.resize(7, 0);

  double position[3];
  double orientation[4];
  double seed[7];


  std::array<double, 7> result;

  int step = 0;
  for(const auto &pose: req.pose_stamp)
  {
    position[0] = pose.pose.position.x;
    position[1] = pose.pose.position.y;
    position[2] = pose.pose.position.z;
    orientation[0] = pose.pose.orientation.x;
    orientation[1] = pose.pose.orientation.y;
    orientation[2] = pose.pose.orientation.z;
    orientation[3] = pose.pose.orientation.w;

    if(req.seed_mode == req.SEED_USER)
      std::copy(req.seed_angles[step].position.begin(), req.seed_angles[step].position.begin()+7, seed);
    else
      std::copy(q, q+7, seed);

    const auto success = inverse_kinematics_function(result, position, orientation, seed);
    std::copy(result.begin(), result.end(), joints.position.begin());
    joints.header.stamp = pose.header.stamp;
    res.joints.push_back(joints);
    res.isValid.push_back(success);
    step++;
  }
  return true;
}



void baxter_kinematics::jointStateCallback(const sensor_msgs::JointState& mMsg)
{
  joint_states = mMsg;
}



bool baxter_kinematics::inverse_kinematics_function(std::array<double, 7> &result, double position[3], double orientation[4] = NULL, double seed[7]=NULL)
{
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(arm_chain);
  ChainIkSolverVel_pinv iksolver_v = 	ChainIkSolverVel_pinv(arm_chain);
  ChainIkSolverPos_NR iksolver_p = ChainIkSolverPos_NR(arm_chain,fksolver,iksolver_v);
  KDL::Vector pos = Vector(position[0],position[1],position[2]);
  KDL::Rotation rot = Rotation();
  if (orientation != NULL){
    rot = rot.Quaternion(orientation[0],orientation[1],orientation[2],orientation[3]);
  }
  //Populate seed wit current angles if not provided
  KDL::JntArray seed_array = JntArray(num_jnts);
  int i;
  for(i=0; i<num_jnts; i++)
  {
    seed_array(i)=seed[i];
  }

  //Make IK Call
  KDL::Frame goal_pose;
  if (orientation != NULL)
  {
    goal_pose = Frame(rot,pos);
  }
  else{
    goal_pose = Frame(pos);
  }
  KDL::JntArray result_angles = JntArray(num_jnts);
  bool ik_status = iksolver_p.CartToJnt(seed_array, goal_pose, result_angles);

  for(i=0; i<7; i++)
  {
    result[i]=result_angles(i);
  }
  return ik_status == 0;
}
}

