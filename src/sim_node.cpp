#include <baxter_simple_sim/sim_node.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf_parser/urdf_parser.h>

using namespace baxter_simple_sim;

using namespace std::chrono_literals;

BaxterSim::BaxterSim() : Node("simulator")
{
  // get baxter description
  const auto model{initRSP()};

  const auto motion_arg{declare_parameter("motion", "none")};
  const auto zero_joints{declare_parameter("zero_joints", false)};

  for(const auto &[name, joint]: model->joints_)
  {
    if(joint->type != urdf::Joint::FIXED)
    {
      if(name.find("left_") != 0 && name.find("right_") != 0)
      {
        state.name.push_back(name);
        state.position.push_back(0);
        state.velocity.push_back(0);
      }
    }
  }
  torso_joints = state.name.size();

  // init joint groups
  auto motion{Motion::CMD};
  if(motion_arg == "mirror") motion = Motion::MIRROR;
  else if(motion_arg == "puppet") motion = Motion::PUPPET;

  if(zero_joints)
    BaxterArmIO::zeroJoints();

  left = std::make_shared<BaxterArmIO>(this, *model, "left", motion);
  right = std::make_shared<BaxterArmIO>(this, *model, "right", motion);

  state.position.resize(torso_joints+14, 0);
  state.velocity.resize(torso_joints+14, 0);
  for(const auto &name: left->jointNames())
    state.name.push_back(name);
  for(const auto &name: right->jointNames())
    state.name.push_back(name);

  js_pub = create_publisher<sensor_msgs::msg::JointState>("/robot/joint_states", 10);
  pub_timer = create_wall_timer(50ms, [&]()
  {
    state.header.set__stamp(get_clock()->now());
    js_pub->publish(state);
  });

  sim_timer = create_wall_timer(BaxterArmIO::samplingTime(), [&](){updateSim();});
}

std::unique_ptr<urdf::Model> BaxterSim::initRSP()
{
  const auto baxter_folder = ament_index_cpp::get_package_share_directory("baxter_description");
  const auto description_file{baxter_folder + "/urdf/baxter.urdf.xacro"};

  // yes we still have to process a command output in 2022
  FILE * stream;
  const int max_buffer = 256;
  std::string cmd{"xacro "};
  cmd += description_file;
  stream = popen(cmd.c_str(), "r");
  std::string xml;

  if (stream)
  {
    while (!feof(stream))
    {
      char buffer[max_buffer];
      if (fgets(buffer, max_buffer, stream) != NULL) xml.append(buffer);
    }
    pclose(stream);
  }

  // override rsp's options
  auto rsp_arg{rclcpp::NodeOptions()
        .arguments({"--ros-args", "-r", "__ns:=/robot", "-p", "robot_description:='" + xml + "'"})};
  rsp = std::make_shared<robot_state_publisher::RobotStatePublisher>(rsp_arg);

  auto model{std::make_unique<urdf::Model>()};
  model->initString(xml);
  return model;
}

void BaxterSim::updateSim()
{
  left->update(state.position.begin()+torso_joints, get_clock()->now().seconds());
  right->update(state.position.begin()+torso_joints+7, get_clock()->now().seconds());
}
