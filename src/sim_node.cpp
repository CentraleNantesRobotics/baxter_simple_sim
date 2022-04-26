#include <baxter_simple_sim/sim_node.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <urdf_parser/urdf_parser.h>
#include <fstream>

using namespace baxter_simple_sim;

using namespace std::chrono_literals;

BaxterSim::BaxterSim(Motion motion) : Node("simulator")
{
  // get baxter description
  const auto model{initRSP()};

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
  std::string xml_string;
  const auto description_file{ament_index_cpp::get_package_share_directory("baxter_description")
        + "/urdf/baxter.urdf"};
  std::fstream xml_file(description_file.c_str(), std::fstream::in);
  while ( xml_file.good() )
  {
    std::string line;
    std::getline( xml_file, line);
    xml_string += (line + "\n");
  }
  xml_file.close();

  // init rsp with special options
  const std::string rsp_param_file{"/tmp/baxter_description.yaml"};
  std::ofstream description_stream;
  description_stream.open(rsp_param_file.c_str());
  description_stream << "/robot/robot_state_publisher:\n"
                     << "  ros__parameters:\n"
                     << "    robot_description: '"
                     << xml_string << "'\n";
  description_stream.close();

  // override rsp's options
  auto rsp_arg{rclcpp::NodeOptions()
        .arguments({"--ros-args", "-r", "__ns:=/robot", "--params-file", rsp_param_file})
              };
  rsp = std::make_shared<robot_state_publisher::RobotStatePublisher>(rsp_arg);

  auto model{std::make_unique<urdf::Model>()};
  model->initString(xml_string);
  return model;
}

void BaxterSim::updateSim()
{
  left->update(state.position.begin()+torso_joints);
  right->update(state.position.begin()+torso_joints+7, get_clock()->now().seconds());
}
