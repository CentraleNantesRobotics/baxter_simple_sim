#include <Eigen/QR>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>

#include <baxter_simple_sim/arm_io.h>
#include <baxter_simple_sim/sim_node.h>

using namespace baxter_simple_sim;
using namespace std;
using namespace KDL;

using IKReq = baxter_core_msgs::srv::SolvePositionIK::Request::SharedPtr;
using IKRes = baxter_core_msgs::srv::SolvePositionIK::Response::SharedPtr;
using JacReq = baxter_simple_sim::srv::Jacobian::Request::SharedPtr;
using JacRes = baxter_simple_sim::srv::Jacobian::Response::SharedPtr;

void updateSeed(const vector<std::string> &names,
                const vector<double> &req,
                const vector<string> &req_names,
                vector<double> &seed)
{
  if(req_names.size() != req.size() || seed.size() != 7)
    return;

  for(size_t i = 0; i < req_names.size(); ++i)
  {
    if(const auto idx{findIdx(names, req_names[i])};idx != 7)
      seed[idx] = req[i];
  }
}

Solvers::Solvers(const KDL::Chain &chain) : fwd{chain}, ik_v{chain}, ik_p{chain, fwd, ik_v}, jac{chain}
{}

BaxterArmIO::BaxterArmIO(rclcpp::Node* node, const urdf::Model &model, std::string limb, Motion motion)
  : Node(limb + "_solvers"), motion{motion}, limb{limb}
{
  // ensure name ordering for IK
  state.name = {"s0", "s1", "e0", "e1", "w0", "w1", "w2"};
  state.position = {0, -0.7, 0, 1.59, 0, -0.7, 0};
  state.velocity.resize(7,0);

  if(motion == Motion::PUPPET)
  {
    // singularity-free configuration for IK
    if(limb == "left")
    {
      state.position = {-0.10266471341792811, -0.05772519794064627, -0.36909985265803286, 0.7955637684523007, -1.1302109919350245, 1.0871143114989947, 0};
    }
    else
      state.position = {0.05183482296524736, -0.8682037556901855, 0.9476424476835772, 1.685625905154571, 0.7588301888902473, 0.999285765226626, -0.3354836773966444};
  }

  if(limb == "left") this->motion = Motion::CMD;

  for(auto &name: state.name)
  {
    name = limb + "_" + name;
    const auto joint{model.getJoint(name)};
    lower.push_back(joint->limits->lower);
    upper.push_back(joint->limits->upper);
    vel_max.push_back(joint->limits->velocity);
  }

  // init topic
  if(this->motion == Motion::CMD)
  {
    const auto topic{"/robot/limb/" + limb + "/joint_command"};
    cmd_sub = node->create_subscription<msg::JointCommand>
        (topic, 10, [this](msg::JointCommand::SharedPtr msg)
    {std::lock_guard lk(cmd_mtx);last_cmd = *msg;last_cmd_t = this->now().seconds();});
  }

  // init chain from kdl tree
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel(model, tree);
  tree.getChain("base", limb + "_gripper", arm_chain);
  solvers = std::make_unique<Solvers>(arm_chain);

  // init ik service
  ik_service = create_service<baxter_core_msgs::srv::SolvePositionIK>("/ExternalTools/" + limb + "/PositionKinematicsNode/IKService",
                                                          [&](IKReq req, IKRes res){processIK(req,res);});

  // init Jacobian service
  jacobian_service = create_service<srv::Jacobian>("/robot/limb/" + limb + "/jacobian",
                                                         [&](JacReq req, JacRes res){processJacobian(req,res);});
}

void BaxterArmIO::processIK(IKReq req, IKRes res)
{
  // default seed from current position (SEED_CURRENT)
  std::vector<double> seed;
  {
    std::lock_guard lk(state_mtx);
    seed = state.position;
  }

  const auto dim{req->pose_stamp.size()};
  const auto seeds{req->seed_angles.size()};

  if(req->seed_mode == req->SEED_AUTO)
  {
    req->seed_mode = dim == seeds ? req->SEED_USER : req->SEED_NS_MAP;
  }

  res->joints.resize(dim);
  res->is_valid.resize(dim, false);
  res->result_type.resize(dim, res->RESULT_INVALID);
  size_t step{0};
  for(const auto &pose: req->pose_stamp)
  {
    const auto &t{pose.pose.position};
    const auto &q{pose.pose.orientation};

    if(req->seed_mode == req->SEED_USER
       && seeds == dim)
    {
      // seed from user-defined angles
      updateSeed(state.name, req->seed_angles[step].position, req->seed_angles[step].name, seed);
    }
    else if(req->seed_mode == req->SEED_NS_MAP
            && step != 0
            && res->is_valid[step-1])
    {
      // seed from previous solution if valid
      seed = res->joints[step-1].position;
    }

    const auto solution = inverseKinematics({t.x,t.y,t.z}, KDL::Rotation::Quaternion(q.x,q.y,q.z,q.w), seed);

    if(!solution.empty())
    {
      res->joints[step].name = state.name;
      res->joints[step].header.stamp = pose.header.stamp;
      res->joints[step].position = solution;

      res->is_valid[step] = true;
      res->result_type[step] = 1;
    }
    step++;
  }
}


std::vector<double> BaxterArmIO::inverseKinematics(KDL::Vector pos, KDL::Rotation rot, const std::vector<double> &seed)
{  
  // Populate seed
  KDL::JntArray seed_array = JntArray(7);
  for(size_t i = 0; i < 7; ++i)
    seed_array(i) = seed[i];

  //Make IK Call
  KDL::Frame goal_pose(rot, pos);
  auto result_angles = JntArray(7);  
  const auto ik_status = solvers->ik_p.CartToJnt(seed_array, goal_pose, result_angles);

  std::vector<double> solution;
  if(ik_status == solvers->ik_p.E_NOERROR || ik_status == solvers->ik_p.E_DEGRADED)
  {
    solution.resize(7);
    for(size_t i = 0; i < 7; ++i)
      solution[i] = result_angles(i);
  }
  return solution;
}

void BaxterArmIO::updateCmd()
{
  const auto vel_mode{last_cmd.mode == last_cmd.VELOCITY_MODE};
  for(size_t i = 0; i < last_cmd.names.size(); ++i)
  {
    const auto idx{findIdx(state.name, last_cmd.names[i])};
    if(idx == state.name.size())
      continue;

    auto &pos{state.position[idx]};
    auto &vel{state.velocity[idx]};
    const auto cmd{last_cmd.command[i]};

    // what velocity this joint should get
    vel = std::clamp(vel_mode ? cmd : (cmd-pos)/dt,
                     -vel_max[idx],
                     vel_max[idx]);

    // ensure joint limits
    const auto fut_pos{pos + vel*dt};
    if(fut_pos > upper[idx])
    {
      vel = (upper[idx]-pos)/dt;
      pos = upper[idx];
    }
    else if(fut_pos < lower[idx])
    {
      vel = (lower[idx]-pos)/dt;
      pos = lower[idx];
    }
    else
    {
      pos = fut_pos;
    }
  }
}

void BaxterArmIO::processJacobian(Jacobian::Request::SharedPtr req, Jacobian::Response::SharedPtr res)
{
  KDL::JntArray q(7);
  if(req->position.size() == 7)
    std::copy(req->position.begin(), req->position.end(), q.data.data());
  else
    std::copy(state.position.begin(), state.position.end(), q.data.data());

  // get base Jacobian fJe
  KDL::Jacobian J(7);
  solvers->jac.JntToJac(q, J);

  if(req->ee_frame)
  {
    // we want eJe, rotate
    KDL::Frame fMe;
    solvers->fwd.JntToCart(q, fMe);
    J.changeBase(fMe.M.Inverse());
  }

  const auto writeMatrix = [res](const Eigen::Matrix<double,Eigen::Dynamic, Eigen::Dynamic> &M)
  {
    auto elem{res->jacobian.begin()};
    for(int row = 0; row < M.rows(); ++row)
    {
      for(int col = 0; col < M.cols(); ++col)
        *elem++ = M(row,col);
    }
  };
  if(req->inverse)
    writeMatrix(J.data.completeOrthogonalDecomposition().pseudoInverse());
  else
    writeMatrix(J.data);

}

void BaxterArmIO::updateMirror(double t)
{
  for(size_t i = 0; i < 7; ++i)
  {
    const auto rng{upper[i]-lower[i]};
    const auto mid{(upper[i]+lower[i])/2.};
    state.position[i] = mid + 0.02*(i+1)*rng*cos(0.1*(i+2)*t);
  }
}

void BaxterArmIO::updatePuppet(double t)
{
  static KDL::Frame M0(KDL::Rotation::RPY(-1.5,M_PI,0),
                       KDL::Vector{0.7, -0.027, 0.457});

  KDL::Frame Md(KDL::Rotation::RPY(.1*cos(t), .1*sin(t/2), .05*cos(t+4)),
                KDL::Vector(0.05*cos(t/2.),0.05*sin(t/2),0.2*cos(t/2)));
  Md = M0*Md;

  const auto sol{inverseKinematics(Md.p, Md.M, state.position)};

  if(sol.size() == 7)
  {
    for(size_t i = 0; i < 7; ++i)
      state.velocity[i] = (sol[i]-state.position[i])/dt;
    state.position = sol;
  }
}
