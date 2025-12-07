#include "mujoco_ros2_control/mujoco_system.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mujoco_ros2_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

hardware_interface::CallbackReturn MujocoSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (info.joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "No joints specified in hardware info");
    return CallbackReturn::FAILURE;
  }

  size_t nj = info.joints.size();
  hw_positions_.assign(nj, 0.0);
  hw_velocities_.assign(nj, 0.0);
  hw_efforts_.assign(nj, 0.0);
  hw_commands_.assign(nj, 0.0);

  std::string model_path = info.hardware_parameters.count("mj_model_path") ?
    info.hardware_parameters.at("mj_model_path") : "robot.mjcf";

  char error[1000] = "";
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
  if (!model_) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "Failed to load MuJoCo model: %s", error);
    return CallbackReturn::FAILURE;
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "Failed to create MuJoCo data");
    mj_deleteModel(model_);
    model_ = nullptr;
    return CallbackReturn::FAILURE;
  }

  mujoco_joint_ids_.clear();
  for (auto & j : info.joints) {
    int jid = mj_name2id(model_, mjOBJ_JOINT, j.name.c_str());
    if (jid < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "Joint %s missing", j.name.c_str());
      return CallbackReturn::FAILURE;
    }
    mujoco_joint_ids_.push_back(jid);
  }

  if (info.hardware_parameters.count("mj_dt"))
    sim_dt_ = std::stod(info.hardware_parameters.at("mj_dt"));

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  // for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
  //   auto & name = get_info().joints[i].name;
  //   out.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
  //   out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  //   out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  // }
  return out;
}

std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  // for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
  //   auto & name = get_info().joints[i].name;
  //   out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]);
  // }
  return out;
}

hardware_interface::return_type MujocoSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
    int jid = mujoco_joint_ids_[i];
    int qp = model_->jnt_qposadr[jid];
    int qv = model_->jnt_dofadr[jid];
    hw_positions_[i] = data_->qpos[qp];
    hw_velocities_[i] = data_->qvel[qv];
    hw_efforts_[i] = data_->qfrc_applied[qv];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (int i = 0; i < model_->nu; ++i) data_->ctrl[i] = 0.0;

  // for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
  //   auto & name = get_info().joints[i].name;
  //   int aid = mj_name2id(model_, mjOBJ_ACTUATOR, name.c_str());
  //   if (aid >= 0 && aid < model_->nu)
  //     data_->ctrl[aid] = hw_commands_[i];
  //   else {
  //     std::string alt = "act_" + name;
  //     int aid2 = mj_name2id(model_, mjOBJ_ACTUATOR, alt.c_str());
  //     if (aid2 >= 0 && aid2 < model_->nu)
  //       data_->ctrl[aid2] = hw_commands_[i];
  //   }
  // }

  mj_step(model_, data_);
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MujocoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (data_) mj_deleteData(data_), data_ = nullptr;
  if (model_) mj_deleteModel(model_), model_ = nullptr;
  return CallbackReturn::SUCCESS;
}

MujocoSystem::~MujocoSystem()
{
  if (data_) mj_deleteData(data_);
  if (model_) mj_deleteModel(model_);
}

} 
PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem, hardware_interface::SystemInterface)
