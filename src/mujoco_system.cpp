#include <string>
#include <vector>
#include <memory>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav_mujoco_ros2_control
{

class MujocoSystem : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    // std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    // std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    virtual ~MujocoSystem();

private:
    mjModel * model_ = nullptr;
    mjData * data_ = nullptr;

    // std::vector<int> mujoco_joint_ids_;

    // std::vector<double> hw_positions_;
    // std::vector<double> hw_velocities_;
    // std::vector<double> hw_efforts_;
    // std::vector<double> hw_commands_;

    // double sim_dt_ = 0.001;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

hardware_interface::CallbackReturn MujocoSystem::on_init(const hardware_interface::HardwareComponentInterfaceParams &params)
{
    const CallbackReturn bcr = hardware_interface::SystemInterface::on_init(params);

    if (bcr != CallbackReturn::SUCCESS)
    {
        return bcr;
    }

    if (params.hardware_info.joints.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "No joints specified in hardware info");
    return CallbackReturn::FAILURE;
  }

  // parse_command_interface_descriptions(params.hardware_info.joints, joint_command_interfaces_);

  // size_t nj = params.joints.size();
  // hw_positions_.assign(nj, 0.0);
  // hw_velocities_.assign(nj, 0.0);
  // hw_efforts_.assign(nj, 0.0);
  // hw_commands_.assign(nj, 0.0);

  std::string model_path = params.hardware_info.hardware_parameters.count("mj_model_path") ?
                               params.hardware_info.hardware_parameters.at("mj_model_path") : "robot.mjcf";

  char error[1000] = "";
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
  // model_ = mj_compile(mj_parseXMLString(info.original_xml.c_str(), nullptr, error, 1000), nullptr);
  RCLCPP_WARN(rclcpp::get_logger("MujocoSystem"), "Failed to load MuJoCo model: %s", error);
  if (!model_) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "Failed to load MuJoCo model: %s", error);
    return CallbackReturn::FAILURE;
  }

  // Log all joints
  for (int i = 0; i < model_->njnt; ++i) {
    RCLCPP_INFO(rclcpp::get_logger("MujocoSystem"), "Joint %d: %s", i, mj_id2name(model_, mjOBJ_JOINT, i));
  }

  // Log all actuators
  RCLCPP_INFO(rclcpp::get_logger("MujocoSystem"), "Actuators: %d", model_->nu);
  for (int i = 0; i < model_->nu; ++i) {
    RCLCPP_INFO(rclcpp::get_logger("MujocoSystem"), "Actuator %d: %s", i, mj_id2name(model_, mjOBJ_ACTUATOR, i));
  }

  data_ = mj_makeData(model_);
  if (!data_) {
    RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "Failed to create MuJoCo data");
    mj_deleteModel(model_);
    model_ = nullptr;
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("MujocoSystem"), "...");

  // mujoco_joint_ids_.clear();
  // for (auto & j : info.joints) {
  //   int jid = mj_name2id(model_, mjOBJ_ACTUATOR, j.name.c_str());
  //   if (jid < 0) {
  //     RCLCPP_ERROR(rclcpp::get_logger("MujocoSystem"), "Joint %s missing", j.name.c_str());
  //     return CallbackReturn::FAILURE;
  //   }
  //   mujoco_joint_ids_.push_back(jid);
  // }

  // if (params.hardware_info.hardware_parameters.count("mj_dt"))
  //     sim_dt_ = std::stod(params.hardware_info.hardware_parameters.at("mj_dt"));

  return CallbackReturn::SUCCESS;
}

// std::vector<hardware_interface::StateInterface> MujocoSystem::export_state_interfaces()
// {
//   std::vector<hardware_interface::StateInterface> out;
//   // for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
//   //   auto & name = get_info().joints[i].name;
//   //   out.emplace_back(name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
//   //   out.emplace_back(name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
//   //   out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
//   // }

//   out.emplace_back("base_imu", "orientation.x");
//   out.emplace_back("base_imu", "orientation.y");
//   out.emplace_back("base_imu", "orientation.z");
//   out.emplace_back("base_imu", "orientation.w");
//   out.emplace_back("base_imu", "angular_velocity.x");
//   out.emplace_back("base_imu", "angular_velocity.y");
//   out.emplace_back("base_imu", "angular_velocity.z");
//   out.emplace_back("base_imu", "linear_acceleration.x");
//   out.emplace_back("base_imu", "linear_acceleration.y");
//   out.emplace_back("base_imu", "linear_acceleration.z");
//   return out;
// }

// std::vector<hardware_interface::CommandInterface> MujocoSystem::export_command_interfaces()
// {
//   std::vector<hardware_interface::CommandInterface> out;
//   // for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
//   //   auto & name = get_info().joints[i].name;
//   //   out.emplace_back(name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]);
//   // }
//   return out;
// }

hardware_interface::return_type MujocoSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    // mjtSensor_
    // mjSENS_GYRO
  // data_->sensordata[mj_name2id(model_, mjSENS_GYRO, "body_gyro")];

    // TODO: make "base_imu" configurable?

  const int gyro_id = mj_name2id(model_, mjOBJ_SENSOR, "body_gyro");
  if (gyro_id >= 0) {
    int sensor_adr = model_->sensor_adr[gyro_id];
    // Angular velocities from gyro sensor
    set_state<double>("base_imu/angular_velocity.x", data_->sensordata[sensor_adr]);
    set_state<double>("base_imu/angular_velocity.y", data_->sensordata[sensor_adr + 1]);
    set_state<double>("base_imu/angular_velocity.z", data_->sensordata[sensor_adr + 2]);
  }

  const int accel_id = mj_name2id(model_, mjOBJ_SENSOR, "body_linacc");
  if (accel_id >= 0) {
    int sensor_adr = model_->sensor_adr[accel_id];
    // Linear accelerations from accelerometer sensor
    set_state<double>("base_imu/linear_acceleration.x", data_->sensordata[sensor_adr]);
    set_state<double>("base_imu/linear_acceleration.y", data_->sensordata[sensor_adr + 1]);
    set_state<double>("base_imu/linear_acceleration.z", data_->sensordata[sensor_adr + 2]);
  }

  const int quat_id = mj_name2id(model_, mjOBJ_SENSOR, "body_quat");
  if (quat_id >= 0) {
    int sensor_adr = model_->sensor_adr[quat_id];
    // Orientation from quaternion sensor
    set_state<double>("base_imu/orientation.x", data_->sensordata[sensor_adr]);
    set_state<double>("base_imu/orientation.y", data_->sensordata[sensor_adr + 1]);
    set_state<double>("base_imu/orientation.z", data_->sensordata[sensor_adr + 2]);
    set_state<double>("base_imu/orientation.w", data_->sensordata[sensor_adr + 3]);
  }

  // set_state<double>(sensor_name + "/orientation.x", msg_imu.orientation.x);
  // set_state<double>(sensor_name + "/orientation.y", msg_imu.orientation.y);
  // set_state<double>(sensor_name + "/orientation.z", msg_imu.orientation.z);
  // set_state<double>(sensor_name + "/orientation.w", msg_imu.orientation.w);

  // set_state<double>(sensor_name + "/angular_velocity.x", msg_imu.angular_velocity.x);
  // set_state<double>(sensor_name + "/angular_velocity.y", msg_imu.angular_velocity.y);
  // set_state<double>(sensor_name + "/angular_velocity.z", msg_imu.angular_velocity.z);

  // set_state<double>(sensor_name + "/linear_acceleration.x", msg_imu.linear_acceleration.x);
  // set_state<double>(sensor_name + "/linear_acceleration.y", msg_imu.linear_acceleration.y);
  // set_state<double>(sensor_name + "/linear_acceleration.z", msg_imu.linear_acceleration.z);


  // for (size_t i = 0; i < mujoco_joint_ids_.size(); ++i) {
  //   int jid = mujoco_joint_ids_[i];
  //   int qp = model_->jnt_qposadr[jid];
  //   int qv = model_->jnt_dofadr[jid];
  //   hw_positions_[i] = data_->qpos[qp];
  //   hw_velocities_[i] = data_->qvel[qv];
  //   hw_efforts_[i] = data_->qfrc_applied[qv];
  // }
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
PLUGINLIB_EXPORT_CLASS(uav_mujoco_ros2_control::MujocoSystem, hardware_interface::SystemInterface)
