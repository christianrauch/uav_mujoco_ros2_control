#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

namespace uav_mujoco_ros2_control
{

class MuJoCoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // keyboard callback
  void keyboard(GLFWwindow * window, int key, int scancode, int act, int mods)
  {
    // backspace: reset simulation
    if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
    {
      mj_resetData(model_, data_);
      mj_forward(model_, data_);
    }
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
    {
      glfwDestroyWindow(window);
    }
  }

  // mouse button callback
  void mouse_button(GLFWwindow * window, int button, int act, int mods)
  {
    // update button state
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
  }

  // mouse move callback
  void mouse_move(GLFWwindow * window, double xpos, double ypos)
  {
    // no buttons down: nothing to do
    if (!button_left && !button_middle && !button_right)
    {
      return;
    }

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if (button_right)
    {
      action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    }
    else if (button_left)
    {
      action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    }
    else
    {
      action = mjMOUSE_ZOOM;
    }

    // move camera
    mjv_moveCamera(model_, action, dx / height, dy / height, &mjvis.scn, &mjvis.cam);
  }

  // scroll callback
  void scroll(GLFWwindow * window, double xoffset, double yoffset)
  {
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &mjvis.scn, &mjvis.cam);
  }

  void render()
  {
    glfwMakeContextCurrent(window);

    // render scene
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(model_, data_, &mjvis.opt, nullptr, &mjvis.cam, mjCAT_ALL, &mjvis.scn);
    mjr_render(viewport, &mjvis.scn, &mjvis.con);

    glfwSwapBuffers(window);
    glfwPollEvents();
    // glfwWaitEvents();

    glfwMakeContextCurrent(nullptr);
  }

private:
  mjModel * model_ = nullptr;
  mjData * data_ = nullptr;

  tf2_msgs::msg::TFMessage transforms;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_tf = nullptr;
  realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>::UniquePtr pub_rt_tf = nullptr;

  GLFWwindow * window = nullptr;
  struct
  {
    mjvCamera cam;
    mjvOption opt;
    mjvScene scn;
    mjrContext con;
  } mjvis;

  bool button_left = false;
  bool button_middle = false;
  bool button_right = false;
  double lastx = 0;
  double lasty = 0;

  bool visualise = false;
  std::string imu_name;
};

hardware_interface::CallbackReturn MuJoCoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  const CallbackReturn bcr = hardware_interface::SystemInterface::on_init(params);

  if (bcr != CallbackReturn::SUCCESS)
  {
    return bcr;
  }

  if (params.hardware_info.joints.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("MuJoCoSystem"), "No joints specified in hardware info");
    return CallbackReturn::FAILURE;
  }

  const std::string model_path = params.hardware_info.hardware_parameters.count("mj_model_path")
                                   ? params.hardware_info.hardware_parameters.at("mj_model_path")
                                   : "robot.mjcf";

  visualise = params.hardware_info.hardware_parameters.count("visualise");

  const double gravity = params.hardware_info.hardware_parameters.count("gravity")
                           ? std::stod(params.hardware_info.hardware_parameters.at("gravity"))
                           : -9.81;  // m/s^2 in z direction

  imu_name = params.hardware_info.hardware_parameters.at("imu_name");

  char error[1000] = "";
  model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);
  if (!model_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MuJoCoSystem"), "Failed to load MuJoCo model: %s", error);
    return CallbackReturn::FAILURE;
  }

  model_->opt.gravity[2] = gravity;

  data_ = mj_makeData(model_);

  if (!data_)
  {
    RCLCPP_ERROR(rclcpp::get_logger("MuJoCoSystem"), "Failed to create MuJoCo data");
    mj_deleteModel(model_);
    model_ = nullptr;
    return CallbackReturn::FAILURE;
  }

  transforms.transforms.resize(model_->nbody);

  pub_tf =
    get_node()->create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS());

  pub_rt_tf = std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(pub_tf);

  if (visualise)
  {
    if (!glfwInit())
    {
      return CallbackReturn::FAILURE;
    }

    window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0);

    glfwSetKeyCallback(
      window,
      [](GLFWwindow * w, int key, int scancode, int act, int mods)
      {
        auto * system = static_cast<MuJoCoSystem *>(glfwGetWindowUserPointer(w));
        if (system)
        {
          system->keyboard(w, key, scancode, act, mods);
        }
      });

    glfwSetWindowUserPointer(window, this);
    glfwSetMouseButtonCallback(
      window,
      [](GLFWwindow * w, int button, int act, int mods)
      {
        auto * system = static_cast<MuJoCoSystem *>(glfwGetWindowUserPointer(w));
        if (system)
        {
          system->mouse_button(w, button, act, mods);
        }
      });

    glfwSetCursorPosCallback(
      window,
      [](GLFWwindow * w, double xpos, double ypos)
      {
        auto * system = static_cast<MuJoCoSystem *>(glfwGetWindowUserPointer(w));
        if (system)
        {
          system->mouse_move(w, xpos, ypos);
        }
      });

    glfwSetScrollCallback(
      window,
      [](GLFWwindow * w, double xoffset, double yoffset)
      {
        auto * system = static_cast<MuJoCoSystem *>(glfwGetWindowUserPointer(w));
        if (system)
        {
          system->scroll(w, xoffset, yoffset);
        }
      });

    glfwSetWindowCloseCallback(window, [](GLFWwindow * w) { glfwDestroyWindow(w); });

    mjv_defaultCamera(&mjvis.cam);
    mjv_defaultOption(&mjvis.opt);
    mjv_defaultScene(&mjvis.scn);
    mjr_defaultContext(&mjvis.con);

    mjv_makeScene(model_, &mjvis.scn, 2000);
    mjr_makeContext(model_, &mjvis.con, mjFONTSCALE_150);

    glfwMakeContextCurrent(nullptr);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MuJoCoSystem::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const int gyro_id = mj_name2id(model_, mjOBJ_SENSOR, "body_gyro");
  if (gyro_id >= 0)
  {
    const int sensor_adr = model_->sensor_adr[gyro_id];
    set_state<double>(imu_name + "/angular_velocity.x", data_->sensordata[sensor_adr]);
    set_state<double>(imu_name + "/angular_velocity.y", data_->sensordata[sensor_adr + 1]);
    set_state<double>(imu_name + "/angular_velocity.z", data_->sensordata[sensor_adr + 2]);
  }

  const int accel_id = mj_name2id(model_, mjOBJ_SENSOR, "body_linacc");
  if (accel_id >= 0)
  {
    const int sensor_adr = model_->sensor_adr[accel_id];
    set_state<double>(imu_name + "/linear_acceleration.x", data_->sensordata[sensor_adr]);
    set_state<double>(imu_name + "/linear_acceleration.y", data_->sensordata[sensor_adr + 1]);
    set_state<double>(imu_name + "/linear_acceleration.z", data_->sensordata[sensor_adr + 2]);
  }

  const int quat_id = mj_name2id(model_, mjOBJ_SENSOR, "body_quat");
  if (quat_id >= 0)
  {
    const int sensor_adr = model_->sensor_adr[quat_id];
    set_state<double>(imu_name + "/orientation.w", data_->sensordata[sensor_adr]);
    set_state<double>(imu_name + "/orientation.x", data_->sensordata[sensor_adr + 1]);
    set_state<double>(imu_name + "/orientation.y", data_->sensordata[sensor_adr + 2]);
    set_state<double>(imu_name + "/orientation.z", data_->sensordata[sensor_adr + 3]);
  }

  // std::cout << "IMU readings: " << std::endl;
  // std::cout << "Orientation: "
  //           << get_state<double>("base_imu/orientation.x") << ", "
  //           << get_state<double>("base_imu/orientation.y") << ", "
  //           << get_state<double>("base_imu/orientation.z") << ", "
  //           << get_state<double>("base_imu/orientation.w") << std::endl;
  // std::cout << "Angular Velocity: "
  //           << get_state<double>("base_imu/angular_velocity.x") << ", "
  //           << get_state<double>("base_imu/angular_velocity.y") << ", "
  //           << get_state<double>("base_imu/angular_velocity.z") << std::endl;
  // std::cout << "Linear Acceleration: "
  //           << get_state<double>("base_imu/linear_acceleration.x") << ", "
  //           << get_state<double>("base_imu/linear_acceleration.y") << ", "
  //           << get_state<double>("base_imu/linear_acceleration.z") << std::endl;

  // https:  // mujoco.readthedocs.io/en/2.2.1/programming.html
  // To represent 3D orientations and rotations, MuJoCo uses unit quaternions - namely 4D unit
  // vectors arranged as q = (w, x, y, z).

  // Publish all rigid body transforms
  for (int i = 0; i < model_->nbody; i++)
  {
    const double * pos = &data_->xpos[3 * i];
    const double * quat = &data_->xquat[4 * i];

    geometry_msgs::msg::TransformStamped & transform = transforms.transforms[i];
    transform.header.stamp = time;
    transform.header.frame_id = "world";
    transform.child_frame_id = "mujoco/" + std::string(model_->names + model_->name_bodyadr[i]);

    // transform.transform.translation.x = pos[0];
    // transform.transform.translation.y = pos[1];
    // transform.transform.translation.z = pos[2];

    transform.transform.rotation.w = quat[0];
    transform.transform.rotation.x = quat[1];
    transform.transform.rotation.y = quat[2];
    transform.transform.rotation.z = quat[3];
  }

  pub_rt_tf->try_publish(transforms);

  // for (int i = 0; i < model_->nbody; i++)
  // {
  //   const double * pos = &data_->xpos[3 * i];    // body position
  //   const double * xmat = &data_->xmat[9 * i];   // rotation matrix (3x3)
  //   const double * quat = &data_->xquat[4 * i];  // quaternion

  //   std::cout << "Body " << i << " (" << model_->names + model_->name_bodyadr[i] << ")\n";
  //   std::cout << "  pos = [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]\n";

  //   std::cout << "  quat = [" << quat[0] << ", " << quat[1] << ", " << quat[2] << ", " << quat[3]
  //             << "]\n";

  //   std::cout << "  rot matrix:\n";
  //   for (int r = 0; r < 3; r++)
  //   {
  //     std::cout << "    " << xmat[3 * r] << " " << xmat[3 * r + 1] << " " << xmat[3 * r + 2]
  //               << "\n";
  //   }
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MuJoCoSystem::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (int i = 0; i < model_->nu; ++i)
  {
    data_->ctrl[i] = 0;
  }

  // std::cout << "rate: " << 1 / period.seconds() << std::endl;

  const std::string actuator_prefix = "rotor";
  constexpr uint8_t nrotors = 4;

  for (uint8_t i = 0; i < nrotors; i++)
  {
    const float umin = model_->actuator_ctrlrange[(i * 2) + 0];
    const float umax = model_->actuator_ctrlrange[(i * 2) + 1];
    const double cmd = get_command<double>(
      actuator_prefix + "/" + std::to_string(i + 1) + "/" + hardware_interface::HW_IF_VELOCITY);

    data_->ctrl[i] = umin + cmd * (umax - umin);
  }

  mj_step(model_, data_);

  if (visualise)
  {
    render();
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MuJoCoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (data_)
  {
    mj_deleteData(data_), data_ = nullptr;
  }
  if (model_)
  {
    mj_deleteModel(model_), model_ = nullptr;
  }

  if (visualise)
  {
    mjv_freeScene(&mjvis.scn);
    mjr_freeContext(&mjvis.con);
    mj_deleteData(data_);
    mj_deleteModel(model_);

    glfwTerminate();
  }

  return CallbackReturn::SUCCESS;
}

}  // namespace uav_mujoco_ros2_control

PLUGINLIB_EXPORT_CLASS(uav_mujoco_ros2_control::MuJoCoSystem, hardware_interface::SystemInterface)
