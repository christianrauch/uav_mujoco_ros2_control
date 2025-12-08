#include <string>
#include <vector>
#include <memory>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>


namespace uav_mujoco_ros2_control
{

class MujocoSystem : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    virtual ~MujocoSystem();

private:
    // keyboard callback
    void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
        // backspace: reset simulation
        if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
            mj_resetData(model_, data_);
            mj_forward(model_, data_);
        }
        if (act==GLFW_PRESS && key==GLFW_KEY_ESCAPE) {
            glfwDestroyWindow(window);
        }
    }


    // mouse button callback
    void mouse_button(GLFWwindow* window, int button, int act, int mods) {
        // update button state
        button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
        button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
        button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

        // update mouse position
        glfwGetCursorPos(window, &lastx, &lasty);
    }


    // mouse move callback
    void mouse_move(GLFWwindow* window, double xpos, double ypos) {
        // no buttons down: nothing to do
        if (!button_left && !button_middle && !button_right) {
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
        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                          glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

        // determine action based on mouse button
        mjtMouse action;
        if (button_right) {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        } else if (button_left) {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        } else {
            action = mjMOUSE_ZOOM;
        }

        // move camera
        mjv_moveCamera(model_, action, dx/height, dy/height, &mjvis.scn, &mjvis.cam);
    }


    // scroll callback
    void scroll(GLFWwindow* window, double xoffset, double yoffset) {
        // emulate vertical mouse motion = 5% of window height
        mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mjvis.scn, &mjvis.cam);
    }

private:
    mjModel * model_ = nullptr;
    mjData * data_ = nullptr;

    GLFWwindow* window = nullptr;
    struct {
        mjvCamera cam;
        mjvOption opt;
        mjvScene scn;
        mjrContext con;
    } mjvis;

    bool button_left = false;
    bool button_middle = false;
    bool button_right =  false;
    double lastx = 0;
    double lasty = 0;
};


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

  // are we in the main thread here?
  const pid_t pid = getpid();
  const pid_t tid = gettid();
  if (tid == pid) {
      std::cout << "Hello from main thread (" << tid << ")" << std::endl;
  }
  else {
      std::cout << "Hello from child thread (" << tid << ")" << std::endl;
  }

  // init GLFW
  if (!glfwInit()) {
      return CallbackReturn::FAILURE;
  }

  // create window
  window = glfwCreateWindow(1200, 900, "MuJoCo Simulation", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // Create a static lambda that captures the window pointer
  // auto* self = this;
  glfwSetKeyCallback(window, [](GLFWwindow* w, int key, int scancode, int act, int mods) {
    auto* system = static_cast<MujocoSystem*>(glfwGetWindowUserPointer(w));
    if (system) system->keyboard(w, key, scancode, act, mods);
  });

  glfwSetWindowUserPointer(window, this);
  glfwSetMouseButtonCallback(window, [](GLFWwindow* w, int button, int act, int mods) {
    auto* system = static_cast<MujocoSystem*>(glfwGetWindowUserPointer(w));
    if (system) system->mouse_button(w, button, act, mods);
  });

  glfwSetCursorPosCallback(window, [](GLFWwindow* w, double xpos, double ypos) {
    auto* system = static_cast<MujocoSystem*>(glfwGetWindowUserPointer(w));
    if (system) system->mouse_move(w, xpos, ypos);
  });

  glfwSetScrollCallback(window, [](GLFWwindow* w, double xoffset, double yoffset) {
    auto* system = static_cast<MujocoSystem*>(glfwGetWindowUserPointer(w));
    if (system) system->scroll(w, xoffset, yoffset);
  });

  mjv_defaultCamera(&mjvis.cam);
  mjv_defaultOption(&mjvis.opt);
  mjv_defaultScene(&mjvis.scn);
  mjr_defaultContext(&mjvis.con);

  mjv_makeScene(model_, &mjvis.scn, 2000);
  mjr_makeContext(model_, &mjvis.con, mjFONTSCALE_150);

  glfwMakeContextCurrent(nullptr);

  return CallbackReturn::SUCCESS;
}

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

  // for (int i = 0; i < model_->nbody; i++) {
  //     const double* pos  = &data_->xpos[3 * i];   // body position
  //     const double* xmat = &data_->xmat[9 * i];   // rotation matrix (3x3)
  //     const double* quat = &data_->xquat[4 * i];  // quaternion

  //     std::cout << "Body " << i << " (" << model_->names + model_->name_bodyadr[i] << ")\n";
  //     std::cout << "  pos = [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]\n";

  //     std::cout << "  quat = ["
  //               << quat[0] << ", " << quat[1] << ", "
  //               << quat[2] << ", " << quat[3] << "]\n";

  //     std::cout << "  rot matrix:\n";
  //     for (int r = 0; r < 3; r++) {
  //         std::cout << "    "
  //                   << xmat[3*r] << " "
  //                   << xmat[3*r+1] << " "
  //                   << xmat[3*r+2] << "\n";
  //     }
  // }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (int i = 0; i < model_->nu; ++i) data_->ctrl[i] = 0.0;

  mj_step(model_, data_);

  // const pid_t pid = getpid();
  // const pid_t tid = gettid();
  // if (tid == pid) {
  //     std::cout << "Hello from main thread (" << tid << ")" << std::endl;
  // }
  // else {
  //     std::cout << "Hello from child thread (" << tid << ")" << std::endl;
  // }

  // std::cout << "context ..." << std::endl;

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

  // std::cout << "bla ..." << std::endl;

  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MujocoSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (data_) mj_deleteData(data_), data_ = nullptr;
  if (model_) mj_deleteModel(model_), model_ = nullptr;

  // cleanup
  mjv_freeScene(&mjvis.scn);
  mjr_freeContext(&mjvis.con);
  mj_deleteData(data_);
  mj_deleteModel(model_);

  glfwTerminate();

  return CallbackReturn::SUCCESS;
}

MujocoSystem::~MujocoSystem()
{
  if (data_) mj_deleteData(data_);
  if (model_) mj_deleteModel(model_);
}

}
PLUGINLIB_EXPORT_CLASS(uav_mujoco_ros2_control::MujocoSystem, hardware_interface::SystemInterface)
