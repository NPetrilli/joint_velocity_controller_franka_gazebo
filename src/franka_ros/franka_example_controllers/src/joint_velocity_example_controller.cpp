

// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <franka_example_controllers/pseudo_inversion.h>

namespace franka_example_controllers {

bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,ros::NodeHandle& node_handle) {

  sub_trajectory = node_handle.subscribe(
      "trajectory_loader", 20, &JointVelocityExampleController::trajectoryPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

  target_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("position_d_target", 10);
  
  
  velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();

  if (velocity_joint_interface_ == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Error getting velocity joint interface from hardware!");
    return false;
  }

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }

  velocity_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("JointVelocityExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();

  if (model_interface==nullptr){
    ROS_ERROR_STREAM("JointVelocityExampleController: Error getting model interface from hardwer");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }


  auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
    return false;
  }

  try {
  //joint velocity
   auto state_handle = state_interface->getHandle(arm_id + "_robot");
  //jacobian
   state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));

    std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    for (size_t i = 0; i < q_start.size(); i++) {
      if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
        ROS_ERROR_STREAM("JointVelocityExampleController: Robot is not in the expected starting position for "
                         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
                         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
        return false;
      }
    }
  } catch (const hardware_interface::HardwareInterfaceException& e) {
    ROS_ERROR_STREAM("JointVelocityExampleController: Exception getting state handle: " << e.what());
    return false;
  }

  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  ROS_INFO_STREAM("pos_d_init:" << position_d_.transpose());
  ROS_INFO_STREAM("pos_d_target_init: " << position_d_target_.transpose());

  return true;
}

void JointVelocityExampleController::starting(const ros::Time& /* time */) {
  elapsed_time_ = ros::Duration(0.0);

  franka::RobotState initial_state = state_handle_->getRobotState();
  std::array<double,42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double,7,1>>q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());

  
  ROS_INFO_STREAM("Initial Position_starting " << initial_transform.translation().transpose());
  ROS_INFO_STREAM("Initial Desired_starting : " << position_d_.transpose());

}

Eigen::Matrix<double, 3, 1> computePoseError(const Eigen::Affine3d& T, const Eigen::Affine3d& Td) {
  //PC's function
  Eigen::Matrix<double, 3, 1> e;
  Eigen::Matrix3d R = Td.rotation() * T.rotation().transpose();
 
  Eigen::Vector3d li;
  li << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);

  if (li.norm() < 1e-6) {
    //Diagonal case: no rotation
    if (R.trace() > 0) {
      e.tail(3).setZero();
    } else {
    //Special case: PI rotation
      e.tail(3) = M_PI / 2 * (R.diagonal().array() + 1).matrix();
    }
  } else {
    //NO diagonal case, computing axis-angle
    double ln = li.norm();
    e.tail(3) = atan2(ln, R.trace() - 1) * li / ln;
  }

  return e;
}

void JointVelocityExampleController::update(const ros::Time& time, const ros::Duration& period) {
  
  //Taking robot model and state and jacobian matrix
  franka::RobotState robot_state = state_handle_->getRobotState();
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
  Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

  //Obtaining EE pose (traslation+rotation)
  Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
  Eigen::Vector3d position(transform.translation());
  Eigen::Quaterniond orientation(transform.rotation());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  //Computing pinv of jacobian
  Eigen::MatrixXd jacobian_pinv;
  pseudoInverse(jacobian, jacobian_pinv);

  //----------------------ERROR--------------------------------------------

  //Error position
  Eigen::Matrix<double, 6, 1> error;
  error.head(3) << position_d_ - position;
  
  //Error orientation
  Eigen::Affine3d Te;
  Te.translation() = position_d_target_;
  Te.linear() = orientation_d_target_.toRotationMatrix();
  
  Eigen::Matrix<double, 3, 1> error_rot = computePoseError(transform, Te);
  error.tail(3)=error_rot;

  //----------------------VELOCITY------------------------------------------
  //Velocity position
  Eigen::Matrix<double, 6, 1> xd_dot;
  xd_dot.head(3) = (position_d_target_ - position_d_) / period.toSec();

  //Velocity orientation
  Eigen::Matrix3d rot_orientation_d_ = orientation_d_.toRotationMatrix();
  Eigen::Matrix3d rot_orientation_d_target_ = orientation_d_target_.toRotationMatrix();

  Eigen::Vector3d euler_ang_d_ = rot_orientation_d_.eulerAngles(0, 1, 2);  //XYZ
  Eigen::Vector3d euler_ang_d_target_= rot_orientation_d_target_.eulerAngles(0, 1, 2);

  xd_dot.tail(3) = ( euler_ang_d_target_ - euler_ang_d_ ) / period.toSec();

  //xd_dot.tail(3).setZero();
  

  //----------------------TUNING---------------------------------------------
  Eigen::Matrix<double, 6, 6> K;
  K.setIdentity();
  K.diagonal() << 80, 70, 80, 30, 30, 40;


  //------------------Differential Kinematics equation-----------------------
  Eigen::Matrix<double, 7, 1> q_dot;
  q_dot = jacobian_pinv * (xd_dot + K * error);//+(Eigen::MatrixXd::Identity(7, 7)-jacobian_pinv*jacobian)*dq;

  position_d_ = position_d_target_;
  orientation_d_ = orientation_d_target_;

  for (size_t i = 0; i < 7; i++) {
    velocity_joint_handles_[i].setCommand(q_dot[i]);
  }



  // Debug
  // ROS_INFO_STREAM("Position_d_target_: " << position_d_target_.transpose());
  // ROS_INFO_STREAM("Position: " << position.transpose());
  //ROS_INFO_STREAM("Error: " << error.tail(3).transpose());
  //ROS_INFO_STREAM("x_dot: " << xd_dot.transpose());
}


void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
  // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
  // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
  // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
}

void JointVelocityExampleController::trajectoryPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  
  std::lock_guard<std::mutex> position_d_target_mutex_lock(position_and_orientation_d_target_mutex_);
  
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  
 
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,msg->pose.orientation.z, msg->pose.orientation.w;
  
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}
}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
                       controller_interface::ControllerBase)

