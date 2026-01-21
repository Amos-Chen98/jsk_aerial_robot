#include <dragon/dragon_copilot.h>
#include <aerial_robot_control/util/joy_parser.h>
#include <tf_conversions/tf_kdl.h>
#include <algorithm>
#include <limits>
#include <cmath>
#include <iomanip>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

DragonCopilot::DragonCopilot()
  : DragonNavigator()
  , r2_trigger_initialized_(false)
  , l2_trigger_initialized_(false)
  , root_pitch_cmd_(0.0)
  , root_yaw_cmd_(0.0)
  , link_num_(0)  // Will be initialized from robot model
{
}

void DragonCopilot::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du)
{
  /* initialize the parent class */
  DragonNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  copilot_ = std::make_shared<DragonCopilotControl>(robot_model, estimator, loop_du, copilot_params_);
  if (copilot_)
  {
    cout << "[DragonCopilot] Copilot control module initialized." << endl;
  }

  /* Initialize Jacobian computation components */
  const KDL::Tree& tree = robot_model_->getTree();

  link_num_ = robot_model_->getRotorNum();  // For Dragon, rotor_num equals link_num

  ROS_INFO("[DragonCopilot] Link num: %d", link_num_);

  /* Get link_length from transformable robot model */
  auto transformable_model = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(robot_model_);
  if (transformable_model)
  {
    link_joint_indices_.clear();
    link_joint_indices_ = transformable_model->getLinkJointIndices();
    link_joint_num_ = link_joint_indices_.size();
    ROS_INFO("[DragonCopilot] Link joint indices initialized: %d joints", link_joint_num_);

    // Get joint limits from URDF model
    link_joint_lower_limits_ = transformable_model->getLinkJointLowerLimits();
    link_joint_upper_limits_ = transformable_model->getLinkJointUpperLimits();
    ROS_INFO("[DragonCopilot] Joint limits loaded from URDF: lower=[%.3f, ...], upper=[%.3f, ...]",
             link_joint_lower_limits_.empty() ? 0.0 : link_joint_lower_limits_[0],
             link_joint_upper_limits_.empty() ? 0.0 : link_joint_upper_limits_[0]);

    std::stringstream ss;
    ss << "[DragonCopilot] Link joint indices: ";
    for (const auto& idx : link_joint_indices_)
    {
      ss << idx << " ";
    }
    ROS_INFO("%s", ss.str().c_str());

    /* Pre-compute all link names */
    link_names_.reserve(link_num_);
    for (int i = 1; i <= link_num_; i++)
    {
      link_names_.push_back("link" + std::to_string(i));
    }
  }
  else
  {
    ROS_WARN("[DragonCopilot] Could not cast to TransformableRobotModel");
  }

  /* Initialize snake following publishers and subscribers */
  target_rotation_motion_pub_ = nh_.advertise<nav_msgs::Odometry>("target_rotation_motion", 1);
  snake_trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("copilot/snake_trajectory", 1);
  reset_trajectory_sub_ =
      nh_.subscribe("copilot/reset_trajectory", 1, &DragonCopilot::resetTrajectoryBufferCallback, this);

  /* Calculate snake_max_joint_delta from max_rot_vel and loop period */
  ROS_INFO("[DragonCopilot] Loop duration: %.4f s", loop_du);
}

void DragonCopilot::rosParamInit()
{
  DragonNavigator::rosParamInit();

  /* Load copilot-specific parameters */
  ros::NodeHandle navi_nh(nh_, "navigation/copilot");

  getParam<double>(navi_nh, "max_x_vel", copilot_params_.max_copilot_x_vel, 1.0);
  getParam<double>(navi_nh, "max_y_vel", copilot_params_.max_copilot_y_vel, 1.0);
  getParam<double>(navi_nh, "max_z_vel", copilot_params_.max_copilot_z_vel, 0.5);
  getParam<double>(navi_nh, "max_rot_vel", copilot_params_.max_copilot_rot_vel, 0.5);  // rad/s for both pitch and yaw
  getParam<double>(navi_nh, "trigger_deadzone", copilot_params_.joy_stick_deadzone, 0.1);
  getParam<bool>(navi_nh, "hold_attitude_on_idle", copilot_params_.hold_attitude_on_idle, true);
  /* Load snake-following parameters */
  getParam<bool>(navi_nh, "snake_mode_enabled", copilot_params_.snake_mode_enabled, true);
  getParam<double>(navi_nh, "trajectory_sample_interval", copilot_params_.trajectory_sample_interval, 0.1);
  getParam<double>(navi_nh, "trajectory_buffer_max_length", copilot_params_.trajectory_buffer_max_length, 3.0);
  getParam<double>(navi_nh, "snake_ik_gain", copilot_params_.snake_ik_gain, 10.0);

  ROS_INFO("[DragonCopilot] Copilot mode initialized with custom joystick mapping");
  ROS_INFO("[DragonCopilot] - Max X velocity: %.2f m/s", copilot_params_.max_copilot_x_vel);
  ROS_INFO("[DragonCopilot] - Max Y velocity: %.2f m/s", copilot_params_.max_copilot_y_vel);
  ROS_INFO("[DragonCopilot] - Max Z velocity: %.2f m/s", copilot_params_.max_copilot_z_vel);
  ROS_INFO("[DragonCopilot] - Max rotation velocity: %.2f rad/s", copilot_params_.max_copilot_rot_vel);
  ROS_INFO("[DragonCopilot] - Hold attitude on idle: %s", copilot_params_.hold_attitude_on_idle ? "true" : "false");
  ROS_INFO("[DragonCopilot] - Pitch/Yaw commands control joint1_pitch and joint1_yaw");
  ROS_INFO("[DragonCopilot] Snake following mode: %s", copilot_params_.snake_mode_enabled ? "enabled" : "disabled");
  ROS_INFO("[DragonCopilot] - Trajectory sample interval: %.3f m", copilot_params_.trajectory_sample_interval);
  ROS_INFO("[DragonCopilot] - Trajectory buffer max length: %.2f m", copilot_params_.trajectory_buffer_max_length);
}

void DragonCopilot::joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg)
{
  sensor_msgs::Joy joy_cmd = joyParse(*joy_msg);
  if (joy_cmd.axes.size() == 0 || joy_cmd.buttons.size() == 0)
  {
    ROS_WARN("[DragonCopilot] the joystick type is not supported (buttons: %d, axes: %d)", (int)joy_msg->buttons.size(),
             (int)joy_msg->axes.size());
    return;
  }

  if (!joy_stick_heart_beat_)
    joy_stick_heart_beat_ = true;
  joy_stick_prev_time_ = ros::Time::now().toSec();

  /* ========== Common Commands (inherited from BaseNavigator) ========== */

  /* Start/Motor Arming */
  if (joy_cmd.buttons[JOY_BUTTON_START] == 1 && getNaviState() == ARM_OFF_STATE)
  {
    motorArming();
    return;
  }

  /* Force Landing && Halt */
  if (joy_cmd.buttons[JOY_BUTTON_STOP] == 1)
  {
    /* Force Landing in inflight mode: TAKEOFF_STATE/LAND_STATE/HOVER_STATE */
    if (!force_landing_flag_ &&
        (getNaviState() == TAKEOFF_STATE || getNaviState() == LAND_STATE || getNaviState() == HOVER_STATE))
    {
      ROS_WARN("[DragonCopilot] Joy Control: force landing state");
      spinal::FlightConfigCmd flight_config_cmd;
      flight_config_cmd.cmd = spinal::FlightConfigCmd::FORCE_LANDING_CMD;
      flight_config_pub_.publish(flight_config_cmd);
      force_landing_flag_ = true;

      /* update the force landing stamp for the halt process*/
      force_landing_start_time_ = joy_cmd.header.stamp;
    }

    /* Halt mode */
    if (joy_cmd.header.stamp.toSec() - force_landing_start_time_.toSec() > force_landing_to_halt_du_ &&
        getNaviState() > START_STATE)
    {
      ROS_ERROR("[DragonCopilot] Joy Control: Halt!");
      setNaviState(STOP_STATE);

      /* update the target pos */
      setTargetXyFromCurrentState();
      setTargetYawFromCurrentState();
    }
    return;
  }
  else
  {
    /* update the halt process */
    force_landing_start_time_ = joy_cmd.header.stamp;
  }

  /* Takeoff */
  if (joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] == 1 && joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] == 1)
  {
    startTakeoff();
    return;
  }

  /* Landing */
  if (joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] == 1 && joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] == 1)
  {
    if (force_att_control_flag_)
      return;
    if (getNaviState() == LAND_STATE)
      return;
    if (!teleop_flag_)
      return;

    setNaviState(LAND_STATE);
    ROS_INFO("[DragonCopilot] Joy Control: Land state");
    return;
  }

  teleop_reset_time_ = teleop_reset_duration_ + ros::Time::now().toSec();

  /* ========== Custom Copilot Control Mapping ========== */

  /* Only allow control in HOVER_STATE */
  if (getNaviState() != HOVER_STATE)
  {
    return;
  }

  /* Finish if teleop flag is not true */
  if (!teleop_flag_)
    return;

  /* Parse joystick inputs to get velocity and attitude commands */
  nav_msgs::Odometry root_cmd = parseJoystickInputs(joy_cmd);

  /* Transform velocity commands and set control targets */
  transformAndSetControlTargets(root_cmd);
}

nav_msgs::Odometry DragonCopilot::parseJoystickInputs(const sensor_msgs::Joy& joy_cmd)
{
  nav_msgs::Odometry root_cmd;
  // ---------- X-axis control (forward/backward) via R2/L2 triggers ----------
  // R2 trigger: forward (neutral=+1, full=-1, so we need to invert and normalize)
  // Handle initialization issue: ROS topic reports 0 until first press, then reports correctly
  double raw_r2 = 1.0;  // Default to neutral position
  double r2_value = joy_cmd.axes[JOY_AXIS_BUTTON_REAR_RIGHT_2];

  // Detect first press: value changes from 0 (uninitialized) to something else
  if (!r2_trigger_initialized_ && fabs(r2_value) > 0.01)
  {
    r2_trigger_initialized_ = true;
    ROS_INFO("[DragonCopilot] R2 trigger initialized");
  }

  // Only use actual value after initialization
  if (r2_trigger_initialized_)
  {
    raw_r2 = r2_value;
  }

  // L2 trigger: backward (neutral=+1, full=-1, so we need to invert and normalize)
  // Handle initialization issue: ROS topic reports 0 until first press, then reports correctly
  double raw_l2 = 1.0;  // Default to neutral position
  double l2_value = joy_cmd.axes[JOY_AXIS_BUTTON_REAR_LEFT_2];

  // Detect first press: value changes from 0 (uninitialized) to something else
  if (!l2_trigger_initialized_ && fabs(l2_value) > 0.01)
  {
    l2_trigger_initialized_ = true;
    ROS_INFO("[DragonCopilot] L2 trigger initialized");
  }

  // Only use actual value after initialization
  if (l2_trigger_initialized_)
  {
    raw_l2 = l2_value;
  }

  double forward_cmd = 0.0;
  if (raw_r2 < (1.0 - copilot_params_.joy_stick_deadzone))
  {
    forward_cmd = (1.0 - raw_r2) / 2.0;  // normalize to [0, 1]
  }

  double backward_cmd = 0.0;
  if (raw_l2 < (1.0 - copilot_params_.joy_stick_deadzone))
  {
    backward_cmd = (1.0 - raw_l2) / 2.0;  // normalize to [0, 1]
  }

  // Calculate net x velocity command
  // R2 (forward_cmd) -> positive X velocity (root frame forward direction)
  // L2 (backward_cmd) -> negative X velocity (root frame backward direction)
  root_cmd.twist.twist.linear.x = (forward_cmd - backward_cmd) * copilot_params_.max_copilot_x_vel;

  // ---------- Y-axis control via JOY_AXIS_STICK_RIGHT ----------
  // Right stick horizontal: lateral translation (y-axis)
  double raw_y_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];

  /* Process Y-axis motion (lateral translation) */
  if (fabs(raw_y_cmd) > copilot_params_.joy_stick_deadzone)
  {
    root_cmd.twist.twist.linear.y = raw_y_cmd * copilot_params_.max_copilot_y_vel;
  }

  // ---------- Z-axis control via JOY_AXIS_STICK_RIGHT ----------
  // Right stick vertical: vertical translation (z-axis)
  double raw_z_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];

  /* Process Z-axis motion (vertical translation) */
  if (fabs(raw_z_cmd) > copilot_params_.joy_stick_deadzone)
  {
    root_cmd.twist.twist.linear.z = raw_z_cmd * copilot_params_.max_copilot_z_vel;
  }

  // ---------- pitch control via JOY_AXIS_STICK_LEFT ----------
  // Left stick vertical: pitch angular velocity
  double raw_pitch_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];

  /* Process Pitch velocity control */
  if (fabs(raw_pitch_cmd) > copilot_params_.joy_stick_deadzone)
  {
    // Active input: apply pitch velocity
    root_cmd.twist.twist.angular.y = raw_pitch_cmd * copilot_params_.max_copilot_rot_vel;
  }
  else if (copilot_params_.hold_attitude_on_idle)
  {
    // When no input and attitude hold is enabled, set pitch_vel to 0 (hold current attitude)
    root_cmd.twist.twist.angular.y = 0.0;
  }
  // else: root_cmd.pitch_vel remains 0.0 from constructor (will return to level flight)

  // ---------- yaw control via JOY_AXIS_STICK_LEFT ----------
  // Left stick horizontal: yaw rotation
  double raw_yaw_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];

  /* Process Yaw motion (rotation around z-axis) */
  if (fabs(raw_yaw_cmd) > copilot_params_.joy_stick_deadzone)
  {
    root_cmd.twist.twist.angular.z = raw_yaw_cmd * copilot_params_.max_copilot_rot_vel;
  }

  // Finally, revert x and y axis, so that the forward on joystick means forward along the head direction
  // Also revert yaw so that stick right increases joint1_yaw
  root_cmd.twist.twist.linear.x = -root_cmd.twist.twist.linear.x;
  root_cmd.twist.twist.linear.y = -root_cmd.twist.twist.linear.y;
  root_cmd.twist.twist.angular.y = -root_cmd.twist.twist.angular.y;
  root_cmd.twist.twist.angular.z = root_cmd.twist.twist.angular.z;

  return root_cmd;
}

void DragonCopilot::transformAndSetControlTargets(const nav_msgs::Odometry& root_cmd)
{
  if (!copilot_)
    return;

  copilot_->updateControlState(root_cmd);

  Eigen::VectorXd desired_joint_positions = copilot_->computeJointPositions(root_cmd);

  publishJointCommands(desired_joint_positions);

  // Set CoG Target Velocity
  Eigen::Vector3d cog_vel = copilot_->computeCoGVelocity();
  setTargetVelX(cog_vel.x());
  setTargetVelY(cog_vel.y());
  setTargetVelZ(cog_vel.z());

  // Set Baselink Target Pose
  nav_msgs::Odometry baselink_pose = copilot_->computeBaselinkTargetPose(root_cmd);
  target_rotation_motion_pub_.publish(baselink_pose);

  // Publish visualization
  snake_trajectory_viz_pub_.publish(copilot_->getSnakeTrajectoryVizMsg());
}

void DragonCopilot::publishJointCommands(const Eigen::VectorXd& desired_joint_positions)
{
  KDL::JntArray joint_positions = robot_model_->getJointPositions();

  // Get current joint positions for comparison
  Eigen::VectorXd current_link_joint_positions(link_joint_num_);
  for (int i = 0; i < link_joint_num_; i++)
  {
    int joint_idx = link_joint_indices_[i];
    current_link_joint_positions(i) = joint_positions(joint_idx);
  }

  // Clamp desired positions to joint limits (from URDF)
  Eigen::VectorXd clamped_desired_positions = desired_joint_positions;
  for (int i = 0; i < link_joint_num_; i++)
  {
    clamped_desired_positions(i) =
        std::clamp(clamped_desired_positions(i), link_joint_lower_limits_[i], link_joint_upper_limits_[i]);
  }

  // Publish joint command message
  sensor_msgs::JointState joint_control_msg;
  joint_control_msg.header.stamp = ros::Time::now();
  joint_control_msg.position.resize(link_joint_num_);
  for (int i = 0; i < link_joint_num_; i++)
  {
    joint_control_msg.position[i] = clamped_desired_positions(i);
  }

  joint_control_pub_.publish(joint_control_msg);
}

void DragonCopilot::resetTrajectoryBufferCallback(const std_msgs::EmptyConstPtr& msg)
{
  if (copilot_)
    copilot_->resetTrajectoryBuffer();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::DragonCopilot, aerial_robot_navigation::BaseNavigator);
