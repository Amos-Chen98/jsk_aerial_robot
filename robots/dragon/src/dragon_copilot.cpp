#include <dragon/dragon_copilot.h>
#include <aerial_robot_control/util/joy_parser.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

DragonCopilot::DragonCopilot()
  : DragonNavigator()
  , r2_trigger_initialized_(false)
  , l2_trigger_initialized_(false)
  , last_commanded_pitch_(0.0)
  , hold_attitude_on_idle_(true)
{
}

void DragonCopilot::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du)
{
  /* initialize the parent class */
  DragonNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);
}

void DragonCopilot::rosParamInit()
{
  DragonNavigator::rosParamInit();

  /* Load copilot-specific parameters */
  ros::NodeHandle navi_nh(nh_, "navigation/copilot");

  getParam<double>(navi_nh, "max_x_vel", max_copilot_x_vel_, 1.0);
  getParam<double>(navi_nh, "max_y_vel", max_copilot_y_vel_, 1.0);
  getParam<double>(navi_nh, "max_z_vel", max_copilot_z_vel_, 0.5);
  getParam<double>(navi_nh, "max_yaw_vel", max_copilot_yaw_vel_, 0.5);
  getParam<double>(navi_nh, "max_pitch_angle", max_copilot_pitch_angle_, 0.5);  // ~30 degrees
  getParam<double>(navi_nh, "trigger_deadzone", trigger_deadzone_, 0.1);
  getParam<bool>(navi_nh, "hold_attitude_on_idle", hold_attitude_on_idle_, true);

  ROS_INFO("[DragonCopilot] Copilot mode initialized with custom joystick mapping");
  ROS_INFO("[DragonCopilot] - Max X velocity: %.2f m/s", max_copilot_x_vel_);
  ROS_INFO("[DragonCopilot] - Max Y velocity: %.2f m/s", max_copilot_y_vel_);
  ROS_INFO("[DragonCopilot] - Max Z velocity: %.2f m/s", max_copilot_z_vel_);
  ROS_INFO("[DragonCopilot] - Max Yaw velocity: %.2f rad/s", max_copilot_yaw_vel_);
  ROS_INFO("[DragonCopilot] - Max Pitch angle: %.2f rad", max_copilot_pitch_angle_);
  ROS_INFO("[DragonCopilot] - Hold attitude on idle: %s", hold_attitude_on_idle_ ? "true" : "false");
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

  // Left stick horizontal: yaw rotation
  double raw_yaw_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];
  // Left stick vertical: pitch attitude
  double raw_pitch_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];

  // Right stick horizontal: lateral translation (y-axis)
  double raw_y_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];
  // Right stick vertical: vertical translation (z-axis)
  double raw_z_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];

  /* Process X-axis motion (forward/backward) from triggers */
  // Convert trigger values: neutral(+1) -> 0, full(-1) -> max_vel
  // Forward movement (R2): positive x velocity
  double forward_cmd = 0.0;
  if (raw_r2 < (1.0 - trigger_deadzone_))
  {
    forward_cmd = (1.0 - raw_r2) / 2.0;  // normalize to [0, 1]
  }

  // Backward movement (L2): negative x velocity
  double backward_cmd = 0.0;
  if (raw_l2 < (1.0 - trigger_deadzone_))
  {
    backward_cmd = (1.0 - raw_l2) / 2.0;  // normalize to [0, 1]
  }

  // Calculate net x velocity command
  double x_vel_cmd = (forward_cmd - backward_cmd) * max_copilot_x_vel_;

  /* Process Y-axis motion (lateral translation) */
  double y_vel_cmd = 0.0;
  if (fabs(raw_y_cmd) > joy_stick_deadzone_)
  {
    y_vel_cmd = raw_y_cmd * max_copilot_y_vel_;
  }

  /* Process Z-axis motion (vertical translation) */
  double z_vel_cmd = 0.0;
  if (fabs(raw_z_cmd) > joy_stick_deadzone_)
  {
    z_vel_cmd = raw_z_cmd * max_copilot_z_vel_;
  }

  /* Process Yaw motion (rotation around z-axis) */
  double yaw_vel_cmd = 0.0;
  if (fabs(raw_yaw_cmd) > joy_stick_deadzone_)
  {
    yaw_vel_cmd = raw_yaw_cmd * max_copilot_yaw_vel_;
  }

  /* Process Pitch attitude control */
  double pitch_cmd = 0.0;
  bool has_pitch_input = false;

  if (fabs(raw_pitch_cmd) > joy_stick_deadzone_)
  {
    pitch_cmd = raw_pitch_cmd * max_copilot_pitch_angle_;
    has_pitch_input = true;
  }
  else if (hold_attitude_on_idle_)
  {
    // When no input and attitude hold is enabled, use last commanded pitch
    pitch_cmd = last_commanded_pitch_;
  }
  // else: pitch_cmd remains 0.0 (level flight)

  // Update last commanded pitch if there was input
  if (has_pitch_input)
  {
    last_commanded_pitch_ = pitch_cmd;
  }

  /* Transform velocity commands to world frame */
  // Strategy:
  // - R2/L2 (x-axis) and right stick horizontal (y-axis): follow body orientation (including pitch)
  //   This means forward/backward motion will climb/descend when body is pitched
  // - Right stick vertical (z-axis): adds pure vertical velocity in world frame
  //   This allows independent altitude control regardless of body attitude

  // Get current body orientation
  tf::Quaternion body_orientation;
  estimator_->getOrientation(Frame::BASELINK, estimate_mode_).getRotation(body_orientation);
  tf::Matrix3x3 rot_mat(body_orientation);

  // Transform x-y velocity in body frame (follows body pitch and yaw)
  // Note: z component is 0 in body frame for R2/L2 and lateral stick
  tf::Vector3 body_vel_xy(x_vel_cmd, y_vel_cmd, 0);
  tf::Vector3 world_vel_xy = rot_mat * body_vel_xy;

  /* Set velocity targets in world frame */
  // Combine body-frame transformed x-y with world-frame z
  setTargetVelX(world_vel_xy.x());
  setTargetVelY(world_vel_xy.y());
  setTargetVelZ(world_vel_xy.z() + z_vel_cmd);  // Body z component + world frame z command

  /* Set angular velocity target (yaw) */
  setTargetOmegaZ(yaw_vel_cmd);

  /* Set pitch attitude target through baselink rotation */
  // Get current baselink rotation
  tf::Vector3 current_rpy;
  double r, p, y;
  tf::Matrix3x3(curr_target_baselink_rot_).getRPY(r, p, y);

  // Update pitch target
  double target_pitch = pitch_cmd;

  // Smooth pitch command to avoid abrupt changes
  double pitch_diff = target_pitch - p;
  if (fabs(pitch_diff) > baselink_rot_change_thresh_)
  {
    p += pitch_diff / fabs(pitch_diff) * baselink_rot_change_thresh_;
  }
  else
  {
    p = target_pitch;
  }

  // Update target baselink rotation with new pitch
  final_target_baselink_rot_.setRPY(0, p, 0);  // Keep roll=0, yaw controlled by body frame

  /* Debug output (optional) */
  if (param_verbose_)
  {
    ROS_INFO_THROTTLE(1.0, "[DragonCopilot] X: %.2f, Y: %.2f, Z: %.2f, Yaw: %.2f, Pitch: %.2f", world_vel_xy.x(),
                      world_vel_xy.y(), z_vel_cmd, yaw_vel_cmd, target_pitch);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::DragonCopilot, aerial_robot_navigation::BaseNavigator);
