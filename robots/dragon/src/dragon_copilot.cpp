#include <dragon/dragon_copilot.h>
#include <aerial_robot_control/util/joy_parser.h>
#include <tf_conversions/tf_kdl.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

DragonCopilot::DragonCopilot()
  : DragonNavigator()
  , r2_trigger_initialized_(false)
  , l2_trigger_initialized_(false)
  , last_commanded_pitch_(0.0)
  , hold_attitude_on_idle_(true)
  , link_num_(0)       // Will be initialized from robot model
  , link_length_(0.5)  // Default value, will be updated from robot model
{
}

void DragonCopilot::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du)
{
  /* initialize the parent class */
  DragonNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  /* Initialize MINCO parameters based on robot model */
  link_num_ = robot_model_->getRotorNum();  // For Dragon, rotor_num equals link_num

  /* Get link_length from transformable robot model */
  auto transformable_model = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(robot_model_);
  if (transformable_model)
  {
    link_length_ = transformable_model->getLinkLength();
    ROS_INFO("[DragonCopilot] Link length retrieved from robot model: %.3f m", link_length_);
  }
  else
  {
    ROS_WARN("[DragonCopilot] Could not cast to TransformableRobotModel, using default link length: %.3f m",
             link_length_);
  }

  ROS_INFO("[DragonCopilot] MINCO trajectory initialized:");
  ROS_INFO("[DragonCopilot] - Trajectory pieces: %d (one per link)", link_num_);

  /* Initialize trajectory visualization publisher */
  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("copilot/trajectory_visualization", 1);
  ROS_INFO("[DragonCopilot] Trajectory visualization publisher initialized");
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

  /* Parse joystick inputs to get velocity and attitude commands */
  RootFrameCommand cmd = parseJoystickInputs(joy_cmd);

  /* Transform velocity commands and set control targets */
  transformAndSetControlTargets(cmd);
}

RootFrameCommand DragonCopilot::parseJoystickInputs(const sensor_msgs::Joy& joy_cmd)
{
  RootFrameCommand cmd;
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
  if (raw_r2 < (1.0 - trigger_deadzone_))
  {
    forward_cmd = (1.0 - raw_r2) / 2.0;  // normalize to [0, 1]
  }

  double backward_cmd = 0.0;
  if (raw_l2 < (1.0 - trigger_deadzone_))
  {
    backward_cmd = (1.0 - raw_l2) / 2.0;  // normalize to [0, 1]
  }

  // Calculate net x velocity command
  // R2 (forward_cmd) -> positive X velocity (root frame forward direction)
  // L2 (backward_cmd) -> negative X velocity (root frame backward direction)
  cmd.x_vel = (forward_cmd - backward_cmd) * max_copilot_x_vel_;

  // ---------- Y-axis control via JOY_AXIS_STICK_RIGHT ----------
  // Right stick horizontal: lateral translation (y-axis)
  double raw_y_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];

  /* Process Y-axis motion (lateral translation) */
  if (fabs(raw_y_cmd) > joy_stick_deadzone_)
  {
    cmd.y_vel = raw_y_cmd * max_copilot_y_vel_;
  }

  // ---------- Z-axis control via JOY_AXIS_STICK_RIGHT ----------
  // Right stick vertical: vertical translation (z-axis)
  double raw_z_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];

  /* Process Z-axis motion (vertical translation) */
  if (fabs(raw_z_cmd) > joy_stick_deadzone_)
  {
    cmd.z_vel = raw_z_cmd * max_copilot_z_vel_;
  }

  // ---------- pitch control via JOY_AXIS_STICK_LEFT ----------
  // Left stick vertical: pitch attitude
  double raw_pitch_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];

  /* Process Pitch attitude control */
  bool has_pitch_input = false;

  if (fabs(raw_pitch_cmd) > joy_stick_deadzone_)
  {
    cmd.pitch = raw_pitch_cmd * max_copilot_pitch_angle_;
    has_pitch_input = true;
  }
  else if (hold_attitude_on_idle_)
  {
    // When no input and attitude hold is enabled, use last commanded pitch
    cmd.pitch = last_commanded_pitch_;
  }
  // else: cmd.pitch remains 0.0 (level flight) from constructor

  // Update last commanded pitch if there was input
  if (has_pitch_input)
  {
    last_commanded_pitch_ = cmd.pitch;
  }

  // ---------- yaw control via JOY_AXIS_STICK_LEFT ----------
  // Left stick horizontal: yaw rotation
  double raw_yaw_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];

  /* Process Yaw motion (rotation around z-axis) */
  if (fabs(raw_yaw_cmd) > joy_stick_deadzone_)
  {
    cmd.yaw_vel = raw_yaw_cmd * max_copilot_yaw_vel_;
  }

  // Finally, revert x and y axis, so that the forward on joystick means forward along the head direction
  cmd.x_vel = -cmd.x_vel;
  cmd.y_vel = -cmd.y_vel;
  cmd.pitch = -cmd.pitch;

  return cmd;
}

void DragonCopilot::transformAndSetControlTargets(const RootFrameCommand& cmd)
{
  /* Transform velocity commands from root frame to CoG velocity targets */
  // - Root frame: the first link of the robot (link1 in Dragon URDF), what the user wants to control
  // - Baselink frame: the FC (flight controller) frame, what the controller uses
  // - CoG: center of gravity, what the controller actually tracks

  // ===== Step 1: Get all necessary frames using KDL and TF =====
  const KDL::JntArray& joint_positions = robot_model_->getJointPositions();

  KDL::Frame world_to_cog;  // CoG frame in world coordinates {}^{world}T_{cog}
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::COG, estimate_mode_),
                                         estimator_->getPos(Frame::COG, estimate_mode_))),
                  world_to_cog);

  KDL::Frame world_to_baselink;
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::BASELINK, estimate_mode_),
                                         estimator_->getPos(Frame::BASELINK, estimate_mode_))),
                  world_to_baselink);

  KDL::Frame root_to_baselink = robot_model_->forwardKinematics<KDL::Frame>("fc", joint_positions);

  KDL::Frame baselink_to_root = root_to_baselink.Inverse();

  KDL::Frame world_to_root = world_to_baselink * baselink_to_root;

  // ===== Step 2: get desired root vel and omega in world frame =====

  KDL::Vector root_vel_body(cmd.x_vel, cmd.y_vel, 0.0);

  KDL::Vector des_root_vel_world = world_to_root.M * root_vel_body;

  KDL::Vector root_omega_body(0.0, 0.0, cmd.yaw_vel);

  KDL::Vector des_root_omega_world = world_to_root.M * root_omega_body;

  // ===== Step 3: get desired CoG vel and omega in world frame =====

  // Position offset from root to CoG in world frame
  KDL::Vector root_to_cog_offset_world = world_to_cog.p - world_to_root.p;

  // Velocity contribution from rotation: omega x r
  KDL::Vector vel_from_rotation = des_root_omega_world * root_to_cog_offset_world;  // KDL Vector cross product operator

  // Final CoG velocity: v_cog = v_root + omega x (r_cog - r_root)
  KDL::Vector des_cog_vel_world = des_root_vel_world + vel_from_rotation;

  // ===== Step 4: Set velocity targets =====

  setTargetVelX(des_cog_vel_world.x());
  setTargetVelY(des_cog_vel_world.y());
  setTargetVelZ(des_cog_vel_world.z() + cmd.z_vel);  // Add world frame z command

  setTargetOmegaZ(des_root_omega_world.z());

  // ===== Step 5: Set pitch attitude target using rotation matrices =====

  // Get current root yaw in world frame
  double root_r, root_p, root_y;
  world_to_root.M.GetRPY(root_r, root_p, root_y);

  // Construct desired root orientation in world frame
  // Keep current yaw, apply commanded pitch, zero roll (level flight)
  KDL::Rotation des_world_to_root_rotation = KDL::Rotation::RPY(0.0, cmd.pitch, root_y);

  // Calculate desired baselink orientation using rotation composition:
  // {}^{world}R_{baselink} = {}^{world}R_{root} * {}^{root}R_{baselink}
  KDL::Rotation des_world_to_baselink_rotation = des_world_to_root_rotation * root_to_baselink.M;

  // Extract RPY from desired baselink rotation
  double des_baselink_r, des_baselink_p, des_baselink_y;
  des_world_to_baselink_rotation.GetRPY(des_baselink_r, des_baselink_p, des_baselink_y);

  // Get current target baselink rotation for smooth transition
  double curr_baselink_r, curr_baselink_p, curr_baselink_y;
  tf::Matrix3x3(curr_target_baselink_rot_).getRPY(curr_baselink_r, curr_baselink_p, curr_baselink_y);

  // Apply smooth pitch change limit
  double pitch_diff = des_baselink_p - curr_baselink_p;
  if (fabs(pitch_diff) > baselink_rot_change_thresh_)
  {
    curr_baselink_p += pitch_diff / fabs(pitch_diff) * baselink_rot_change_thresh_;
  }
  else
  {
    curr_baselink_p = des_baselink_p;
  }

  // Apply smooth roll change limit (though typically small)
  double roll_diff = des_baselink_r - curr_baselink_r;
  if (fabs(roll_diff) > baselink_rot_change_thresh_)
  {
    curr_baselink_r += roll_diff / fabs(roll_diff) * baselink_rot_change_thresh_;
  }
  else
  {
    curr_baselink_r = des_baselink_r;
  }

  // Update final target baselink rotation (yaw controlled by omega, not from attitude)
  final_target_baselink_rot_.setRPY(curr_baselink_r, curr_baselink_p, 0);  // yaw=0 means yaw control via omega_z

  // ===== Step 5.5: Generate MINCO trajectory and compute velocity directions =====
  std::vector<Eigen::Vector3d> link_velocity_directions = copilotPlan(joint_positions);

  // ===== Step 6: Publish joint control commands (minimal placeholder implementation) =====

  // Only publish joint control in HOVER_STATE to avoid conflicts with landing process
  if (getNaviState() == HOVER_STATE)
  {
    sensor_msgs::JointState joint_control_msg;

    // TODO: Implement joint control logic here
    // For now, this is a minimal placeholder that does nothing
    // Example structure (commented out):
    // joint_control_msg.position.push_back(target_joint1_angle);
    // joint_control_msg.position.push_back(target_joint2_angle);
    // ...

    // Uncomment the following line when ready to actually publish joint commands:
    // joint_control_pub_.publish(joint_control_msg);

    // Note: Joint control should be coordinated with pitch attitude control
    // to ensure the robot's physical configuration matches the desired baselink attitude
  }
}

std::vector<Eigen::Vector3d> DragonCopilot::copilotPlan(const KDL::JntArray& joint_positions)
{
  generateMincoTrajectory(joint_positions);

  visualizeTrajectory();

  return computeLinkVel();
}

void DragonCopilot::generateMincoTrajectory(const KDL::JntArray& joint_positions)
{
  std::vector<Eigen::Vector3d> link_waypoints = calculateLinkWaypoints(joint_positions);

  Eigen::Matrix3d headState;
  headState.col(0) = link_waypoints[0];        // Initial position (link1 head)
  headState.col(1) = Eigen::Vector3d::Zero();  // Initial velocity
  headState.col(2) = Eigen::Vector3d::Zero();  // Initial acceleration

  Eigen::Matrix3d tailState;
  tailState.col(0) = link_waypoints[link_num_];  // Final position (last link tail)
  tailState.col(1) = Eigen::Vector3d::Zero();    // Final velocity
  tailState.col(2) = Eigen::Vector3d::Zero();    // Final acceleration

  Eigen::Matrix3Xd intermediate_waypoints(3, link_num_ - 1);
  for (int i = 0; i < link_num_ - 1; i++)
  {
    intermediate_waypoints.col(i) = link_waypoints[i + 1];  // link2, link3, ..., linkN heads
  }

  Eigen::VectorXd time_allocation = Eigen::VectorXd::Ones(link_num_);

  minco_.setConditions(headState, tailState, link_num_);
  minco_.setParameters(intermediate_waypoints, time_allocation);
  minco_.getTrajectory(current_trajectory_);  // the traj is in root frame
}

std::vector<Eigen::Vector3d> DragonCopilot::calculateLinkWaypoints(const KDL::JntArray& joint_positions)
{
  std::vector<Eigen::Vector3d> link_waypoints;
  link_waypoints.reserve(link_num_ + 1);

  // Get world-to-root transformation for converting waypoints to world frame
  KDL::Frame world_to_baselink;
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::BASELINK, estimate_mode_),
                                         estimator_->getPos(Frame::BASELINK, estimate_mode_))),
                  world_to_baselink);

  KDL::Frame root_to_baselink = robot_model_->forwardKinematics<KDL::Frame>("fc", joint_positions);
  KDL::Frame baselink_to_root = root_to_baselink.Inverse();
  KDL::Frame world_to_root = world_to_baselink * baselink_to_root;

  // Get positions for each link head (link1, link2, ..., linkN) in root frame, then convert to world frame
  for (int i = 1; i <= link_num_; i++)
  {
    std::string link_name = "link" + std::to_string(i);
    KDL::Frame link_frame = robot_model_->forwardKinematics<KDL::Frame>(link_name, joint_positions);

    // Transform from root frame to world frame
    KDL::Frame link_frame_world = world_to_root * link_frame;

    Eigen::Vector3d link_pos_world;  // Position in world frame
    link_pos_world << link_frame_world.p.x(), link_frame_world.p.y(), link_frame_world.p.z();
    link_waypoints.push_back(link_pos_world);
  }

  // Get last link frame and orientation
  std::string last_link_name = "link" + std::to_string(link_num_);
  KDL::Frame last_link_frame = robot_model_->forwardKinematics<KDL::Frame>(last_link_name, joint_positions);

  // Calculate tail position: last link head + link_length_ * link_x_direction
  KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);  // x-axis in link frame
  KDL::Vector tail_position_kdl = last_link_frame.p + link_length_ * link_x_direction;

  // Convert tail position from root frame to world frame
  KDL::Frame tail_frame_root;
  tail_frame_root.p = tail_position_kdl;
  tail_frame_root.M = KDL::Rotation::Identity();
  KDL::Frame tail_frame_world = world_to_root * tail_frame_root;

  Eigen::Vector3d tail_pos_world;
  tail_pos_world << tail_frame_world.p.x(), tail_frame_world.p.y(), tail_frame_world.p.z();
  link_waypoints.push_back(tail_pos_world);

  // Log all waypoints (throttled to once per second)
  static double last_waypoint_log_time = 0.0;
  double current_time = ros::Time::now().toSec();
  if (current_time - last_waypoint_log_time >= 1.0)
  {
    ROS_INFO("[DragonCopilot] Link Waypoints (World Frame):");
    for (size_t i = 0; i < link_waypoints.size(); i++)
    {
      if (i < link_waypoints.size() - 1)
      {
        // Link heads
        ROS_INFO("[DragonCopilot]   Link%zu head: [%.3f, %.3f, %.3f]",
                 i + 1,  // Link numbering starts from 1
                 link_waypoints[i].x(), link_waypoints[i].y(), link_waypoints[i].z());
      }
      else
      {
        // Last link tail
        ROS_INFO("[DragonCopilot]   Link%d tail: [%.3f, %.3f, %.3f]", link_num_, link_waypoints[i].x(),
                 link_waypoints[i].y(), link_waypoints[i].z());
      }
    }
    last_waypoint_log_time = current_time;
  }

  return link_waypoints;
}

std::vector<Eigen::Vector3d> DragonCopilot::computeLinkVel()
{
  // Compute velocity directions for all links
  std::vector<Eigen::Vector3d> link_velocity_directions;

  for (int i = 1; i < link_num_; i++)  // Start from link2 (index 1)
  {
    // Time at which this link head appears in trajectory
    // link2 is at t=1.0, link3 is at t=2.0, etc.
    double t = static_cast<double>(i);

    Eigen::Vector3d vel = current_trajectory_.getVel(t);

    Eigen::Vector3d vel_direction = Eigen::Vector3d::Zero();
    double vel_norm = vel.norm();
    if (vel_norm > 1e-6)  // Avoid division by zero
    {
      vel_direction = -vel / vel_norm;  // Negate to get opposite direction
    }

    link_velocity_directions.push_back(vel_direction);
  }

  // Log all velocity directions at once (throttled to once per second)
  static double last_log_time = 0.0;
  double current_time = ros::Time::now().toSec();
  if (current_time - last_log_time >= 1.0)
  {
    ROS_INFO("[DragonCopilot] MINCO Trajectory Link Velocity Directions:");
    for (int i = 0; i < link_velocity_directions.size(); i++)
    {
      // Get velocity norm at this link
      double t = static_cast<double>(i + 1);
      Eigen::Vector3d vel = current_trajectory_.getVel(t);
      double vel_norm = vel.norm();

      ROS_INFO("[DragonCopilot]   Link%d: direction=[%.3f, %.3f, %.3f], norm=%.3f m/s",
               i + 2,  // i+2 because we start from link2
               link_velocity_directions[i].x(), link_velocity_directions[i].y(), link_velocity_directions[i].z(),
               vel_norm);
    }
    last_log_time = current_time;
  }

  return link_velocity_directions;
}

void DragonCopilot::visualizeTrajectory()
{
  ros::Time current_time = ros::Time::now();
  visualization_msgs::MarkerArray marker_array;

  // Create LINE_STRIP marker for trajectory path
  visualization_msgs::Marker trajectory_line;
  trajectory_line.header.frame_id = "world";
  trajectory_line.header.stamp = current_time;
  trajectory_line.ns = "trajectory_path";
  trajectory_line.id = 0;
  trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_line.action = visualization_msgs::Marker::ADD;
  trajectory_line.pose.orientation.w = 1.0;

  // Line appearance
  trajectory_line.scale.x = 0.015;  // Line width (15mm)
  trajectory_line.color.r = 0.0;
  trajectory_line.color.g = 0.5;
  trajectory_line.color.b = 1.0;  // Light blue color
  trajectory_line.color.a = 0.8;

  // Sample the trajectory at high frequency for smooth visualization
  double total_time = static_cast<double>(link_num_);
  double dt = 0.02;  // 50 Hz sampling for smooth line

  for (double t = 0.0; t <= total_time; t += dt)
  {
    Eigen::Vector3d pos = current_trajectory_.getPos(t);
    geometry_msgs::Point p;
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();
    trajectory_line.points.push_back(p);
  }

  marker_array.markers.push_back(trajectory_line);

  // Create velocity direction arrows at each link
  int arrow_id = 2;
  for (int i = 1; i < link_num_; i++)  // Start from link2
  {
    double t = static_cast<double>(i);
    Eigen::Vector3d pos = current_trajectory_.getPos(t);
    Eigen::Vector3d vel = current_trajectory_.getVel(t);

    // Only show arrow if velocity is significant
    if (vel.norm() < 1e-6)
      continue;

    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = "world";
    arrow_marker.header.stamp = current_time;
    arrow_marker.ns = "velocity_arrows";
    arrow_marker.id = arrow_id++;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.orientation.w = 1.0;

    // Arrow appearance
    arrow_marker.scale.x = 0.02;  // Shaft diameter
    arrow_marker.scale.y = 0.04;  // Head diameter
    arrow_marker.scale.z = 0.06;  // Head length
    arrow_marker.color.r = 0.0;
    arrow_marker.color.g = 1.0;  // Green color for velocity arrows
    arrow_marker.color.b = 0.0;
    arrow_marker.color.a = 0.8;

    // Arrow start point (position)
    geometry_msgs::Point start_point;
    start_point.x = pos.x();
    start_point.y = pos.y();
    start_point.z = pos.z();
    arrow_marker.points.push_back(start_point);

    // Arrow end point (position + velocity direction scaled)
    Eigen::Vector3d vel_direction = -vel.normalized() * 0.2;  // 20cm arrows, negated for opposite direction
    geometry_msgs::Point end_point;
    end_point.x = pos.x() + vel_direction.x();
    end_point.y = pos.y() + vel_direction.y();
    end_point.z = pos.z() + vel_direction.z();
    arrow_marker.points.push_back(end_point);

    marker_array.markers.push_back(arrow_marker);
  }

  // Publish all markers at once
  trajectory_viz_pub_.publish(marker_array);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::DragonCopilot, aerial_robot_navigation::BaseNavigator);
