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

  /* Initialize joint_positions_ array with correct size */
  joint_positions_.resize(robot_model_->getJointNum());
  ROS_INFO("[DragonCopilot] Joint positions array initialized with size: %d", robot_model_->getJointNum());

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

  /* Initialize trajectory visualization publisher */
  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("copilot/trajectory_visualization", 1);

  /* Initialize Jacobian computation components */
  const KDL::Tree& tree = robot_model_->getTree();
  jac_solver_.reset(new KDL::TreeJntToJacSolver(tree));
  num_joints_ = tree.getNrOfJoints();

  if (transformable_model)
  {
    link_joint_indices_ = transformable_model->getLinkJointIndices();
    num_link_joints_ = link_joint_indices_.size();
    ROS_INFO("[DragonCopilot] Link joint indices initialized: %d joints", num_link_joints_);
  }
  else
  {
    ROS_ERROR("[DragonCopilot] Failed to get link joint indices from TransformableRobotModel");
  }

  /* Pre-compute all link names */
  link_names_.reserve(link_num_);
  for (int i = 1; i <= link_num_; i++)
  {
    link_names_.push_back("link" + std::to_string(i));
  }
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
  RootFrameCommand root_cmd = parseJoystickInputs(joy_cmd);

  /* Transform velocity commands and set control targets */
  transformAndSetControlTargets(root_cmd);
}

RootFrameCommand DragonCopilot::parseJoystickInputs(const sensor_msgs::Joy& joy_cmd)
{
  RootFrameCommand root_cmd;
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
  root_cmd.x_vel = (forward_cmd - backward_cmd) * max_copilot_x_vel_;

  // ---------- Y-axis control via JOY_AXIS_STICK_RIGHT ----------
  // Right stick horizontal: lateral translation (y-axis)
  double raw_y_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS];

  /* Process Y-axis motion (lateral translation) */
  if (fabs(raw_y_cmd) > joy_stick_deadzone_)
  {
    root_cmd.y_vel = raw_y_cmd * max_copilot_y_vel_;
  }

  // ---------- Z-axis control via JOY_AXIS_STICK_RIGHT ----------
  // Right stick vertical: vertical translation (z-axis)
  double raw_z_cmd = joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS];

  /* Process Z-axis motion (vertical translation) */
  if (fabs(raw_z_cmd) > joy_stick_deadzone_)
  {
    root_cmd.z_vel = raw_z_cmd * max_copilot_z_vel_;
  }

  // ---------- pitch control via JOY_AXIS_STICK_LEFT ----------
  // Left stick vertical: pitch attitude
  double raw_pitch_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];

  /* Process Pitch attitude control */
  bool has_pitch_input = false;

  if (fabs(raw_pitch_cmd) > joy_stick_deadzone_)
  {
    root_cmd.pitch = raw_pitch_cmd * max_copilot_pitch_angle_;
    has_pitch_input = true;
  }
  else if (hold_attitude_on_idle_)
  {
    // When no input and attitude hold is enabled, use last commanded pitch
    root_cmd.pitch = last_commanded_pitch_;
  }
  // else: root_cmd.pitch remains 0.0 (level flight) from constructor

  // Update last commanded pitch if there was input
  if (has_pitch_input)
  {
    last_commanded_pitch_ = root_cmd.pitch;
  }

  // ---------- yaw control via JOY_AXIS_STICK_LEFT ----------
  // Left stick horizontal: yaw rotation
  double raw_yaw_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS];

  /* Process Yaw motion (rotation around z-axis) */
  if (fabs(raw_yaw_cmd) > joy_stick_deadzone_)
  {
    root_cmd.yaw_vel = raw_yaw_cmd * max_copilot_yaw_vel_;
  }

  // Finally, revert x and y axis, so that the forward on joystick means forward along the head direction
  root_cmd.x_vel = -root_cmd.x_vel;
  root_cmd.y_vel = -root_cmd.y_vel;
  root_cmd.pitch = -root_cmd.pitch;

  return root_cmd;
}

void DragonCopilot::transformAndSetControlTargets(const RootFrameCommand& root_cmd)
{
  /* Transform velocity commands from root frame to CoG velocity targets */
  // - Root frame: the first link of the robot (link1 in Dragon URDF), what the user wants to control
  // - Baselink frame: the FC (flight controller) frame, what the controller uses
  // - CoG: center of gravity, what the controller actually tracks

  // ===== Step 1: Update transformation cache (all frames computed once) =====
  updateTransformationCache();

  // ===== Step 2: Compute and set CoG velocity targets =====
  setCoGVelocityTargets(root_cmd);

  // ===== Step 3: Set pitch attitude target =====
  setPitchAttitudeTarget(root_cmd);

  // ===== Step 4: Plan the movement of following links and publish joint control commands =====
  copilotPlan(root_cmd);
}

void DragonCopilot::updateTransformationCache()
{
  // Get current joint positions
  joint_positions_ = robot_model_->getJointPositions();

  // Get CoG frame in world coordinates
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::COG, estimate_mode_),
                                         estimator_->getPos(Frame::COG, estimate_mode_))),
                  world_to_cog_);

  // Get baselink frame in world coordinates
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::BASELINK, estimate_mode_),
                                         estimator_->getPos(Frame::BASELINK, estimate_mode_))),
                  world_to_baselink_);

  // Get root to baselink transform from forward kinematics
  root_to_baselink_ = robot_model_->forwardKinematics<KDL::Frame>("fc", joint_positions_);

  // Calculate inverse transform
  baselink_to_root_ = root_to_baselink_.Inverse();

  // Calculate world to root transform
  world_to_root_ = world_to_baselink_ * baselink_to_root_;

  // Calculate Jacobians and link frames for each link head (link1 to linkN)
  link_jacobians_.clear();
  link_jacobians_.reserve(link_num_);
  link_jacobians_linear_.clear();
  link_jacobians_linear_.reserve(link_num_);
  link_frames_.clear();
  link_frames_.reserve(link_num_);

  // Check if Jacobian solver is initialized
  if (!jac_solver_ || link_joint_indices_.empty())
  {
    ROS_ERROR("[DragonCopilot] Jacobian solver or link joint indices not initialized");
    return;
  }

  for (int i = 0; i < link_num_; i++)
  {
    const std::string& link_name = link_names_[i];

    // Get link frame in root frame coordinates
    KDL::Frame link_frame = robot_model_->forwardKinematics<KDL::Frame>(link_name, joint_positions_);
    link_frames_.push_back(link_frame);

    // Create full Jacobian with appropriate size (6 rows x number of joints)
    KDL::Jacobian full_jacobian(num_joints_);

    // Compute full Jacobian for this link frame with respect to root frame
    int status = jac_solver_->JntToJac(joint_positions_, full_jacobian, link_name);

    if (status < 0)
    {
      ROS_WARN("[DragonCopilot] Failed to compute Jacobian for %s", link_name.c_str());
      continue;
    }

    // Extract only the columns corresponding to the 6 link joints
    // Create a reduced Jacobian with 6 rows (3 linear + 3 angular) and num_link_joints_ columns
    KDL::Jacobian reduced_jacobian(num_link_joints_);

    for (size_t row = 0; row < 6; row++)
    {
      for (size_t col_idx = 0; col_idx < static_cast<size_t>(num_link_joints_); col_idx++)
      {
        int joint_idx = link_joint_indices_[col_idx];
        reduced_jacobian(row, col_idx) = full_jacobian(row, joint_idx);
      }
    }

    link_jacobians_.push_back(reduced_jacobian);

    // Extract and cache the linear part (first 3 rows) for efficiency
    Eigen::MatrixXd linear_jacobian = reduced_jacobian.data.block(0, 0, 3, num_link_joints_);
    link_jacobians_linear_.push_back(linear_jacobian);
  }
}

void DragonCopilot::setCoGVelocityTargets(const RootFrameCommand& root_cmd)
{
  // Transform root frame body velocities to world frame
  KDL::Vector root_vel_body(root_cmd.x_vel, root_cmd.y_vel, 0.0);
  KDL::Vector des_root_vel_world = world_to_root_.M * root_vel_body;

  // Transform root frame angular velocity to world frame
  KDL::Vector root_omega_body(0.0, 0.0, root_cmd.yaw_vel);
  KDL::Vector des_root_omega_world = world_to_root_.M * root_omega_body;

  // Calculate position offset from root to CoG in world frame
  KDL::Vector root_to_cog_offset_world = world_to_cog_.p - world_to_root_.p;

  // Velocity contribution from rotation: omega x r
  KDL::Vector vel_from_rotation = des_root_omega_world * root_to_cog_offset_world;

  // Final CoG velocity: v_cog = v_root + omega x (r_cog - r_root)
  KDL::Vector des_cog_vel_world = des_root_vel_world + vel_from_rotation;

  // Set velocity targets
  setTargetVelX(des_cog_vel_world.x());
  setTargetVelY(des_cog_vel_world.y());
  setTargetVelZ(des_cog_vel_world.z() + root_cmd.z_vel);  // Add world frame z command

  // Set yaw velocity
  setTargetOmegaZ(des_root_omega_world.z());
}

void DragonCopilot::setPitchAttitudeTarget(const RootFrameCommand& root_cmd)
{
  // Get current root yaw in world frame
  double root_r, root_p, root_y;
  world_to_root_.M.GetRPY(root_r, root_p, root_y);

  // Construct desired root orientation in world frame
  // Keep current yaw, apply commanded pitch, zero roll (level flight)
  KDL::Rotation des_world_to_root_rotation = KDL::Rotation::RPY(0.0, root_cmd.pitch, root_y);

  // Calculate desired baselink orientation using rotation composition:
  // {}^{world}R_{baselink} = {}^{world}R_{root} * {}^{root}R_{baselink}
  KDL::Rotation des_world_to_baselink_rotation = des_world_to_root_rotation * root_to_baselink_.M;

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
}

void DragonCopilot::copilotPlan(const RootFrameCommand& root_cmd)
{
  generateMincoTrajectory();

  visualizeTrajectory();

  computeLinkVel();

  computeLinkVelRoot();

  generateJointCommands(root_cmd);
}

void DragonCopilot::generateMincoTrajectory()
{
  std::vector<Eigen::Vector3d> link_waypoints = calculateLinkWaypoints();

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

std::vector<Eigen::Vector3d> DragonCopilot::calculateLinkWaypoints()
{
  std::vector<Eigen::Vector3d> link_waypoints;
  link_waypoints.reserve(link_num_ + 1);

  // Use cached world-to-root transformation (already computed in updateTransformationCache)
  // No need to recalculate world_to_baselink, root_to_baselink, baselink_to_root, world_to_root

  // Get positions for each link head (link1, link2, ..., linkN) in root frame, then convert to world frame
  for (int i = 0; i < link_num_; i++)
  {
    const std::string& link_name = link_names_[i];
    KDL::Frame link_frame = robot_model_->forwardKinematics<KDL::Frame>(link_name, joint_positions_);

    // Transform from root frame to world frame
    KDL::Frame link_frame_world = world_to_root_ * link_frame;

    Eigen::Vector3d link_pos_world;  // Position in world frame
    link_pos_world << link_frame_world.p.x(), link_frame_world.p.y(), link_frame_world.p.z();
    link_waypoints.push_back(link_pos_world);
  }

  // Get last link frame and orientation
  const std::string& last_link_name = link_names_[link_num_ - 1];
  KDL::Frame last_link_frame = robot_model_->forwardKinematics<KDL::Frame>(last_link_name, joint_positions_);

  // Calculate tail position: last link head + link_length_ * link_x_direction
  KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);  // x-axis in link frame
  KDL::Vector tail_position_kdl = last_link_frame.p + link_length_ * link_x_direction;

  // Convert tail position from root frame to world frame
  KDL::Frame tail_frame_root;
  tail_frame_root.p = tail_position_kdl;
  tail_frame_root.M = KDL::Rotation::Identity();
  KDL::Frame tail_frame_world = world_to_root_ * tail_frame_root;

  Eigen::Vector3d tail_pos_world;
  tail_pos_world << tail_frame_world.p.x(), tail_frame_world.p.y(), tail_frame_world.p.z();
  link_waypoints.push_back(tail_pos_world);

  return link_waypoints;
}

void DragonCopilot::computeLinkVel()
{
  // Clear previous velocity directions
  link_vel_directions_.clear();

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

    link_vel_directions_.push_back(vel_direction);
  }
}

void DragonCopilot::computeLinkVelRoot()
{
  // Clear previous velocity directions in root frame
  link_vel_directions_root_.clear();

  // Get the rotation matrix from world to root frame
  // world_to_root_ is already computed in updateTransformationCache()
  KDL::Rotation world_to_root_rotation = world_to_root_.M;
  KDL::Rotation root_to_world_rotation = world_to_root_rotation.Inverse();

  // Transform each velocity direction from world frame to root frame
  for (const auto& vel_world : link_vel_directions_)
  {
    // Convert Eigen::Vector3d to KDL::Vector
    KDL::Vector vel_kdl_world(vel_world.x(), vel_world.y(), vel_world.z());

    // Transform to root frame: v_root = R^T * v_world (inverse rotation)
    KDL::Vector vel_kdl_root = root_to_world_rotation * vel_kdl_world;

    // Convert back to Eigen::Vector3d
    Eigen::Vector3d vel_root(vel_kdl_root.x(), vel_kdl_root.y(), vel_kdl_root.z());

    link_vel_directions_root_.push_back(vel_root);
  }
}

void DragonCopilot::generateJointCommands(const RootFrameCommand& root_cmd)
{
  // Only publish joint control in HOVER_STATE to avoid conflicts with landing process
  if (getNaviState() != HOVER_STATE)
  {
    return;
  }

  // Check if we have valid Jacobians and velocity directions
  if (link_jacobians_.empty() || link_vel_directions_root_.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[DragonCopilot] Jacobians or velocity directions not computed yet");
    return;
  }

  // ========== Step 1: Build constraint matrix A and target vector b ==========
  // We want to minimize ||A * dq - b||^2
  // where dq is the change in joint positions over one control cycle

  // Constraint count:
  // 1. Link1 x-velocity matching root command: 1 equation
  // 2. Link compression constraints (k = 2, 3, ..., N): (N-1) equations
  // 3. Link velocity direction alignment (k = 2, 3, ..., N): 2*(N-1) equations (y and z components)
  // Total: 1 + (N-1) + 2*(N-1) = 3*N - 2 equations

  const int num_constraints = 3 * link_num_ - 2;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, num_link_joints_);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);

  int constraint_idx = 0;

  // ===== Constraint 1: Link1 x-velocity matching root command =====
  // We want: (J_1 * dq)[0] = root_cmd.x_vel * dt
  // where [0] means the x-component (row 0) of the 6D velocity vector
  A.row(constraint_idx) = link_jacobians_[0].data.row(0);  // x-component of link1 velocity
  b(constraint_idx) = root_cmd.x_vel * loop_du_;
  constraint_idx++;

  // ===== Constraint 2: Link compression constraints =====
  // For consecutive links, we want to prevent compression/extension along each link's local x-axis.
  // For link i, its x-axis direction in root frame is: axis_i = R_i * [1, 0, 0]^T
  // where R_i is the rotation matrix of link i.
  // The constraint is: (v_{link_i} - v_{link_{i+1}}) · axis_i = 0
  // where v_{link_i} = J_i * dq (velocity of link i head in root frame)
  //
  // Expanding: (J_i * dq - J_{i+1} * dq) · axis_i = 0
  //           => ((J_i - J_{i+1}) * dq) · axis_i = 0
  //           => axis_i^T * (J_i - J_{i+1}) * dq = 0
  //
  // Since we only need the linear velocity components (first 3 rows of Jacobian):
  //           => axis_i^T * ((J_i - J_{i+1})[0:2, :]) * dq = 0 (shape: (1,3) * (3,num_link_joints_) *
  //           (num_link_joints_,1) = scalar)

  for (int i = 0; i < link_num_ - 1; i++)
  // Compare link i with link i+1 (i=0: link1 vs link2, ..., i=N-2: link{N-1} vs linkN)
  {
    // Get link i's x-axis direction in root frame
    KDL::Vector link_x_axis_kdl = link_frames_[i].M * KDL::Vector(1.0, 0.0, 0.0);
    Eigen::Vector3d link_x_axis(link_x_axis_kdl.x(), link_x_axis_kdl.y(), link_x_axis_kdl.z());

    // Use pre-computed linear velocity Jacobians for consecutive links
    const Eigen::MatrixXd& J_i_linear = link_jacobians_linear_[i];
    const Eigen::MatrixXd& J_iplus1_linear = link_jacobians_linear_[i + 1];

    // Construct the constraint: axis_i^T * (J_i - J_{i+1}) * dq = 0
    Eigen::RowVectorXd constraint_row = link_x_axis.transpose() * (J_i_linear - J_iplus1_linear);

    A.row(constraint_idx) = constraint_row;
    b(constraint_idx) = 0.0;
    constraint_idx++;
  }

  // ===== Constraint 3: Link velocity direction alignment =====
  // For link k (k = 2, 3, ..., N), we want the 3D velocity direction to align with
  // the desired trajectory velocity direction (link_vel_directions_root_).
  // Let d_k = link_vel_directions_root_[k-2] (desired direction for link k, where k=2 corresponds to index 0)
  // Let v_k = J_k * dq be the velocity of link k head
  //
  // To enforce alignment without division, we use the cross-product formulation:
  // If v_k is parallel to d_k, then v_k x d_k = 0
  // This gives us 3 equations, but only 2 are independent (the third is redundant)
  //
  // We use the first 2 components of the cross product:
  // 1. (v_k x d_k)[0] = v_k[1] * d_k[2] - v_k[2] * d_k[1] = 0
  //    => (J_k * dq)[1] * d_k[2] - (J_k * dq)[2] * d_k[1] = 0
  // 2. (v_k x d_k)[1] = v_k[2] * d_k[0] - v_k[0] * d_k[2] = 0
  //    => (J_k * dq)[2] * d_k[0] - (J_k * dq)[0] * d_k[2] = 0
  //
  // Equivalently:
  // 1. d_k[2] * (J_k * dq)[1] - d_k[1] * (J_k * dq)[2] = 0
  // 2. d_k[0] * (J_k * dq)[2] - d_k[2] * (J_k * dq)[0] = 0

  for (int k = 1; k < link_num_; k++)  // k = 1, 2, ..., N-1 (corresponds to link2, link3, ..., linkN)
  {
    const Eigen::Vector3d& desired_dir = link_vel_directions_root_[k - 1];  // link2 -> index 0

    // Use pre-computed linear Jacobian rows for x, y, and z velocity components
    const Eigen::MatrixXd& J_k_linear = link_jacobians_linear_[k];
    Eigen::RowVectorXd J_k_x = J_k_linear.row(0);
    Eigen::RowVectorXd J_k_y = J_k_linear.row(1);
    Eigen::RowVectorXd J_k_z = J_k_linear.row(2);

    // First cross-product constraint: d_k[2] * v_k[1] - d_k[1] * v_k[2] = 0
    A.row(constraint_idx) = desired_dir.z() * J_k_y - desired_dir.y() * J_k_z;
    b(constraint_idx) = 0.0;
    constraint_idx++;

    // Second cross-product constraint: d_k[0] * v_k[2] - d_k[2] * v_k[0] = 0
    A.row(constraint_idx) = desired_dir.x() * J_k_z - desired_dir.z() * J_k_x;
    b(constraint_idx) = 0.0;
    constraint_idx++;
  }

  // ========== Step 2: Solve the least-squares problem ==========
  // We want to find dq that minimizes ||A * dq - b||^2
  // The solution is: dq = (A^T * A)^(-1) * A^T * b = A^+ * b
  // where A^+ is the Moore-Penrose pseudoinverse

  Eigen::VectorXd dq = A.completeOrthogonalDecomposition().solve(b);

  // ========== Step 3: Compute target joint positions ==========
  // target_joint_positions = current_joint_positions + dq
  // Note: joint_positions_ is a KDL::JntArray, we need to convert to Eigen and extract link joints

  Eigen::VectorXd current_link_joint_positions(num_link_joints_);
  for (int i = 0; i < num_link_joints_; i++)
  {
    int joint_idx = link_joint_indices_[i];
    current_link_joint_positions(i) = joint_positions_(joint_idx);
  }

  Eigen::VectorXd target_link_joint_positions = current_link_joint_positions + dq;

  // ========== Step 4: Publish joint command message ==========
  sensor_msgs::JointState joint_control_msg;

  // The target_link_joint_positions vector is already ordered according to link_joint_indices_
  // We just need to populate the joint control message with these target positions
  joint_control_msg.position.resize(num_link_joints_);
  for (int i = 0; i < num_link_joints_; i++)
  {
    joint_control_msg.position[i] = target_link_joint_positions(i);
  }

  joint_control_pub_.publish(joint_control_msg);

  // Debug output (throttled to avoid spamming)
  ROS_DEBUG_THROTTLE(1.0, "[DragonCopilot] Published joint commands: dq_norm=%.4f", dq.norm());
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
