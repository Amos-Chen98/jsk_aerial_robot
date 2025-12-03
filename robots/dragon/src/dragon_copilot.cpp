#include <dragon/dragon_copilot.h>
#include <aerial_robot_control/util/joy_parser.h>
#include <tf_conversions/tf_kdl.h>
#include <nav_msgs/Odometry.h>
#include <limits>
#include <cmath>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

DragonCopilot::DragonCopilot()
  : DragonNavigator()
  , r2_trigger_initialized_(false)
  , l2_trigger_initialized_(false)
  , root_pitch_cmd_(0.0)
  , root_yaw_cmd_(0.0)
  , hold_attitude_on_idle_(true)
  , link_num_(0)       // Will be initialized from robot model
  , link_length_(0.5)  // Default value, will be updated from robot model
  , snake_mode_enabled_(true)
  , snake_joint_control_enabled_(false)  // Disabled by default for safety
  , trajectory_sample_interval_(0.01)    // 1cm minimum distance between samples
  , trajectory_buffer_max_length_(3.0)   // Store up to 3m of trajectory
  , snake_ik_gain_(1.0)                  // IK gain
  , snake_max_joint_delta_(0.1)          // Max 0.1 rad per iteration
  , total_arc_length_(0.0)
  , trajectory_initialized_(false)
{
}

void DragonCopilot::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                               boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du)
{
  /* initialize the parent class */
  DragonNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

  /* Initialize Jacobian computation components */
  const KDL::Tree& tree = robot_model_->getTree();
  jac_solver_.reset(new KDL::TreeJntToJacSolver(tree));
  kdl_tree_joint_num_ = tree.getNrOfJoints();  // including fixed joints (rotors)
  moveable_joint_num_ = robot_model_->getJointNum();

  joint_positions_.resize(moveable_joint_num_);

  link_num_ = robot_model_->getRotorNum();  // For Dragon, rotor_num equals link_num

  // output kdl_tree_joint_num_, moveable_joint_num_, link_num_
  ROS_INFO("[DragonCopilot] KDL tree joint num: %d", kdl_tree_joint_num_);
  ROS_INFO("[DragonCopilot] Moveable joint num: %d", moveable_joint_num_);
  ROS_INFO("[DragonCopilot] Link num: %d", link_num_);

  /* Print KDL Jacobian joint names in column order */
  //   ROS_INFO("[DragonCopilot] KDL Jacobian column ordering:");
  //   std::vector<std::string> kdl_joint_names;

  //   // Traverse the tree to get joint names in KDL's internal ordering
  //   // KDL stores joints in the order they appear during tree traversal
  //   std::function<void(const KDL::SegmentMap::const_iterator&)> traverseTree;
  //   traverseTree = [&](const KDL::SegmentMap::const_iterator& segment_it) {
  //     const KDL::Joint& joint = segment_it->second.segment.getJoint();
  //     if (joint.getType() != KDL::Joint::None)
  //     {
  //       kdl_joint_names.push_back(joint.getName());
  //     }

  //     // Recursively traverse children
  //     for (const auto& child : segment_it->second.children)
  //     {
  //       traverseTree(child);
  //     }
  //   };

  //   // Start traversal from root
  //   traverseTree(tree.getRootSegment());

  //   // Print joint names with their column indices
  //   for (size_t i = 0; i < kdl_joint_names.size(); i++)
  //   {
  //     ROS_INFO("[DragonCopilot]   Column %zu: %s", i, kdl_joint_names[i].c_str());
  //   }

  /* Get link_length from transformable robot model */
  auto transformable_model = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(robot_model_);
  if (transformable_model)
  {
    link_length_ = transformable_model->getLinkLength();
    ROS_INFO("[DragonCopilot] Link length: %.3f m", link_length_);

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

  /* Initialize trajectory visualization publisher */
  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("copilot/trajectory_visualization", 1);

  /* Initialize root frame odometry publisher */
  root_frame_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("copilot/root_frame_odom", 1);

  /* Initialize snake following publishers and subscribers */
  snake_trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("copilot/snake_trajectory", 1);
  reset_trajectory_sub_ =
      nh_.subscribe("copilot/reset_trajectory", 1, &DragonCopilot::resetTrajectoryBufferCallback, this);

  /* Initialize cached velocities */
  root_vel_world_ = KDL::Vector::Zero();
  root_omega_world_ = KDL::Vector::Zero();
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
  getParam<double>(navi_nh, "max_pitch_vel", max_copilot_pitch_vel_, 0.5);  // rad/s
  getParam<double>(navi_nh, "trigger_deadzone", trigger_deadzone_, 0.1);
  getParam<bool>(navi_nh, "hold_attitude_on_idle", hold_attitude_on_idle_, true);

  ROS_INFO("[DragonCopilot] Copilot mode initialized with custom joystick mapping");
  ROS_INFO("[DragonCopilot] - Max X velocity: %.2f m/s", max_copilot_x_vel_);
  ROS_INFO("[DragonCopilot] - Max Y velocity: %.2f m/s", max_copilot_y_vel_);
  ROS_INFO("[DragonCopilot] - Max Z velocity: %.2f m/s", max_copilot_z_vel_);
  ROS_INFO("[DragonCopilot] - Max Yaw velocity: %.2f rad/s", max_copilot_yaw_vel_);
  ROS_INFO("[DragonCopilot] - Max Pitch velocity: %.2f rad/s", max_copilot_pitch_vel_);
  ROS_INFO("[DragonCopilot] - Hold attitude on idle: %s", hold_attitude_on_idle_ ? "true" : "false");

  /* Load snake-following parameters */
  getParam<bool>(navi_nh, "snake_mode_enabled", snake_mode_enabled_, true);
  getParam<bool>(navi_nh, "snake_joint_control_enabled", snake_joint_control_enabled_, false);
  getParam<double>(navi_nh, "trajectory_sample_interval", trajectory_sample_interval_, 0.01);
  getParam<double>(navi_nh, "trajectory_buffer_max_length", trajectory_buffer_max_length_, 3.0);
  getParam<double>(navi_nh, "snake_ik_gain", snake_ik_gain_, 1.0);
  getParam<double>(navi_nh, "snake_max_joint_delta", snake_max_joint_delta_, 0.1);

  ROS_INFO("[DragonCopilot] Snake following mode: %s", snake_mode_enabled_ ? "enabled" : "disabled");
  ROS_INFO("[DragonCopilot] Snake joint control: %s", snake_joint_control_enabled_ ? "enabled" : "disabled");
  ROS_INFO("[DragonCopilot] - Trajectory sample interval: %.3f m", trajectory_sample_interval_);
  ROS_INFO("[DragonCopilot] - Trajectory buffer max length: %.2f m", trajectory_buffer_max_length_);
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

  /* Publish root frame odometry (position and velocity in world frame) */
  //   publishRootFrameOdom();
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
  // Left stick vertical: pitch angular velocity
  double raw_pitch_cmd = joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS];

  /* Process Pitch velocity control */
  if (fabs(raw_pitch_cmd) > joy_stick_deadzone_)
  {
    // Active input: apply pitch velocity
    root_cmd.pitch_vel = raw_pitch_cmd * max_copilot_pitch_vel_;
  }
  else if (hold_attitude_on_idle_)
  {
    // When no input and attitude hold is enabled, set pitch_vel to 0 (hold current attitude)
    root_cmd.pitch_vel = 0.0;
  }
  // else: root_cmd.pitch_vel remains 0.0 from constructor (will return to level flight)

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
  root_cmd.pitch_vel = -root_cmd.pitch_vel;

  return root_cmd;
}

void DragonCopilot::transformAndSetControlTargets(const RootFrameCommand& root_cmd)
{
  /* Transform velocity commands from root frame to CoG velocity targets */
  // - Root frame: the first link of the robot (link1 in Dragon URDF), what the user wants to control
  // - Baselink frame: the FC (flight controller) frame, what the controller uses
  // - CoG: center of gravity, what the controller actually tracks

  // ===== Step 1: Cache transformations, positions, and velocities =====
  cacheFrameTransforms();
  cacheJacobians();
  cacheLastLinkTailJacobian();
  cacheRootFrameVelocities(root_cmd);

  // ===== Step 2: Compute and set CoG velocity targets =====
  setCoGVelocityTargets(root_cmd);

  // ===== Step 3: Update baselink attitude target from pitch velocity command =====
  setBaselinkAttitudeTarget(root_cmd);

  // ===== Step 4: Plan the movement of following links and publish joint control commands =====

  //   copilotPlan(root_cmd);

  /* Execute snake-following control if enabled */
  if (snake_mode_enabled_)
  {
    executeSnakeFollowing();
  }
}

void DragonCopilot::cacheFrameTransforms()
{
  // Get current joint positions
  joint_positions_ = robot_model_->getJointPositions();

  // Update robot model with current joint positions to ensure seg_tf_map_ is fresh
  robot_model_->updateRobotModel(joint_positions_);

  // Get CoG frame in world coordinates
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::COG, estimate_mode_),
                                         estimator_->getPos(Frame::COG, estimate_mode_))),
                  world_to_cog_);

  // Get baselink frame in world coordinates
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::BASELINK, estimate_mode_),
                                         estimator_->getPos(Frame::BASELINK, estimate_mode_))),
                  world_to_baselink_);

  const auto& seg_tf_map = robot_model_->getSegmentsTf();

  // Get root to baselink transform from pre-computed segments tf
  // Use getBaselinkName() to dynamically get the baselink frame name from URDF configuration
  root_to_baselink_ = seg_tf_map.at(robot_model_->getBaselinkName());

  // Calculate inverse transform
  baselink_to_root_ = root_to_baselink_.Inverse();

  // Calculate world to root transform
  world_to_root_ = world_to_baselink_ * baselink_to_root_;

  // Cache rotation matrix for other computations (e.g., Jacobian transformation)
  KDL::Rotation world_to_root_rotation = world_to_root_.M;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R_world_to_root_(i, j) = world_to_root_rotation(i, j);
    }
  }

  // Cache Eigen version of root position
  root_pos_world_eigen_ << world_to_root_.p.x(), world_to_root_.p.y(), world_to_root_.p.z();

  // Cache all link frames using pre-computed segments tf
  link_frames_.clear();
  link_frames_.reserve(link_num_);

  for (int i = 0; i < link_num_; i++)
  {
    const std::string& link_name = link_names_[i];
    link_frames_.push_back(seg_tf_map.at(link_name));
  }
}

void DragonCopilot::cacheJacobians()
{
  link_jacobians_linear_.clear();
  link_jacobians_linear_.reserve(link_num_);

  // Check if Jacobian solver is initialized
  if (!jac_solver_ || link_joint_indices_.empty())
  {
    ROS_ERROR("[DragonCopilot] Jacobian solver or link joint indices not initialized");
    return;
  }

  // Compute and store Jacobians for all link heads (link1 to linkN)
  // link_jacobians_linear_[i] corresponds to link(i+1)'s head Jacobian
  // Note: link(i+1)'s head = link(i)'s tail for i >= 1
  for (int i = 0; i < link_num_; i++)
  {
    const std::string& link_name = link_names_[i];

    // Create full Jacobian with appropriate size (6 rows x kdl_tree_joint_num_ columns)
    KDL::Jacobian full_jacobian(kdl_tree_joint_num_);

    // Compute full Jacobian for this link frame with respect to root frame
    int status = jac_solver_->JntToJac(joint_positions_, full_jacobian, link_name);

    if (status < 0)
    {
      ROS_WARN("[DragonCopilot] Failed to compute Jacobian for %s", link_name.c_str());
      continue;
    }

    // Extract only the columns corresponding to the link joints
    KDL::Jacobian reduced_jacobian(link_joint_num_);

    for (int row = 0; row < 6; row++)
    {
      for (int col_idx = 0; col_idx < link_joint_num_; col_idx++)
      {
        int joint_idx = link_joint_indices_[col_idx];
        reduced_jacobian(row, col_idx) = full_jacobian(row, joint_idx);
      }
    }

    // Extract and cache the linear part (first 3 rows) for efficiency
    Eigen::MatrixXd linear_jacobian = reduced_jacobian.data.block(0, 0, 3, link_joint_num_);
    link_jacobians_linear_.push_back(linear_jacobian);
  }
}

void DragonCopilot::cacheLastLinkTailJacobian()
{
  // Compute and append linkN tail Jacobian to the end of link_jacobians_linear_
  // After this: link_jacobians_linear_[link_num_] = linkN tail Jacobian
  // linkN tail position = linkN head position + link_length * linkN x-axis direction
  const std::string& last_link_name = link_names_[link_num_ - 1];
  KDL::Frame last_link_frame = link_frames_[link_num_ - 1];

  // Calculate tail position direction in root frame
  KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);

  // Compute Jacobian at linkN head first
  KDL::Jacobian full_jacobian_last_head(kdl_tree_joint_num_);
  int status = jac_solver_->JntToJac(joint_positions_, full_jacobian_last_head, last_link_name);

  if (status < 0)
  {
    ROS_WARN("[DragonCopilot] Failed to compute Jacobian for %s tail", last_link_name.c_str());
    return;
  }

  // Extract reduced Jacobian for linkN head
  KDL::Jacobian reduced_jacobian_head(link_joint_num_);
  for (int row = 0; row < 6; row++)
  {
    for (int col_idx = 0; col_idx < link_joint_num_; col_idx++)
    {
      int joint_idx = link_joint_indices_[col_idx];
      reduced_jacobian_head(row, col_idx) = full_jacobian_last_head(row, joint_idx);
    }
  }

  // Offset vector in root frame: from linkN head to linkN tail
  KDL::Vector offset_root = link_length_ * link_x_direction;
  Eigen::Vector3d offset_eigen(offset_root.x(), offset_root.y(), offset_root.z());

  // Compute tail Jacobian: J_tail = J_head_linear + [J_head_angular × offset]
  // For each column j (joint j): v_tail_j = v_head_j + omega_j × offset
  KDL::Jacobian reduced_jacobian_tail(link_joint_num_);

  for (int col_idx = 0; col_idx < link_joint_num_; col_idx++)
  {
    // Linear velocity at head
    Eigen::Vector3d v_head(reduced_jacobian_head(0, col_idx), reduced_jacobian_head(1, col_idx),
                           reduced_jacobian_head(2, col_idx));

    // Angular velocity
    Eigen::Vector3d omega(reduced_jacobian_head(3, col_idx), reduced_jacobian_head(4, col_idx),
                          reduced_jacobian_head(5, col_idx));

    // Linear velocity at tail: v_tail = v_head + omega × offset
    Eigen::Vector3d v_tail = v_head + omega.cross(offset_eigen);

    // Fill in the tail Jacobian
    reduced_jacobian_tail(0, col_idx) = v_tail(0);
    reduced_jacobian_tail(1, col_idx) = v_tail(1);
    reduced_jacobian_tail(2, col_idx) = v_tail(2);

    // Angular part remains the same (rigid body)
    reduced_jacobian_tail(3, col_idx) = omega(0);
    reduced_jacobian_tail(4, col_idx) = omega(1);
    reduced_jacobian_tail(5, col_idx) = omega(2);
  }

  // Extract and cache the linear part for linkN tail
  Eigen::MatrixXd linear_jacobian_tail = reduced_jacobian_tail.data.block(0, 0, 3, link_joint_num_);
  link_jacobians_linear_.push_back(linear_jacobian_tail);
}

void DragonCopilot::cacheRootFrameVelocities(const RootFrameCommand& root_cmd)
{
  // Compute root frame body velocities (local variables)
  // Note: x and y velocities are in root body frame, z velocity is in world frame
  KDL::Vector root_vel_body(root_cmd.x_vel, root_cmd.y_vel, 0.0);

  // Transform to world frame
  root_vel_world_ = world_to_root_.M * root_vel_body;

  // Add world frame z velocity component
  root_vel_world_.z(root_vel_world_.z() + root_cmd.z_vel);

  // Cache Eigen version of root velocity in world frame
  root_vel_world_eigen_ << root_vel_world_.x(), root_vel_world_.y(), root_vel_world_.z();

  // Compute root frame angular velocities (local variable)
  // pitch_vel is around y-axis in body frame, yaw_vel is around z-axis
  KDL::Vector root_omega_body(0.0, root_cmd.pitch_vel, root_cmd.yaw_vel);

  // Transform to world frame
  root_omega_world_ = world_to_root_.M * root_omega_body;
}

void DragonCopilot::setCoGVelocityTargets(const RootFrameCommand& root_cmd)
{
  // Use cached root frame velocities (computed in cacheRootFrameVelocities)
  // root_vel_world_ and root_omega_world_ are already available
  // Note: root_vel_world_ already includes the z velocity component

  // Calculate position offset from root to CoG in world frame
  KDL::Vector root_to_cog_offset_world = world_to_cog_.p - world_to_root_.p;

  // Velocity contribution from rotation: omega x r
  KDL::Vector vel_from_rotation = root_omega_world_ * root_to_cog_offset_world;

  // Final CoG velocity: v_cog = v_root + omega x (r_cog - r_root)
  KDL::Vector des_cog_vel_world = root_vel_world_ + vel_from_rotation;

  // Set velocity targets
  setTargetVelX(des_cog_vel_world.x());
  setTargetVelY(des_cog_vel_world.y());
  setTargetVelZ(des_cog_vel_world.z());  // z velocity already included in root_vel_world_

  // Set yaw velocity
  setTargetOmegaZ(root_omega_world_.z());
}

void DragonCopilot::setBaselinkAttitudeTarget(const RootFrameCommand& root_cmd)
{
  // Get current root yaw in world frame
  double root_r, root_p, root_y;
  world_to_root_.M.GetRPY(root_r, root_p, root_y);

  root_pitch_cmd_ = root_p + root_cmd.pitch_vel * loop_du_;
  root_yaw_cmd_ = root_y + root_cmd.yaw_vel * loop_du_;

  const double max_pitch_angle = 1.05;  // ~60 degrees
  root_pitch_cmd_ = std::max(-max_pitch_angle, std::min(max_pitch_angle, root_pitch_cmd_));

  KDL::Rotation des_world_to_root_rotation = KDL::Rotation::RPY(0.0, root_pitch_cmd_, root_yaw_cmd_);

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

  // Compute velocity directions for link tails (starting from link2)
  // link1 motion is determined by external input (root_cmd), so we only constrain:
  // link2 tail = link3 head (at t=2.0)
  // link3 tail = link4 head (at t=3.0)
  // ...
  // linkN tail (at t=N)
  for (int i = 2; i <= link_num_; i++)  // i = 2, 3, ..., N (link2, link3, ..., linkN)
  {
    // Time at which this link tail appears in trajectory
    // link2 tail is at t=2.0, link3 tail is at t=3.0, ..., linkN tail is at t=N
    double t = static_cast<double>(i);

    Eigen::Vector3d vel = current_trajectory_.getVel(t);

    // For the last point (linkN tail), velocity might be zero (end condition of MINCO)
    // In that case, use velocity slightly before the endpoint
    if (vel.norm() < 1e-6 && i == link_num_)
    {
      // Use velocity at t - 0.1 for the endpoint
      vel = current_trajectory_.getVel(t - 0.1);
    }

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
  if (link_jacobians_linear_.empty() || link_vel_directions_root_.empty())
  {
    ROS_WARN_THROTTLE(1.0, "[DragonCopilot] Jacobians or velocity directions not computed yet");
    return;
  }

  // Note: root_vel_world_ and R_world_to_root_ are already cached by cacheRootFrameVelocities()

  // Step 1: Build constraint matrix A and target vector b
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  buildConstraintMatrix(A, b);

  // Step 2: Solve for joint velocities
  Eigen::VectorXd dq = solveJointVelocities(A, b);

  // Step 3: Debug output
  debugPrintMatrixInfo(A, dq);
  debugPrintVelocityInfo(dq, A, b);

  // Step 4: Publish joint commands
  publishJointCommands(dq);
}

void DragonCopilot::buildConstraintMatrix(Eigen::MatrixXd& A, Eigen::VectorXd& b)
{
  // Constraint count:
  // Link tail velocity direction alignment (link2_tail, link3_tail, ..., linkN_tail): 2*(N-1) equations
  // link1's motion is determined by external input, so we only constrain link2 to linkN
  // Each link tail has 2 constraints (x and y cross product components to enforce direction)
  // Total: 2*(N-1) equations

  const int num_constraints = 2 * (link_num_ - 1);
  A = Eigen::MatrixXd::Zero(num_constraints, link_joint_num_);
  b = Eigen::VectorXd::Zero(num_constraints);

  int constraint_idx = 0;

  // DEBUG: Output all Jacobians in root frame
  // link_jacobians_linear_[i] = link(i+1) head Jacobian, link_jacobians_linear_[link_num_] = linkN tail Jacobian
  std::cout << "[DragonCopilot] All Jacobians in root frame:" << std::endl;
  for (size_t k = 0; k < link_jacobians_linear_.size(); k++)
  {
    std::cout << "  Jacobian[" << k << "] (link" << (k + 1) << " head";
    if (k == link_jacobians_linear_.size() - 1)
      std::cout << " / link" << link_num_ << " tail";
    std::cout << "):" << std::endl;
    std::cout << link_jacobians_linear_[k] << std::endl;
  }

  for (int k = 0; k < link_num_ - 1; k++)  // k = 0, 1, ..., N-2 (corresponds to link2, link3, ..., linkN)
  {
    // Get desired velocity direction for link(k+2)'s tail
    // k=0 -> link2_tail (index 0 in link_vel_directions_)
    // k=1 -> link3_tail (index 1 in link_vel_directions_)
    // k=N-2 -> linkN_tail (index N-2 in link_vel_directions_)
    const Eigen::Vector3d& des_vel_eigen = link_vel_directions_[k];  // Desired direction in world frame
    KDL::Vector des_vel(des_vel_eigen(0), des_vel_eigen(1), des_vel_eigen(2));

    // Transform Jacobian from root frame to world frame: Jac_world = R * Jac_root
    // Use link(k+2)'s tail Jacobian (= link(k+3)'s head for k < N-2, or linkN tail for k = N-2):
    // k=0: link2_tail = link3_head, use link_jacobians_linear_[2] (link3 head)
    // k=1: link3_tail = link4_head, use link_jacobians_linear_[3] (link4 head)
    // k=N-2: linkN_tail, use link_jacobians_linear_[link_num_] (linkN tail)
    int jac_idx = (k < link_num_ - 2) ? (k + 2) : link_num_;
    const Eigen::MatrixXd& Jac_root = link_jacobians_linear_[jac_idx];
    Eigen::MatrixXd Jac_world = R_world_to_root_ * Jac_root;

    KDL::Vector v_root_cross_d = root_vel_world_ * des_vel;  // root_vel_world_ × des_vel

    // vel_world = root_vel_world_ + Jac_world * dq
    // We want: (root_vel_world_ + Jac_world * dq) × des_vel = 0
    // => (Jac_world * dq) × des_vel = - (root_vel_world_ × des_vel)
    // => (Jac_world * dq) × des_vel = -v_root_cross_d

    // Cross product (Jac_world * dq) × des_vel:
    // Component [0]: (Jac_world[1,:] * dq) * des_vel[2] - (Jac_world[2,:] * dq) * des_vel[1]
    //              = (des_vel[2] * Jac_world[1,:] - des_vel[1] * Jac_world[2,:]) * dq
    // Component [1]: (Jac_world[2,:] * dq) * des_vel[0] - (Jac_world[0,:] * dq) * des_vel[2]
    //              = (des_vel[0] * Jac_world[2,:] - des_vel[2] * Jac_world[0,:]) * dq

    A.row(constraint_idx) = des_vel.z() * Jac_world.row(1) - des_vel.y() * Jac_world.row(2);
    b(constraint_idx) = -v_root_cross_d.x();
    constraint_idx++;

    A.row(constraint_idx) = des_vel.x() * Jac_world.row(2) - des_vel.z() * Jac_world.row(0);
    b(constraint_idx) = -v_root_cross_d.y();
    constraint_idx++;
  }
}

Eigen::VectorXd DragonCopilot::solveJointVelocities(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
  // Solve the least-squares problem: minimize ||A * dq - b||^2
  // The solution is: dq = (A^T * A)^(-1) * A^T * b = A^+ * b
  // where A^+ is the Moore-Penrose pseudoinverse
  return A.completeOrthogonalDecomposition().solve(b);
}

void DragonCopilot::debugPrintMatrixInfo(const Eigen::MatrixXd& A, const Eigen::VectorXd& dq)
{
  // Print constraint matrix rank and condition number
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd singular_values = svd.singularValues();
  int rank = 0;
  double tolerance = 1e-10;
  for (int i = 0; i < singular_values.size(); i++)
  {
    if (singular_values(i) > tolerance)
      rank++;
  }

  std::cout << "[DragonCopilot] Constraint matrix A: " << A.rows() << "x" << A.cols() << std::endl;
  std::cout << "[DragonCopilot] Matrix rank: " << rank << " (full rank would be " << std::min(A.rows(), A.cols()) << ")"
            << std::endl;
  std::cout << "[DragonCopilot] Singular values: " << singular_values.transpose() << std::endl;

  // Check which joints are actually used in constraints
  std::cout << "[DragonCopilot] Column norms of A (joint influence):" << std::endl;
  for (int col = 0; col < A.cols(); col++)
  {
    double col_norm = A.col(col).norm();
    std::cout << "  Joint " << col << " (idx " << link_joint_indices_[col] << "): " << col_norm << std::endl;
  }

  std::cout << "[DragonCopilot] dq: " << dq.transpose() << std::endl;
}

void DragonCopilot::debugPrintVelocityInfo(const Eigen::VectorXd& dq, const Eigen::MatrixXd& A,
                                           const Eigen::VectorXd& b)
{
  // Root frame control commands
  std::cout << "[DragonCopilot] Root Link velocity (world frame):" << std::endl;
  std::cout << "Root velocity: [" << root_vel_world_.x() << ", " << root_vel_world_.y() << ", " << root_vel_world_.z()
            << "]" << std::endl;

  // MINCO trajectory velocity directions in world frame (for link2 to linkN tails)
  std::cout << "[DragonCopilot] MINCO velocity directions (world frame):" << std::endl;
  for (size_t i = 0; i < link_vel_directions_.size(); i++)
  {
    const Eigen::Vector3d& vel_dir_world = link_vel_directions_[i];
    std::cout << "  Link" << (i + 2) << " tail direction: [" << vel_dir_world.x() << ", " << vel_dir_world.y() << ", "
              << vel_dir_world.z() << "]" << std::endl;
  }

  // Print actual link tail velocities (link2 to linkN)
  std::cout << "[DragonCopilot] Link tail velocities (world frame):" << std::endl;
  for (int i = 1; i < link_num_; i++)  // i = 1, 2, ..., N-1 (link2, link3, ..., linkN)
  {
    // Compute tail velocity: v_tail = v_root + J * dq
    // link(i+1)_tail = link(i+2)_head for i < N-1, or linkN_tail for i = N-1
    // link2_tail (i=1) uses link_jacobians_linear_[2] (link3 head)
    // link3_tail (i=2) uses link_jacobians_linear_[3] (link4 head)
    // linkN_tail (i=N-1) uses link_jacobians_linear_[link_num_] (linkN tail)
    int jac_idx = (i < link_num_ - 1) ? (i + 1) : link_num_;
    Eigen::Vector3d tail_vel_root = link_jacobians_linear_[jac_idx] * dq;
    Eigen::Vector3d tail_vel_world = R_world_to_root_ * tail_vel_root;
    Eigen::Vector3d tail_total_vel_world;
    tail_total_vel_world(0) = root_vel_world_.x() + tail_vel_world(0);
    tail_total_vel_world(1) = root_vel_world_.y() + tail_vel_world(1);
    tail_total_vel_world(2) = root_vel_world_.z() + tail_vel_world(2);
    std::cout << "  Link" << (i + 1) << " tail velocity: [" << tail_total_vel_world(0) << ", "
              << tail_total_vel_world(1) << ", " << tail_total_vel_world(2) << "]" << std::endl;
  }

  // Compute residual: A * dq - b
  Eigen::VectorXd residual = A * dq - b;
  std::cout << "[DragonCopilot] Residual (A*dq - b): " << residual.transpose() << std::endl;
  std::cout << "[DragonCopilot] Residual norm: " << residual.norm() << std::endl;

  std::cout << "----------------------------------------" << std::endl;
  std::cout << std::endl;
}

void DragonCopilot::publishJointCommands(const Eigen::VectorXd& dq)
{
  // Compute target joint positions: target = current + dq
  Eigen::VectorXd current_link_joint_positions(link_joint_num_);
  for (int i = 0; i < link_joint_num_; i++)
  {
    int joint_idx = link_joint_indices_[i];
    current_link_joint_positions(i) = joint_positions_(joint_idx);
  }

  Eigen::VectorXd target_link_joint_positions = current_link_joint_positions + dq;

  // Clamp joint positions to joint limits (from URDF)
  for (int i = 0; i < link_joint_num_; i++)
  {
    target_link_joint_positions(i) =
        std::clamp(target_link_joint_positions(i), link_joint_lower_limits_[i], link_joint_upper_limits_[i]);
  }

  // Publish joint command message
  sensor_msgs::JointState joint_control_msg;
  joint_control_msg.position.resize(link_joint_num_);
  for (int i = 0; i < link_joint_num_; i++)
  {
    joint_control_msg.position[i] = target_link_joint_positions(i);
  }

  joint_control_pub_.publish(joint_control_msg);

  ROS_DEBUG_THROTTLE(1.0, "[DragonCopilot] Published joint commands: dq_norm=%.4f", dq.norm());
}

void DragonCopilot::publishRootFrameOdom()
{
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "world";
  odom_msg.child_frame_id = "root";  // root frame (link1)

  // ===== Position from cached world_to_root_ transformation =====
  odom_msg.pose.pose.position.x = world_to_root_.p.x();
  odom_msg.pose.pose.position.y = world_to_root_.p.y();
  odom_msg.pose.pose.position.z = world_to_root_.p.z();

  // ===== Orientation from cached world_to_root_ transformation =====
  double qx, qy, qz, qw;
  world_to_root_.M.GetQuaternion(qx, qy, qz, qw);
  odom_msg.pose.pose.orientation.x = qx;
  odom_msg.pose.pose.orientation.y = qy;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;

  // ===== Velocity: use cached velocities (assuming zero tracking error) =====
  // Linear velocity in world frame (already computed in cacheRootFrameVelocities)
  odom_msg.twist.twist.linear.x = root_vel_world_.x();
  odom_msg.twist.twist.linear.y = root_vel_world_.y();
  odom_msg.twist.twist.linear.z = root_vel_world_.z();

  // Angular velocity in world frame (already computed in cacheRootFrameVelocities)
  odom_msg.twist.twist.angular.x = root_omega_world_.x();
  odom_msg.twist.twist.angular.y = root_omega_world_.y();
  odom_msg.twist.twist.angular.z = root_omega_world_.z();

  // Publish the odometry message
  root_frame_odom_pub_.publish(odom_msg);
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

  // Create velocity direction arrows at link tail positions
  // Show arrows at: link2_tail (t=2), link3_tail (t=3), ..., linkN_tail (t=N)
  int arrow_id = 2;
  for (int i = 2; i <= link_num_; i++)  // i = 2, 3, ..., N (link2, link3, ..., linkN tails)
  {
    double t = static_cast<double>(i);
    Eigen::Vector3d pos = current_trajectory_.getPos(t);
    Eigen::Vector3d vel = current_trajectory_.getVel(t);

    // For the last point (linkN tail), velocity might be zero (end condition)
    // In that case, use velocity slightly before the endpoint
    if (vel.norm() < 1e-6)
    {
      if (i == link_num_)
      {
        // Use velocity at t - 0.1 for the endpoint
        vel = current_trajectory_.getVel(t - 0.1);
      }

      // Skip if still no significant velocity
      if (vel.norm() < 1e-6)
        continue;
    }

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

/* ===== Snake Following Implementation ===== */

void DragonCopilot::resetTrajectoryBufferCallback(const std_msgs::EmptyConstPtr& msg)
{
  trajectory_buffer_.clear();
  total_arc_length_ = 0.0;
  trajectory_initialized_ = false;
  ROS_INFO("[DragonCopilot] Trajectory buffer reset");
}

void DragonCopilot::updateTrajectoryBuffer()
{
  // Use cached root position in world frame (already updated in cacheFrameTransforms)
  const Eigen::Vector3d& current_position = root_pos_world_eigen_;
  double current_time = ros::Time::now().toSec();

  // Initialize if this is the first point
  if (!trajectory_initialized_)
  {
    trajectory_buffer_.push_front(TrajectoryPoint(current_position, current_time));
    last_recorded_position_ = current_position;
    trajectory_initialized_ = true;
    ROS_INFO("[DragonCopilot] Trajectory buffer initialized at position [%.3f, %.3f, %.3f]", current_position.x(),
             current_position.y(), current_position.z());
    return;
  }

  // Calculate distance from last recorded position
  double distance = (current_position - last_recorded_position_).norm();

  // Only add new point if we've moved enough
  if (distance >= trajectory_sample_interval_)
  {
    // Add new point to the front of the buffer
    trajectory_buffer_.push_front(TrajectoryPoint(current_position, current_time));
    total_arc_length_ += distance;
    last_recorded_position_ = current_position;

    // Remove old points if buffer exceeds maximum arc length
    while (total_arc_length_ > trajectory_buffer_max_length_ && trajectory_buffer_.size() > 2)
    {
      // Calculate arc length of the segment being removed
      size_t last_idx = trajectory_buffer_.size() - 1;
      double segment_length =
          (trajectory_buffer_[last_idx].position - trajectory_buffer_[last_idx - 1].position).norm();
      total_arc_length_ -= segment_length;
      trajectory_buffer_.pop_back();
    }
  }
}

std::vector<Eigen::Vector3d> DragonCopilot::getCurrentLinkTailPositions()
{
  std::vector<Eigen::Vector3d> tail_positions;
  tail_positions.reserve(link_num_);

  // Get each link's tail position in ROOT frame (not world frame)
  // This ensures joint changes only affect relative link positions
  // link_i's tail = link_(i+1)'s head for i < link_num
  // last link's tail = last link head + link_length * link_x_direction

  for (int i = 0; i < link_num_; i++)
  {
    Eigen::Vector3d tail_pos;

    if (i < link_num_ - 1)
    {
      // For link 1 to link(N-1), tail = next link's head
      // link_frames_[i+1] is already in root frame coordinates
      const KDL::Frame& next_link_frame = link_frames_[i + 1];
      tail_pos << next_link_frame.p.x(), next_link_frame.p.y(), next_link_frame.p.z();
    }
    else
    {
      // For last link, tail = head + link_length * x_direction (in root frame)
      const KDL::Frame& last_link_frame = link_frames_[i];
      KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);
      KDL::Vector tail_pos_kdl = last_link_frame.p + link_length_ * link_x_direction;
      tail_pos << tail_pos_kdl.x(), tail_pos_kdl.y(), tail_pos_kdl.z();
    }

    tail_positions.push_back(tail_pos);
  }

  return tail_positions;
}

std::vector<Eigen::Vector3d> DragonCopilot::getCurrentLinkTailPositionsWorld()
{
  std::vector<Eigen::Vector3d> tail_positions_world;
  tail_positions_world.reserve(link_num_ - 1);  // From link2 tail to linkN tail (N-1 tails)

  // Get each link's tail position in WORLD frame, starting from link2
  // This matches the indexing of target_positions in computeSnakeTargetPositions
  // link_i's tail = link_(i+1)'s head for i < link_num
  // last link's tail = last link head + link_length * link_x_direction

  for (int i = 1; i < link_num_; i++)  // Start from link2 (i=1)
  {
    KDL::Vector tail_pos_root;

    if (i < link_num_ - 1)
    {
      // For link 1 to link(N-1), tail = next link's head
      const KDL::Frame& next_link_frame = link_frames_[i + 1];
      tail_pos_root = next_link_frame.p;
    }
    else
    {
      // For last link, tail = head + link_length * x_direction (in root frame)
      const KDL::Frame& last_link_frame = link_frames_[i];
      KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);
      tail_pos_root = last_link_frame.p + link_length_ * link_x_direction;
    }

    // Transform from root frame to world frame
    KDL::Vector tail_pos_world = world_to_root_.M * tail_pos_root + world_to_root_.p;

    Eigen::Vector3d tail_pos;
    tail_pos << tail_pos_world.x(), tail_pos_world.y(), tail_pos_world.z();
    tail_positions_world.push_back(tail_pos);
  }

  return tail_positions_world;
}

Eigen::Vector3d DragonCopilot::findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point,
                                                               double target_distance)
{
  // Find a point on the trajectory that is exactly target_distance away from from_point
  // IMPORTANT: Search from front (newest) to back (oldest), and only consider points
  // that are FURTHER BACK in trajectory history (i.e., older points)

  if (trajectory_buffer_.size() < 2)
  {
    return from_point;  // Fallback
  }

  // First, find which segment the from_point is closest to (or on)
  // This helps us determine where to start searching backward
  size_t start_segment = 0;
  double min_dist_to_trajectory = std::numeric_limits<double>::max();

  for (size_t i = 0; i < trajectory_buffer_.size() - 1; i++)
  {
    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;

    // Find closest point on segment to from_point
    Eigen::Vector3d seg = p2 - p1;
    double seg_len_sq = seg.squaredNorm();
    double t = 0.0;
    if (seg_len_sq > 1e-10)
    {
      t = std::max(0.0, std::min(1.0, (from_point - p1).dot(seg) / seg_len_sq));
    }
    Eigen::Vector3d closest = p1 + t * seg;
    double dist = (from_point - closest).norm();

    if (dist < min_dist_to_trajectory)
    {
      min_dist_to_trajectory = dist;
      start_segment = i;
    }
  }

  // Now search BACKWARD from start_segment (toward older points in trajectory)
  Eigen::Vector3d best_point = trajectory_buffer_.back().position;
  double best_distance_error = std::numeric_limits<double>::max();
  bool found_valid_point = false;

  // Walk backward through trajectory segments (from start_segment to end)
  for (size_t i = start_segment; i < trajectory_buffer_.size() - 1; i++)
  {
    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;

    // For segment p1-p2, find point(s) at distance target_distance from from_point
    // Point on segment: P(t) = p1 + t * (p2 - p1), t in [0, 1]
    // We want: |P(t) - from_point| = target_distance
    // |p1 + t*(p2-p1) - from_point|^2 = target_distance^2

    Eigen::Vector3d d = p2 - p1;          // segment direction
    Eigen::Vector3d f = p1 - from_point;  // vector from from_point to p1

    double a = d.dot(d);
    double b = 2.0 * f.dot(d);
    double c = f.dot(f) - target_distance * target_distance;

    double discriminant = b * b - 4.0 * a * c;

    if (discriminant >= 0 && a > 1e-10)
    {
      double sqrt_disc = std::sqrt(discriminant);

      // Two possible solutions
      double t1 = (-b - sqrt_disc) / (2.0 * a);
      double t2 = (-b + sqrt_disc) / (2.0 * a);

      // Prefer t1 if valid (usually the first intersection along the segment)
      // Check t1 first (smaller t means closer to p1, which is closer to from_point)
      if (t1 >= 0.0 && t1 <= 1.0)
      {
        Eigen::Vector3d candidate = p1 + t1 * d;
        double dist_error = std::abs((candidate - from_point).norm() - target_distance);
        if (dist_error < 1e-6)  // Found exact solution
        {
          return candidate;
        }
        if (dist_error < best_distance_error)
        {
          best_distance_error = dist_error;
          best_point = candidate;
          found_valid_point = true;
        }
      }

      // Check t2 (only if t1 didn't give exact solution)
      if (t2 >= 0.0 && t2 <= 1.0)
      {
        Eigen::Vector3d candidate = p1 + t2 * d;
        double dist_error = std::abs((candidate - from_point).norm() - target_distance);
        if (dist_error < 1e-6)  // Found exact solution
        {
          return candidate;
        }
        if (dist_error < best_distance_error)
        {
          best_distance_error = dist_error;
          best_point = candidate;
          found_valid_point = true;
        }
      }
    }

    // If we found a valid point with good accuracy, we can stop searching
    // (first valid point along backward trajectory is what we want)
    if (found_valid_point && best_distance_error < 0.01)  // 1cm tolerance
    {
      return best_point;
    }

    // Also check segment endpoints as candidates
    double dist_p1 = (p1 - from_point).norm();
    double dist_p2 = (p2 - from_point).norm();

    if (std::abs(dist_p1 - target_distance) < best_distance_error)
    {
      best_distance_error = std::abs(dist_p1 - target_distance);
      best_point = p1;
    }
    if (std::abs(dist_p2 - target_distance) < best_distance_error)
    {
      best_distance_error = std::abs(dist_p2 - target_distance);
      best_point = p2;
    }
  }

  return best_point;
}

std::vector<Eigen::Vector3d> DragonCopilot::computeSnakeTargetPositions()
{
  std::vector<Eigen::Vector3d> target_positions;
  target_positions.reserve(link_num_ - 1);  // For link2_tail, link3_tail, link4_tail (N-1 tails)

  if (trajectory_buffer_.size() < 2)
  {
    return target_positions;
  }

  // Use PREDICTED root frame state after dt to align with root_cmd timing
  // dt is the actual control loop period from the system
  double dt = loop_du_;

  // Predicted root position in world frame: p_root_predicted = p_root_current + v_root * dt
  // Use cached Eigen versions of root position and velocity
  Eigen::Vector3d root_pos_predicted = root_pos_world_eigen_ + root_vel_world_eigen_ * dt;

  // Predicted root orientation using full angular velocity (expressed in world frame)
  double omega_norm = root_omega_world_.Norm();
  KDL::Rotation delta_rot = KDL::Rotation::Identity();
  if (omega_norm > 1e-6)
  {
    KDL::Vector axis = root_omega_world_ / omega_norm;
    delta_rot = KDL::Rotation::Rot(axis, omega_norm * dt);
  }
  // exp([omega] dt) * R_current (left-multiply because omega is in world frame)
  KDL::Rotation predicted_root_rot = delta_rot * world_to_root_.M;

  // Create predicted world_to_root transform
  KDL::Frame world_to_root_predicted;
  world_to_root_predicted.p = KDL::Vector(root_pos_predicted.x(), root_pos_predicted.y(), root_pos_predicted.z());
  world_to_root_predicted.M = predicted_root_rot;

  // Iteratively compute target positions for each link tail
  // The key constraint: each link tail must be exactly link_length_ away from its head
  // link(i) head is fixed, we find link(i) tail on trajectory at distance link_length_ from head

  // Start with link2: its head = link1's tail, computed with PREDICTED root frame
  KDL::Frame link2_head_frame_world =
      world_to_root_predicted * link_frames_[1];  // link_frames_[1] = link2 frame in root
  Eigen::Vector3d current_head_world(link2_head_frame_world.p.x(), link2_head_frame_world.p.y(),
                                     link2_head_frame_world.p.z());

  for (int i = 1; i < link_num_; i++)  // i = 1, 2, 3 for 4-link robot (link2, link3, link4)
  {
    // Find target tail position on trajectory at exactly link_length_ from current head
    Eigen::Vector3d target_tail_world = findPointOnTrajectoryAtDistance(current_head_world, link_length_);

    // Store target in WORLD frame (we'll use world frame Jacobian for IK)
    target_positions.push_back(target_tail_world);

    // For next iteration: the next link's head = current link's tail (target position)
    current_head_world = target_tail_world;
  }

  return target_positions;
}

Eigen::VectorXd DragonCopilot::computeSnakeJointCommands(const std::vector<Eigen::Vector3d>& target_positions_world)
{
  // Initialize joint delta to zero
  Eigen::VectorXd dq = Eigen::VectorXd::Zero(link_joint_num_);

  if (target_positions_world.empty() || link_jacobians_linear_.empty())
  {
    return dq;
  }

  // Use cached current link tail positions in WORLD frame
  const std::vector<Eigen::Vector3d>& current_positions_world = snake_current_positions_world_;

  // Build constraint matrix for least-squares IK in WORLD frame
  // Total link tail velocity in world: v_total = root_vel_world + R_world_to_root * J_root * dq = root_vel_world +
  // J_world * dq
  //
  // We want: pos_current + v_total * dt = pos_target
  // => J_world * dq = (pos_target - pos_current) / dt - root_vel_world
  //
  // For position control with gain: J_world * dq = gain * (pos_target - pos_current) / dt - root_vel_world

  double dt = loop_du_;  // Use actual control loop period
  // Use cached Eigen version of root velocity
  const Eigen::Vector3d& root_vel = root_vel_world_eigen_;

  int num_constraints = 3 * (link_num_ - 1);  // 3 DOF per tail (x, y, z)
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, link_joint_num_);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(num_constraints);

  for (int i = 0; i < link_num_ - 1; i++)  // i = 0, 1, 2 for link2, link3, link4 tails
  {
    // Position error in world frame
    // target_positions_world[i] is target for link(i+2) tail in world frame
    // current_positions_world[i] is current position of link(i+2) tail in world frame (now aligned)
    Eigen::Vector3d pos_error = target_positions_world[i] - current_positions_world[i];

    // Desired velocity to correct the error (scaled by gain) minus root motion contribution
    // This ensures that joint motion compensates for the difference between
    // desired link motion and root frame motion (translation + rotation)
    Eigen::Vector3d omega_world(root_omega_world_.x(), root_omega_world_.y(), root_omega_world_.z());
    Eigen::Vector3d relative_pos = current_positions_world[i] - root_pos_world_eigen_;
    Eigen::Vector3d root_motion = root_vel + omega_world.cross(relative_pos);
    Eigen::Vector3d desired_joint_vel_contribution = snake_ik_gain_ * pos_error / dt - root_motion;

    // Get Jacobian for this link tail and transform to world frame
    // i=0: link2_tail = link3_head, use link_jacobians_linear_[2]
    // i=1: link3_tail = link4_head, use link_jacobians_linear_[3]
    // i=N-2: linkN_tail, use link_jacobians_linear_[link_num_]
    int jac_idx = (i < link_num_ - 2) ? (i + 2) : link_num_;
    if (jac_idx < static_cast<int>(link_jacobians_linear_.size()))
    {
      const Eigen::MatrixXd& J_root = link_jacobians_linear_[jac_idx];
      Eigen::MatrixXd J_world = R_world_to_root_ * J_root;  // Transform to world frame

      // Fill in constraint matrix: J_world * dq = desired_joint_vel_contribution * dt
      A.block(3 * i, 0, 3, link_joint_num_) = J_world;
      b.segment(3 * i, 3) = desired_joint_vel_contribution * dt;
    }
  }

  // Solve least-squares problem with damping for stability
  double damping = 0.01;
  Eigen::MatrixXd AtA = A.transpose() * A + damping * Eigen::MatrixXd::Identity(link_joint_num_, link_joint_num_);
  Eigen::VectorXd Atb = A.transpose() * b;
  dq = AtA.ldlt().solve(Atb);

  // DEBUG: Override dq with fixed values for testing
  dq << 0.0, 0.0, 0.0, 0.0, 0.0, -1.0;

  //   Clamp joint deltas to maximum allowed change
  for (int i = 0; i < link_joint_num_; i++)
  {
    if (dq(i) > snake_max_joint_delta_)
      dq(i) = snake_max_joint_delta_;
    else if (dq(i) < -snake_max_joint_delta_)
      dq(i) = -snake_max_joint_delta_;
  }

  return dq;
}

void DragonCopilot::executeSnakeFollowing()
{
  // Only execute in HOVER_STATE
  if (getNaviState() != HOVER_STATE)
  {
    return;
  }

  // Step 1: Update trajectory buffer with current root position
  updateTrajectoryBuffer();

  // Step 2: Check if trajectory buffer has sufficient arc length for snake following
  // We need at least (link_num_ - 1) * link_length_ arc length to compute valid targets
  // for all link tails (link2, link3, link4 in a 4-link robot)
  double min_required_arc_length = (link_num_)*link_length_;
  bool trajectory_ready = (total_arc_length_ >= min_required_arc_length);

  if (!trajectory_ready)
  {
    ROS_INFO_THROTTLE(1.0, "[DragonCopilot] Waiting for trajectory buffer: %.2f / %.2f m (%.1f%%)", total_arc_length_,
                      min_required_arc_length, 100.0 * total_arc_length_ / min_required_arc_length);
    // Still visualize the trajectory being built, but don't compute or publish joint commands
    visualizeSnakeTrajectory();
    return;
  }

  // Step 3: Cache current link tail positions (in WORLD frame)
  snake_current_positions_world_ = getCurrentLinkTailPositionsWorld();

  // Step 4: Compute and cache target positions for each link tail (in WORLD frame)
  // This uses predicted root frame state to align timing with root_cmd
  snake_target_positions_world_ = computeSnakeTargetPositions();

  // Debug output: print target positions (in world frame)
  //   {
  //     std::stringstream ss_target;
  //     ss_target << "[DragonCopilot] Target positions (world frame): ";
  //     for (size_t i = 0; i < snake_target_positions_world_.size(); i++)
  //     {
  //       ss_target << "Link" << (i + 2) << "=[" << std::fixed << std::setprecision(3)
  //                 << snake_target_positions_world_[i].x() << ", " << snake_target_positions_world_[i].y() << ", "
  //                 << snake_target_positions_world_[i].z() << "] ";
  //     }
  //     ROS_INFO_THROTTLE(1.0, "%s", ss_target.str().c_str());
  //   }

  // Step 5: Compute joint commands using IK (in world frame, considering root motion)
  // Uses cached snake_target_positions_world_ and snake_current_positions_world_
  Eigen::VectorXd dq = computeSnakeJointCommands(snake_target_positions_world_);

  // Debug output: print joint commands
  std::stringstream ss;
  ss << "[DragonCopilot] dq = [";
  for (int i = 0; i < link_joint_num_; i++)
  {
    ss << dq(i);
    if (i < link_joint_num_ - 1)
      ss << ", ";
  }
  ss << "]";
  ROS_INFO_THROTTLE(1.0, "%s", ss.str().c_str());

  // Step 6: Publish joint commands (only if snake_joint_control_enabled_ is true)
  if (snake_joint_control_enabled_)
  {
    publishJointCommands(dq);
  }

  // Step 7: Visualize (uses cached snake_target_positions_world_ and snake_current_positions_world_)
  visualizeSnakeTrajectory();
}

void DragonCopilot::visualizeSnakeTrajectory()
{
  visualization_msgs::MarkerArray marker_array;
  ros::Time current_time = ros::Time::now();

  // Marker 1: Trajectory line (path traced by root) - already in world frame
  visualization_msgs::Marker trajectory_line;
  trajectory_line.header.frame_id = "world";
  trajectory_line.header.stamp = current_time;
  trajectory_line.ns = "snake_trajectory";
  trajectory_line.id = 0;
  trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_line.action = visualization_msgs::Marker::ADD;
  trajectory_line.pose.orientation.w = 1.0;
  trajectory_line.scale.x = 0.02;  // Line width
  trajectory_line.color.r = 1.0;
  trajectory_line.color.g = 0.5;
  trajectory_line.color.b = 0.0;  // Orange color
  trajectory_line.color.a = 0.8;

  for (const auto& point : trajectory_buffer_)
  {
    geometry_msgs::Point p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = point.position.z();
    trajectory_line.points.push_back(p);
  }
  marker_array.markers.push_back(trajectory_line);

  // Marker 2: Target positions for link tails (spheres) - use cached target positions
  for (size_t i = 0; i < snake_target_positions_world_.size(); i++)
  {
    visualization_msgs::Marker target_sphere;
    target_sphere.header.frame_id = "world";
    target_sphere.header.stamp = current_time;
    target_sphere.ns = "snake_targets";
    target_sphere.id = i;
    target_sphere.type = visualization_msgs::Marker::SPHERE;
    target_sphere.action = visualization_msgs::Marker::ADD;
    target_sphere.pose.position.x = snake_target_positions_world_[i].x();
    target_sphere.pose.position.y = snake_target_positions_world_[i].y();
    target_sphere.pose.position.z = snake_target_positions_world_[i].z();
    target_sphere.pose.orientation.w = 1.0;
    target_sphere.scale.x = 0.05;
    target_sphere.scale.y = 0.05;
    target_sphere.scale.z = 0.05;
    target_sphere.color.r = 0.0;
    target_sphere.color.g = 1.0;
    target_sphere.color.b = 0.0;  // Green
    target_sphere.color.a = 0.8;
    marker_array.markers.push_back(target_sphere);
  }

  // Marker 3: Current link tail positions (spheres for comparison) - use cached current positions
  for (size_t i = 0; i < snake_current_positions_world_.size(); i++)  // From link2 tail to linkN tail
  {
    visualization_msgs::Marker current_sphere;
    current_sphere.header.frame_id = "world";
    current_sphere.header.stamp = current_time;
    current_sphere.ns = "snake_current";
    current_sphere.id = i;
    current_sphere.type = visualization_msgs::Marker::SPHERE;
    current_sphere.action = visualization_msgs::Marker::ADD;
    current_sphere.pose.position.x = snake_current_positions_world_[i].x();
    current_sphere.pose.position.y = snake_current_positions_world_[i].y();
    current_sphere.pose.position.z = snake_current_positions_world_[i].z();
    current_sphere.pose.orientation.w = 1.0;
    current_sphere.scale.x = 0.04;
    current_sphere.scale.y = 0.04;
    current_sphere.scale.z = 0.04;
    current_sphere.color.r = 1.0;
    current_sphere.color.g = 0.0;
    current_sphere.color.b = 0.0;  // Red
    current_sphere.color.a = 0.8;
    marker_array.markers.push_back(current_sphere);
  }

  snake_trajectory_viz_pub_.publish(marker_array);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::DragonCopilot, aerial_robot_navigation::BaseNavigator);
