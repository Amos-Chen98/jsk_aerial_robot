// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <dragon/dragon_navigation.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <std_msgs/Empty.h>

namespace aerial_robot_navigation
{
/**
 * @brief Structure to hold control commands in root frame
 */
struct RootFrameCommand
{
  double x_vel;      // Forward(+)/backward(-) velocity in root frame body coordinates [m/s]
  double y_vel;      // Left(+)/right(-) velocity in root frame body coordinates [m/s]
  double z_vel;      // Up(+)/down(-) velocity in world frame [m/s]
  double yaw_vel;    // Yaw angular velocity (counter-clockwise: +) [rad/s]
  double pitch_vel;  // Pitch angular velocity (nose up: +, nose down: -) [rad/s]

  RootFrameCommand() : x_vel(0.0), y_vel(0.0), z_vel(0.0), yaw_vel(0.0), pitch_vel(0.0)
  {
  }
};

/**
 * @brief Structure to hold a trajectory point with position and timestamp
 */
struct TrajectoryPoint
{
  Eigen::Vector3d position;  // Position in world frame [m]
  double timestamp;          // Time when this point was recorded [s]

  TrajectoryPoint() : position(Eigen::Vector3d::Zero()), timestamp(0.0)
  {
  }
  TrajectoryPoint(const Eigen::Vector3d& pos, double t) : position(pos), timestamp(t)
  {
  }
};

class DragonCopilot : public DragonNavigator
{
public:
  DragonCopilot();
  ~DragonCopilot()
  {
  }

  void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, double loop_du) override;

private:
  /**
   * @brief Override joystick control with custom mapping for copilot mode
   *
   * Custom mapping (works with any supported controller via joyParse):
   * - R2 trigger (JOY_AXIS_BUTTON_REAR_RIGHT_2): Forward movement along x-axis
   * - L2 trigger (JOY_AXIS_BUTTON_REAR_LEFT_2): Backward movement along x-axis
   * - Left stick horizontal (JOY_AXIS_STICK_LEFT_LEFTWARDS): Yaw rotation
   * - Left stick vertical (JOY_AXIS_STICK_LEFT_UPWARDS): Pitch attitude control
   * - Right stick horizontal (JOY_AXIS_STICK_RIGHT_LEFTWARDS): Lateral translation (left/right)
   * - Right stick vertical (JOY_AXIS_STICK_RIGHT_UPWARDS): Vertical translation (up/down)
   *
   * Supported controllers: PS3, PS4, Bluetooth (BLT), ROG1
   */
  void joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg) override;

  void rosParamInit() override;

  /**
   * @brief Parse joystick inputs and generate velocity/attitude commands
   *
   * Reads joystick axes and converts them to velocity and attitude commands in root frame.
   * Handles trigger initialization, deadzone processing, and command scaling.
   *
   * @param joy_cmd Parsed joystick message
   * @return RootFrameCommand structure containing all control commands
   */
  RootFrameCommand parseJoystickInputs(const sensor_msgs::Joy& joy_cmd);

  /**
   * @brief Transform velocity commands from root frame to world frame and set control targets
   *
   * This method takes joystick commands in root frame, performs coordinate transformations
   * considering root frame orientation and position offset from CoG, and sets the target
   * velocities and attitude for the control system.
   *
   * @param root_cmd Root frame command structure containing velocity and attitude commands
   */
  void transformAndSetControlTargets(const RootFrameCommand& root_cmd);

  /* ===== Copilot Control Parameters ===== */
  double max_copilot_x_vel_;    // maximum forward/backward velocity
  double max_copilot_y_vel_;    // maximum lateral velocity
  double max_copilot_z_vel_;    // maximum vertical velocity
  double max_copilot_rot_vel_;  // maximum rotation angular velocity for pitch and yaw [rad/s]
  double trigger_deadzone_;     // deadzone for L2/R2 triggers

  /* ===== Joystick State Tracking ===== */
  bool r2_trigger_initialized_;  // true after R2 has been pressed at least once
  bool l2_trigger_initialized_;  // true after L2 has been pressed at least once
  double root_pitch_cmd_;        // commanded root pitch angle (legacy, kept for compatibility)
  double root_yaw_cmd_;          // commanded root yaw angle (legacy, kept for compatibility)
  bool hold_attitude_on_idle_;   // flag to enable attitude hold when no input (default: true)

  /* ===== Joint1 Control via dq ===== */
  Eigen::Vector2d joint1_dq_;  // joint1 velocity increment [dq_pitch, dq_yaw] [rad]

  /* ===== Robot Model Parameters (initialized once) ===== */
  int link_num_;                                         // Number of robot links (equals rotor number)
  double link_length_;                                   // Length of each link segment [m]
  std::vector<double> link_joint_lower_limits_;          // Joint angle lower limits [rad] (from URDF)
  std::vector<double> link_joint_upper_limits_;          // Joint angle upper limits [rad] (from URDF)
  int moveable_joint_num_;                               // Number of moveable joints (excluding rotor joints)
  int kdl_tree_joint_num_;                               // Total number of joints in the robot tree
  int link_joint_num_;                                   // Number of link joints (cached size of link_joint_indices_)
  std::vector<int> link_joint_indices_;                  // Indices of the 6 link joints in the full joint array
  std::vector<std::string> link_names_;                  // Pre-computed link names ("link1", "link2", ..., "linkN")
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;  // KDL Jacobian solver

  /* ===== Cached Transformations (updated once per control cycle) ===== */
  KDL::Frame world_to_cog_;          // CoG frame in world coordinates
  KDL::Frame world_to_root_;         // Root frame in world coordinates
  KDL::Frame root_to_baselink_;      // Transform from root (link1) to baselink (FC)
  KDL::Frame baselink_to_root_;      // Transform from baselink to root
  Eigen::Matrix3d R_world_to_root_;  // Rotation matrix from world to root frame
  double baselink_yaw_world_;        // Baselink yaw angle in world frame [rad]
  double baselink_yaw_world_init_;   // Initial baselink yaw angle in world frame [rad] (recorded on first execution)
  bool baselink_yaw_world_init_recorded_;  // Flag to track if initial yaw has been recorded

  /* ===== Cached Joint State ===== */
  KDL::JntArray joint_positions_;  // Current joint positions

  /* ===== Cached Link Frames (in root frame) ===== */
  // link_frames_[i] contains the frame for link{i+1} (i.e., index 0 = link1, index 1 = link2, etc.)
  std::vector<KDL::Frame> link_frames_;

  /* ===== Cached Jacobians ===== */
  // Jacobians map joint velocities to link frame velocities: v_link = J * q_dot
  // Linear parts of Jacobians (first 3 rows: x, y, z velocities)
  // Pre-computed for efficiency to avoid repeated extraction in constraint generation
  // link_jacobians_linear_[i] corresponds to link(i+1)'s head Jacobian
  // link_jacobians_linear_[link_num_] = linkN tail Jacobian
  std::vector<Eigen::MatrixXd> link_jacobians_linear_;

  /* ===== Cached Root Frame Velocities ===== */
  KDL::Vector root_vel_world_;    // Root velocity in world frame [m/s]
  KDL::Vector root_omega_world_;  // Root angular velocity in world frame [rad/s]

  /* ===== Cached Eigen Conversions (from KDL types) ===== */
  Eigen::Vector3d root_pos_world_eigen_;        // Root position in world frame (Eigen version of world_to_root_.p)
  Eigen::Vector3d root_vel_world_eigen_;        // Root velocity in world frame (Eigen version of root_vel_world_)
  Eigen::Vector3d link2_head_pos_world_eigen_;  // Link2 head position in world frame (for trajectory recording)

  /* ===== Snake Following Parameters ===== */
  bool snake_mode_enabled_;              // Flag to enable/disable snake following visualization
  double trajectory_sample_interval_;    // Minimum distance between trajectory samples [m]
  double trajectory_buffer_max_length_;  // Maximum arc length to store in buffer [m]
  double snake_ik_gain_;                 // Gain for IK position error correction
  double snake_max_joint_delta_;         // Maximum joint angle change per iteration [rad] (= max_rot_vel * loop_du)

  /* ===== Snake Following State ===== */
  std::deque<TrajectoryPoint> trajectory_buffer_;  // Buffer storing recent trajectory points
  double total_arc_length_;                        // Total arc length of trajectory in buffer [m]
  Eigen::Vector3d last_recorded_position_;         // Last position added to buffer
  bool trajectory_initialized_;                    // Flag indicating if buffer has been initialized

  /* ===== Snake Following Cached Values (updated once per control cycle) ===== */
  std::vector<Eigen::Vector3d> snake_target_positions_world_;   // Cached target positions for link tails in world frame
  std::vector<Eigen::Vector3d> snake_current_positions_world_;  // Cached current positions for link tails in world
                                                                // frame
  double joy_joint1_yaw_dq_;  // Cached joint1_yaw delta (after clamping) for yaw rate control [rad]

  /* ===== ROS Publishers ===== */
  ros::Publisher snake_trajectory_viz_pub_;  // Publisher for snake trajectory visualization
  ros::Publisher target_rotation_motion_pub_;  // Publisher for target rotation motion

  /* ===== ROS Subscribers ===== */
  ros::Subscriber reset_trajectory_sub_;  // Subscriber to reset trajectory buffer

  /**
   * @brief Cache frame transformations and link frames
   *
   * Updates joint_positions_, world_to_cog_, world_to_baselink_, root_to_baselink_,
   * baselink_to_root_, and world_to_root_.
   * Also computes forward kinematics for all links and stores their frames in link_frames_.
   */
  void cacheFrameTransforms();

  /**
   * @brief Compute and cache linear Jacobians for all link heads
   *
   * Computes linear Jacobians (first 3 rows) for link1 to linkN heads.
   * Results are stored in link_jacobians_linear_.
   * Requires link_frames_ to be populated first by cacheFrameTransforms().
   */
  void cacheJacobians();

  /**
   * @brief Compute and cache linear Jacobian for the last link's tail position
   *
   * The tail Jacobian is computed from the head Jacobian plus the rotational
   * contribution from the offset vector (head to tail).
   * Result is appended to link_jacobians_linear_.
   */
  void cacheLastLinkTailJacobian();

  /**
   * @brief Cache root frame velocities from command input
   *
   * This method computes and caches the root frame velocities (both linear and angular)
   * in world frame. Should be called once per control cycle after
   * updateTransformationCache() to avoid redundant calculations across multiple functions.
   *
   * Updates:
   * - root_vel_world_: Linear velocity transformed to world frame
   * - root_vel_world_eigen_: Eigen version of root velocity
   * - root_omega_world_: Angular velocity transformed to world frame
   *
   * @param root_cmd Root frame command structure containing velocity commands
   */
  void cacheRootFrameVelocities(const RootFrameCommand& root_cmd);

  /**
   * @brief Compute and set CoG velocity targets from root frame commands
   *
   * This method transforms velocity commands from root frame to CoG velocity targets
   * considering the position offset between root and CoG, as well as the contribution
   * from rotational velocity. It then sets the target velocities for the control system.
   *
   * Formula: v_cog = v_root + omega x (r_cog - r_root)
   *
   * @param root_cmd Root frame command structure containing velocity and attitude commands
   */
  void setCoGVelocityTargets(const RootFrameCommand& root_cmd);

  /**
   * @brief Update baselink attitude target from pitch velocity command
   *
   * This method integrates the pitch velocity command to update the target pitch angle,
   * computes the desired baselink orientation (roll and pitch) from the accumulated pitch,
   * applies smooth transition limits to avoid abrupt attitude changes, and updates the
   * final target baselink rotation for the control system.
   *
   * The transformation follows: {}^{world}R_{baselink} = {}^{world}R_{root} * {}^{root}R_{baselink}
   *
   * @param root_cmd Root frame command structure containing pitch velocity command
   */
  void setBaselinkAttitudeTarget(const RootFrameCommand& root_cmd);

  /**
   * @brief Compute joint1 dq (velocity increment) from joystick input
   *
   * This method computes the dq (velocity increment) for joint1_pitch and joint1_yaw
   * from joystick pitch_vel and yaw_vel commands. The dq values are clamped to respect
   * joint limits. This approach maintains consistency with snake following which also
   * outputs dq commands.
   *
   * @param root_cmd Root frame command structure containing pitch_vel and yaw_vel
   */
  void getJoint1DqFromJoystick(const RootFrameCommand& root_cmd);

  /**
   * @brief Send baselink yaw target to target_rotation_motion topic
   *
   * This method computes the desired baselink yaw based on the logic from
   * setBaselinkAttitudeTarget, sets roll and pitch to 0, and publishes
   * the attitude command to /dragon/target_rotation_motion topic.
   *
   * @param root_cmd Root frame command structure (used for consistency with other methods)
   */
  void sendBaselinkYawTarget(const RootFrameCommand& root_cmd);

  /**
   * @brief Compute and publish all joint commands
   *
   * Unified method that:
   * 1. Computes joint1 angles from snake target position
   * 2. Sets default positions for other joints (0, 90, 0, 90 degrees)
   * 3. Adds joystick compensation (joint1_pitch_dq, joint1_yaw_dq) to joint1
   * 4. Publishes the final desired joint positions
   * 
   * Note: Snake mode joint commands only applied when x_vel is non-zero
   * 
   * @param root_cmd Root frame command structure containing velocity commands
   */
  void computeAndPublishJointCommands(const RootFrameCommand& root_cmd);

  /**
   * @brief Prepare trajectory data: update buffer, check readiness, compute targets
   *
   * This method:
   * 1. Updates trajectory buffer with current link2 head position
   * 2. Checks if trajectory has sufficient arc length
   * 3. Computes current and target link tail positions if ready
   * 4. Visualizes the trajectory
   *
   * @return true if trajectory is ready (sufficient arc length), false otherwise
   */
  bool prepareTrajectoryData();

  /**
   * @brief Compute all joint positions from snake target position
   *
   * This method:
   * 1. Initializes all joints with default values: 0, 90, 0, 90 degrees (for joints 2-5)
   * 2. Gets link2 tail target position from snake_target_positions_world_[0]
   * 3. Transforms it from world frame to root frame to link2 frame
   * 4. Calculates desired joint1_pitch and joint1_yaw from the position in link2 frame
   * 5. Clamps joint1 angles to joint limits
   * 6. Returns complete joint position vector with joint1 from snake computation and others at default
   *
   * @return Eigen::VectorXd containing all joint positions [joint1_pitch, joint1_yaw, joint2_pitch, ...] in radians
   */
  Eigen::VectorXd computeJointAnglesFromSnakeTarget();

  /**
   * @brief Publish joint position commands
   *
   * Computes target joint positions from current positions and dq,
   * then publishes the joint control message.
   *
   * @param dq Joint velocity increments for all joints
   */
  void publishJointCommands(const Eigen::VectorXd& dq);

  /**
   * @brief Clamp joint deltas to maximum allowed change
   *
   * Limits each element of the joint delta vector to be within
   * [-snake_max_joint_delta_, snake_max_joint_delta_].
   *
   * @param dq Input joint velocity increments
   * @return Clamped joint velocity increments
   */
  Eigen::VectorXd clampJointDeltas(const Eigen::VectorXd& dq);

  /* ===== Snake Following Methods ===== */

  /**
   * @brief Update the trajectory history buffer with current link2 head position
   *
   * Records the current link2 head frame position (in world coordinates) to the trajectory buffer.
   * Maintains a fixed-length buffer based on maximum arc length needed.
   * Link2 head is used as the reference point for snake following.
   */
  void updateTrajectoryBuffer();

  /**
   * @brief Find a position along the trajectory at a given arc length from the head
   *
   * @param arc_length The arc length distance from the current head position [m]
   * @return The interpolated position at the given arc length
   */
  Eigen::Vector3d findPositionAtArcLength(double arc_length);

  /**
   * @brief Find a point on trajectory that is exactly target_distance away from a given point
   *
   * This method searches through the trajectory buffer to find a point on the trajectory
   * that satisfies the distance constraint. This is essential for snake-following where
   * each link tail must be exactly link_length away from its head.
   *
   * @param from_point The reference point (link head position) in world frame
   * @param target_distance The required distance from from_point to the trajectory point
   * @return The point on trajectory at the specified distance, or closest match if not found
   */
  Eigen::Vector3d findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point, double target_distance);

  /**
   * @brief Compute target positions for each link tail based on trajectory history
   *
   * Uses the trajectory buffer (which records link2 head positions) to iteratively find
   * where each link tail should be positioned. Starting from link2 head, each link's tail
   * target is found by searching the trajectory for a point at exactly link_length distance
   * from its head (which is the previous link's tail target).
   *
   * This iterative approach ensures:
   * - link2 tail target: at distance link_length from link2 head
   * - link3 tail target: at distance link_length from link2 tail target (= link3 head)
   * - link4 tail target: at distance link_length from link3 tail target (= link4 head)
   *
   * @return Vector of target positions for link2_tail, link3_tail, link4_tail in world frame
   */
  std::vector<Eigen::Vector3d> computeSnakeTargetPositions();

  /**
   * @brief Compute desired joint positions for ALL joints (including joint1) to achieve target link tail positions
   *
   * Directly solves for joint angles geometrically to place link tails at their target positions.
   * Computes positions for all joints starting from joint1 (link2), forming the target robot configuration.
   *
   * @param target_positions Target positions for each link tail (link2, link3, link4) in world frame
   * @return Desired joint positions for all joints based on geometric IK
   */
  Eigen::VectorXd computeSnakeJointPositions(const std::vector<Eigen::Vector3d>& target_positions);

  /**
   * @brief Visualize the trajectory buffer and target positions in RViz
   */
  void visualizeSnakeTrajectory();

  /**
   * @brief Callback to reset the trajectory buffer
   */
  void resetTrajectoryBufferCallback(const std_msgs::EmptyConstPtr& msg);

  /**
   * @brief Compute the current link tail positions in root frame
   * @return Vector of current positions for link1_tail (=link2_head), link2_tail, link3_tail, link4_tail
   */
  std::vector<Eigen::Vector3d> getCurrentLinkTailPositions();

  /**
   * @brief Compute the current link tail positions in world frame
   * @return Vector of current positions for link1_tail (=link2_head), link2_tail, link3_tail, link4_tail in world frame
   */
  std::vector<Eigen::Vector3d> getCurrentLinkTailPositionsWorld();
};
};  // namespace aerial_robot_navigation
