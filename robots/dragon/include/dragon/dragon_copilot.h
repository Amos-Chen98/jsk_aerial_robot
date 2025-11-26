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
#include <aerial_robot_control/minco_trajectory/minco.hpp>
#include <aerial_robot_control/minco_trajectory/trajectory.hpp>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>

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

  /**
   * @brief Generate MINCO trajectory and compute velocity directions for link control
   *
   * This method:
   * 1. Calculates 3D positions of all link heads and the last link tail using forward kinematics
   * 2. Generates a MINCO (Minimum Control) trajectory passing through these waypoints
   * 3. Computes velocity directions at each link head for joint control
   * 4. Visualizes the trajectory in RViz
   *
   * Uses the cached joint_positions_ member variable updated by updateTransformationCache().
   * The generated trajectory is stored in current_trajectory_ for later use in joint control.
   *
   * @param root_cmd Root frame command structure for debugging purposes
   */
  void copilotPlan(const RootFrameCommand& root_cmd);

  /**
   * @brief Compute velocity directions from the generated MINCO trajectory
   *
   * This method computes the velocity direction (normalized velocity vector) at each link head
   * position along the generated trajectory. The directions are useful for determining the
   * desired joint angles to align each link with the trajectory.
   *
   * The computed velocity directions are stored in the class member link_vel_directions_.
   */
  void computeLinkVel();

  /**
   * @brief Compute velocity directions in root frame from world frame velocity directions
   *
   * This method transforms the velocity directions computed by computeLinkVel() from world frame
   * to root frame coordinates. This is useful for joint control since the robot kinematics
   * are typically defined with respect to the root frame.
   *
   * The computed velocity directions in root frame are stored in link_vel_directions_root_.
   * This method should be called after computeLinkVel().
   */
  void computeLinkVelRoot();

  /**
   * @brief Calculate link waypoints from current joint positions
   *
   * This method computes 3D positions of all link heads and the tail position of the last link
   * using forward kinematics. The waypoints are in world frame coordinates.
   *
   * Uses the cached joint_positions_ member variable updated by updateTransformationCache().
   *
   * Waypoints include:
   * - Link1 head, Link2 head, ..., LinkN head
   * - Last link tail (computed as: last link head + link_length * link_x_direction)
   *
   * @return Vector of waypoints (link_num + 1 waypoints total)
   */
  std::vector<Eigen::Vector3d> calculateLinkWaypoints();

  /**
   * @brief Generate MINCO trajectory from link waypoints
   *
   * This method:
   * 1. Calculates link waypoints using calculateLinkWaypoints()
   * 2. Sets up boundary conditions (position, velocity, acceleration)
   * 3. Generates the MINCO trajectory through all waypoints
   *
   * Uses the cached joint_positions_ member variable updated by updateTransformationCache().
   * The generated trajectory is stored in current_trajectory_ member variable.
   */
  void generateMincoTrajectory();

  /**
   * @brief Visualize the MINCO trajectory in RViz
   *
   * This method publishes two visualization messages:
   * 1. A LINE_STRIP marker showing the continuous trajectory path
   * 2. ARROW markers showing velocity directions at each link head
   *
   * The trajectory is sampled at 50 Hz (dt=0.02s) to provide smooth visualization.
   * Velocity arrows are displayed at link positions (starting from link2) to indicate
   * the trajectory direction, with arrows pointing opposite to the velocity for visualization.
   */
  void visualizeTrajectory();

  /* ===== Copilot Control Parameters ===== */
  double max_copilot_x_vel_;       // maximum forward/backward velocity
  double max_copilot_y_vel_;       // maximum lateral velocity
  double max_copilot_z_vel_;       // maximum vertical velocity
  double max_copilot_yaw_vel_;     // maximum yaw angular velocity
  double max_copilot_pitch_vel_;   // maximum pitch angular velocity for attitude control
  double trigger_deadzone_;        // deadzone for L2/R2 triggers

  /* Joystick state tracking */
  bool r2_trigger_initialized_;  // true after R2 has been pressed at least once
  bool l2_trigger_initialized_;  // true after L2 has been pressed at least once
  double last_commanded_pitch_;  // last pitch angle when there was joystick input
  bool hold_attitude_on_idle_;   // flag to enable attitude hold when no input (default: true)

  /* ===== Robot Model Parameters (initialized once) ===== */
  int link_num_;                                         // Number of robot links (equals rotor number)
  double link_length_;                                   // Length of each link segment [m]
  int moveable_joint_num_;                               // Number of moveable joints (excluding rotor joints)
  int kdl_tree_joint_num_;                               // Total number of joints in the robot tree
  int link_joint_num_;                                   // Number of link joints (cached size of link_joint_indices_)
  std::vector<int> link_joint_indices_;                  // Indices of the 6 link joints in the full joint array
  std::vector<std::string> link_names_;                  // Pre-computed link names ("link1", "link2", ..., "linkN")
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;  // KDL Jacobian solver

  /* ===== Cached State (updated once per control cycle) ===== */
  // Transformation frames
  KDL::Frame world_to_cog_;        // CoG frame in world coordinates
  KDL::Frame world_to_baselink_;   // Baselink frame in world coordinates
  KDL::Frame world_to_root_;       // Root frame in world coordinates
  KDL::Frame root_to_baselink_;    // Transform from root (link1) to baselink (FC)
  KDL::Frame baselink_to_root_;    // Transform from baselink to root
  KDL::JntArray joint_positions_;  // Current joint positions

  // Link frames (orientations) in root frame
  // link_frames_[i] contains the frame for link{i+1} (i.e., index 0 = link1, index 1 = link2, etc.)
  std::vector<KDL::Frame> link_frames_;

  // Jacobians for each link frame
  // Jacobians map joint velocities to link frame velocities: v_link = J * q_dot
  // Each Jacobian has 6 rows (linear xyz + angular xyz) and N columns (number of joints)
  // link_jacobians_[i-1] contains the Jacobian for link{i} (i.e., index 0 = link1, index 1 = link2, etc.)
  std::vector<KDL::Jacobian> link_jacobians_;

  // Linear parts of Jacobians (first 3 rows: x, y, z velocities)
  // Pre-computed for efficiency to avoid repeated extraction in constraint generation
  std::vector<Eigen::MatrixXd> link_jacobians_linear_;

  // Root frame velocity and rotation (cached for constraint generation and publishing)
  Eigen::Matrix3d R_world_to_root_;   // Rotation matrix from world to root frame
  KDL::Vector root_vel_body_;         // Root velocity in body frame [m/s]
  KDL::Vector root_vel_world_;        // Root velocity in world frame [m/s]
  KDL::Vector root_omega_body_;       // Root angular velocity in body frame [rad/s]
  KDL::Vector root_omega_world_;      // Root angular velocity in world frame [rad/s]

  /* ===== MINCO Trajectory ===== */
  minco::MINCO_S3NU minco_;           // MINCO trajectory generator for minimum jerk (s=3)
  Trajectory<5> current_trajectory_;  // Current trajectory being executed

  // Computed trajectory velocity directions
  std::vector<Eigen::Vector3d> link_vel_directions_;  // Velocity directions for each link (from link2) in world frame
  std::vector<Eigen::Vector3d> link_vel_directions_root_;  // Velocity directions for each link (from link2) in root
                                                           // frame

  /* ===== ROS Communication ===== */
  ros::Publisher trajectory_viz_pub_;    // Publisher for trajectory visualization (path and markers)
  ros::Publisher root_frame_odom_pub_;   // Publisher for root frame odometry (position and velocity in world frame)

  /**
   * @brief Cache all transformations and Jacobians for current control cycle
   *
   * This method should be called once at the beginning of each control cycle
   * to update all transformation frames and Jacobians. This avoids redundant calculations
   * across multiple functions.
   *
   * Updates:
   * - All transformation frames (world_to_cog_, world_to_baselink_, root_to_baselink_, etc.)
   * - Joint positions cache (joint_positions_)
   * - Jacobians for all link heads (link_jacobians_)
   *
   * The Jacobians computed are with respect to the root frame and map joint velocities
   * to link frame velocities (both linear and angular).
   */
  void cacheTransformations();

  /**
   * @brief Cache frame transformations and link frames
   *
   * Updates joint_positions_, world_to_cog_, world_to_baselink_, root_to_baselink_,
   * baselink_to_root_, and world_to_root_.
   * Also computes forward kinematics for all links and stores their frames in link_frames_.
   */
  void cacheFrameTransforms();

  /**
   * @brief Compute and cache Jacobians for link3 to linkN heads
   *
   * Computes Jacobians for link3 onwards (used for velocity constraints on link2 to linkN-1 tails).
   * Requires link_frames_ to be populated first by cacheLinkFrames().
   */
  void cacheJacobians();

  /**
   * @brief Compute and cache Jacobian for the last link's tail position
   *
   * The tail Jacobian is computed from the head Jacobian plus the rotational
   * contribution from the offset vector (head to tail).
   */
  void cacheLastLinkTailJacobian();

  /**
   * @brief Cache root frame velocities from command input
   *
   * This method computes and caches the root frame velocities (both linear and angular)
   * in both body frame and world frame. Should be called once per control cycle after
   * updateTransformationCache() to avoid redundant calculations across multiple functions.
   *
   * Updates:
   * - root_vel_body_: Linear velocity command in root body frame
   * - root_vel_world_: Linear velocity transformed to world frame
   * - root_omega_body_: Angular velocity command in root body frame
   * - root_omega_world_: Angular velocity transformed to world frame
   * - R_world_to_root_: Rotation matrix cached for other computations
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
   * @brief Generate joint commands from link velocity directions
   *
   * This method converts the velocity directions of each link (computed from the MINCO trajectory)
   * into joint angle commands. Uses the class member link_vel_directions_ computed by computeLinkVel().
   * This is a placeholder implementation that will be replaced with actual joint control logic.
   *
   * @param root_cmd Root frame command structure for debugging purposes
   */
  void generateJointCommands(const RootFrameCommand& root_cmd);

  /**
   * @brief Build constraint matrix for joint velocity optimization
   *
   * Constructs the least-squares constraint matrix A and target vector b to enforce
   * that each link tail moves in the desired direction from the MINCO trajectory.
   * Uses cached R_world_to_root_ and v_root_world_ member variables.
   *
   * @param A Output constraint matrix (2*(N-1) x link_joint_num_)
   * @param b Output target vector (2*(N-1) x 1)
   */
  void buildConstraintMatrix(Eigen::MatrixXd& A, Eigen::VectorXd& b);

  /**
   * @brief Solve least-squares problem for joint velocities
   *
   * Solves the optimization problem: minimize ||A * dq - b||^2
   * using complete orthogonal decomposition for robustness.
   *
   * @param A Constraint matrix
   * @param b Target vector
   * @return dq Joint velocity solution
   */
  Eigen::VectorXd solveJointVelocities(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

  /**
   * @brief Print debug information about constraint matrix
   *
   * Outputs matrix rank, singular values, and per-joint influence to help
   * diagnose optimization problems.
   *
   * @param A Constraint matrix
   * @param dq Computed joint velocities
   */
  void debugPrintMatrixInfo(const Eigen::MatrixXd& A, const Eigen::VectorXd& dq);

  /**
   * @brief Print debug information about velocities and residuals
   *
   * Outputs root velocity commands, MINCO trajectory directions, actual link velocities,
   * and optimization residuals for debugging.
   * Uses cached R_world_to_root_ and v_root_world_ member variables.
   *
   * @param dq Computed joint velocities
   * @param A Constraint matrix
   * @param b Target vector
   */
  void debugPrintVelocityInfo(const Eigen::VectorXd& dq, const Eigen::MatrixXd& A, const Eigen::VectorXd& b);

  /**
   * @brief Publish joint position commands
   *
   * Computes target joint positions from current positions and velocities,
   * then publishes the joint control message.
   *
   * @param dq Joint velocity increments
   */
  void publishJointCommands(const Eigen::VectorXd& dq);

  /**
   * @brief Publish root frame odometry (position and velocity in world frame)
   *
   * This method publishes the root frame's position and velocity in world coordinates
   * as a nav_msgs/Odometry message. The position is obtained from the cached world_to_root_
   * transformation. The velocity uses the cached root_vel_world_ and root_omega_world_.
   */
  void publishRootFrameOdom();
};
};  // namespace aerial_robot_navigation
