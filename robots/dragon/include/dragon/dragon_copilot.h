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

namespace aerial_robot_navigation
{
/**
 * @brief Structure to hold control commands in root frame
 */
struct RootFrameCommand
{
  double x_vel;    // Forward(+)/backward(-) velocity in root frame body coordinates [m/s]
  double y_vel;    // Left(+)/right(-) velocity in root frame body coordinates [m/s]
  double z_vel;    // Up(+)/down(-) velocity in world frame [m/s]
  double yaw_vel;  // Yaw angular velocity (counter-clockwise: +) [rad/s]
  double pitch;    // Pitch attitude (nose up: +, nose down: -) [rad]

  RootFrameCommand() : x_vel(0.0), y_vel(0.0), z_vel(0.0), yaw_vel(0.0), pitch(0.0)
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

  /* copilot specific parameters */
  double max_copilot_x_vel_;        // maximum forward/backward velocity
  double max_copilot_y_vel_;        // maximum lateral velocity
  double max_copilot_z_vel_;        // maximum vertical velocity
  double max_copilot_yaw_vel_;      // maximum yaw angular velocity
  double max_copilot_pitch_angle_;  // maximum pitch angle for attitude control

  double trigger_deadzone_;  // deadzone for L2/R2 triggers

  /* trigger initialization tracking */
  bool r2_trigger_initialized_;  // true after R2 has been pressed at least once
  bool l2_trigger_initialized_;  // true after L2 has been pressed at least once

  /* attitude hold when no joystick input */
  double last_commanded_pitch_;  // last pitch angle when there was joystick input
  bool hold_attitude_on_idle_;   // flag to enable attitude hold when no input (default: true)

  /* MINCO trajectory generation */
  minco::MINCO_S3NU minco_;           // MINCO trajectory generator for minimum jerk (s=3)
  Trajectory<5> current_trajectory_;  // Current trajectory being executed

  /* MINCO parameters */
  // For Dragon: piece_num = link_num, total waypoints = link_num + 1
  // Each piece represents one link segment with 1 second duration
  int link_num_;        // Number of robot links (equals rotor number)
  double link_length_;  // Length of each link segment [m]

  /* Visualization */
  ros::Publisher trajectory_viz_pub_;  // Publisher for trajectory visualization (path and markers)

  /* Cached transformations (updated once per control cycle) */
  KDL::Frame world_to_cog_;        // CoG frame in world coordinates
  KDL::Frame world_to_baselink_;   // Baselink frame in world coordinates
  KDL::Frame root_to_baselink_;    // Transform from root (link1) to baselink (FC)
  KDL::Frame baselink_to_root_;    // Transform from baselink to root
  KDL::Frame world_to_root_;       // Root frame in world coordinates
  KDL::JntArray joint_positions_;  // Current joint positions

  /* Computed trajectory velocity directions */
  std::vector<Eigen::Vector3d> link_vel_directions_;  // Velocity directions for each link (from link2) in world frame
  std::vector<Eigen::Vector3d> link_vel_directions_root_;  // Velocity directions for each link (from link2) in root
                                                           // frame

  /* Jacobians for each link frame */
  // Jacobians map joint velocities to link frame velocities: v_link = J * q_dot
  // Each Jacobian has 6 rows (linear xyz + angular xyz) and N columns (number of joints)
  // link_jacobians_[i-1] contains the Jacobian for link{i} (i.e., index 0 = link1, index 1 = link2, etc.)
  std::vector<KDL::Jacobian> link_jacobians_;

  // Linear parts of Jacobians (first 3 rows: x, y, z velocities)
  // Pre-computed for efficiency to avoid repeated extraction in constraint generation
  std::vector<Eigen::MatrixXd> link_jacobians_linear_;

  /* Link frames (orientations) in root frame */
  // Stores the transformation from root frame to each link frame
  // link_frames_[i] contains the frame for link{i+1} (i.e., index 0 = link1, index 1 = link2, etc.)
  std::vector<KDL::Frame> link_frames_;

  /* Cached variables for Jacobian computation (initialized once) */
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;  // KDL Jacobian solver
  std::vector<int> link_joint_indices_;                  // Indices of the 6 link joints in the full joint array
  int num_joints_;                                       // Total number of joints in the robot tree
  int num_link_joints_;                                  // Number of link joints (cached size of link_joint_indices_)
  std::vector<std::string> link_names_;                  // Pre-computed link names ("link1", "link2", ..., "linkN")

  /**
   * @brief Update all cached transformations and Jacobians for current control cycle
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
  void updateTransformationCache();

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
   * @brief Set pitch attitude target with smooth transition
   *
   * This method computes the desired baselink orientation from the commanded root frame pitch,
   * applies smooth transition limits to avoid abrupt attitude changes, and updates the final
   * target baselink rotation for the control system.
   *
   * The transformation follows: {}^{world}R_{baselink} = {}^{world}R_{root} * {}^{root}R_{baselink}
   *
   * @param root_cmd Root frame command structure containing pitch attitude command
   */
  void setPitchAttitudeTarget(const RootFrameCommand& root_cmd);

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
};
};  // namespace aerial_robot_navigation
