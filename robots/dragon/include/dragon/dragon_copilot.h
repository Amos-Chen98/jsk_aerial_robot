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
#include <dragon/dragon_copilot_control.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>
#include <std_msgs/Empty.h>

namespace aerial_robot_navigation
{
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
  // Stored in struct to pass to DragonCopilotControl
  DragonCopilotControlParams copilot_params_;

  /* ===== Joystick State Tracking ===== */
  bool r2_trigger_initialized_;  // true after R2 has been pressed at least once
  bool l2_trigger_initialized_;  // true after L2 has been pressed at least once
  double root_pitch_cmd_;        // commanded root pitch angle (legacy, kept for compatibility)
  double root_yaw_cmd_;          // commanded root yaw angle (legacy, kept for compatibility)
  // hold_attitude_on_idle_, snake_mode_enabled_ etc moved to copilot_params_

  /* ===== Robot Model Parameters (initialized once) ===== */
  int link_num_;                                 // Number of robot links (equals rotor number)
  std::vector<double> link_joint_lower_limits_;  // Joint angle lower limits [rad] (from URDF)
  std::vector<double> link_joint_upper_limits_;  // Joint angle upper limits [rad] (from URDF)
  int link_joint_num_;                           // Number of link joints (cached size of link_joint_indices_)
  std::vector<int> link_joint_indices_;          // Indices of the 6 link joints in the full joint array
  std::vector<std::string> link_names_;          // Pre-computed link names ("link1", "link2", ..., "linkN")

  /* ===== ROS Publishers ===== */
  ros::Publisher target_rotation_motion_pub_;  // Publisher for target rotation motion
  ros::Publisher snake_trajectory_viz_pub_;

  /* ===== ROS Subscribers ===== */
  ros::Subscriber reset_trajectory_sub_;  // Subscriber to reset trajectory buffer

  /**
   * @brief Publish joint position commands
   *
   * Publishes the desired joint control message.
   *
   * @param desired_positions Target joint positions
   */
  void publishJointCommands(const Eigen::VectorXd& desired_positions);

  /**
   * @brief Callback to reset the trajectory buffer
   */
  void resetTrajectoryBufferCallback(const std_msgs::EmptyConstPtr& msg);

private:
  std::shared_ptr<DragonCopilotControl> copilot_;
};
};  // namespace aerial_robot_navigation
