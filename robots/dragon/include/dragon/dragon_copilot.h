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
   * @param cmd Root frame command structure containing velocity and attitude commands
   */
  void transformAndSetControlTargets(const RootFrameCommand& cmd);

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
};
};  // namespace aerial_robot_navigation
