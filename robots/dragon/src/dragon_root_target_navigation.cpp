#include <dragon/dragon_root_target_navigation.h>

#include <aerial_robot_control/util/joy_parser.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <cmath>

namespace
{
  double clampAxis(const double value)
  {
    return std::max(-1.0, std::min(1.0, value));
  }

  double clampUnit(const double value)
  {
    return std::max(0.0, std::min(1.0, value));
  }

  double applyDeadzone(const double value, const double deadzone)
  {
    return (std::fabs(value) > deadzone) ? value : 0.0;
  }

  double normalizeBipolarTrigger(const double axis, const int button)
  {
    const double axis_value = clampUnit((1.0 - clampAxis(axis)) * 0.5);
    if (button != 0 && axis_value < 1e-3) return 1.0;
    return axis_value;
  }

  double normalizePs3Trigger(const double axis, const int button)
  {
    if (axis < 0.0) return clampUnit((1.0 - clampAxis(axis)) * 0.5);
    if (axis > 0.0 && axis < 1.0) return clampUnit(axis);
    if (std::fabs(axis - 1.0) < 1e-6) return button != 0 ? 1.0 : 0.0;
    return button != 0 ? 1.0 : 0.0;
  }

  double sanitizeDisabledTriggerRaw(const double corrected_raw, const double epsilon)
  {
    return (std::fabs(corrected_raw - 1.0) <= epsilon) ? corrected_raw : 1.0;
  }

  void suppressBaseNavigatorModeSwitches(sensor_msgs::Joy& joy_msg)
  {
    const size_t axes_size = joy_msg.axes.size();
    const size_t buttons_size = joy_msg.buttons.size();

    if (axes_size == PS3_AXIS_SIZE && buttons_size == PS3_BUTTON_SIZE)
      {
        joy_msg.buttons[JOY_BUTTON_CROSS_DOWN] = 0;
        joy_msg.buttons[JOY_BUTTON_ACTION_TRIANGLE] = 0;
        joy_msg.buttons[JOY_BUTTON_ACTION_CROSS] = 0;
        return;
      }

    if (axes_size == PS4_AXIS_SIZE && buttons_size == PS4_BUTTON_SIZE)
      {
        if(joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] < 0.0) joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] = 0.0;
        joy_msg.buttons[PS4_BUTTON_ACTION_TRIANGLE] = 0;
        joy_msg.buttons[PS4_BUTTON_ACTION_CROSS] = 0;
        return;
      }

    if (axes_size == BLT_AXIS_SIZE && buttons_size == BLT_BUTTON_SIZE)
      {
        if(joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] < 0.0) joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] = 0.0;
        joy_msg.buttons[BLT_BUTTON_ACTION_TRIANGLE] = 0;
        joy_msg.buttons[BLT_BUTTON_ACTION_CROSS] = 0;
        return;
      }

    if (axes_size == ROG1_AXIS_SIZE && buttons_size == ROG1_BUTTON_SIZE)
      {
        if(joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_UP_DOWN] < 0.0) joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_UP_DOWN] = 0.0;
        joy_msg.buttons[ROG1_BUTTON_ACTION_Y] = 0;
        joy_msg.buttons[ROG1_BUTTON_ACTION_A] = 0;
      }
  }
}

namespace aerial_robot_navigation
{
  DragonRootTargetNavigator::DragonRootTargetNavigator():
    DragonNavigator(),
    has_root_tail_pose_(false),
    root_target_initialized_(false),
    root_target_command_active_(false),
    root_target_joint_reset_combo_pressed_(false),
    l2_trigger_initialized_(false),
    r2_trigger_initialized_(false),
    root_target_deadzone_(0.0),
    root_target_trigger_init_epsilon_(0.05),
    joy_reconnect_reset_du_(1.0),
    root_target_forward_gain_(0.45),
    root_target_yaw_gain_(0.40),
    root_target_pitch_gain_(0.35),
    root_target_forward_acc_limit_(0.25),
    root_target_yaw_acc_limit_(0.35),
    root_target_pitch_acc_limit_(0.25),
    root_target_pitch_rate_(0.30),
    root_target_pitch_(0.0),
    root_target_yaw_(0.0),
    current_forward_speed_cmd_(0.0),
    current_yaw_rate_cmd_(0.0),
    current_pitch_rate_cmd_(0.0)
  {
  }

  void DragonRootTargetNavigator::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                             boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                             boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                             double loop_du)
  {
    DragonNavigator::initialize(nh, nhp, robot_model, estimator, loop_du);

    root_tail_pose_sub_ = nh_.subscribe("root/tail_pose", 1, &DragonRootTargetNavigator::rootTailPoseCallback, this);
    root_target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("root/target_pose", 1);
    root_target_joint_reset_pub_ = nh_.advertise<sensor_msgs::JointState>("joints_ctrl", 1);

    ros::NodeHandle navi_nh(nh_, "navigation");
    navi_nh.param("root_target_deadzone", root_target_deadzone_, joy_stick_deadzone_);
    navi_nh.param("root_target_trigger_init_epsilon", root_target_trigger_init_epsilon_, 0.05);
    navi_nh.param("joy_reconnect_reset_du", joy_reconnect_reset_du_, 1.0);
    navi_nh.param("root_target_forward_gain", root_target_forward_gain_, 0.45);
    navi_nh.param("root_target_yaw_gain", root_target_yaw_gain_, 0.40);
    navi_nh.param("root_target_pitch_gain", root_target_pitch_gain_, 0.35);
    navi_nh.param("root_target_forward_acc_limit", root_target_forward_acc_limit_, 0.25);
    navi_nh.param("root_target_yaw_acc_limit", root_target_yaw_acc_limit_, 0.35);
    navi_nh.param("root_target_pitch_acc_limit", root_target_pitch_acc_limit_, 0.25);
    navi_nh.param("root_target_pitch_rate", root_target_pitch_rate_, 0.30);
  }

  void DragonRootTargetNavigator::update()
  {
    DragonNavigator::update();

    geometry_msgs::PoseStamped target_pose;
    bool publish_target_pose = false;
    bool waiting_for_tail_pose = false;

    {
      std::lock_guard<std::mutex> lock(root_target_mutex_);

      if(getNaviState() != HOVER_STATE || !getTeleopFlag())
        {
          clearRootTargetStateLocked();
        }
      else if(!root_target_command_active_)
        {
          // Do not publish or seed a root target until the first valid pose-control input arrives.
        }
      else if(!has_root_tail_pose_)
        {
          clearRootTargetStateLocked();
          waiting_for_tail_pose = true;
        }
      else
        {
          if(!root_target_initialized_) seedRootTargetFromTailPoseLocked();

          root_target_pose_.header.stamp = ros::Time::now();
          root_target_pose_.header.frame_id = latest_root_tail_pose_.header.frame_id;
          target_pose = root_target_pose_;
          publish_target_pose = true;
        }
    }

    if(waiting_for_tail_pose)
      {
        ROS_WARN_THROTTLE(1.0, "[DragonRootTargetNavigator] Waiting for root/tail_pose before publishing root/target_pose.");
      }

    if(publish_target_pose) root_target_pose_pub_.publish(target_pose);
  }

  void DragonRootTargetNavigator::joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg)
  {
    if(handleJointResetCombo(*joy_msg)) return;

    sensor_msgs::Joy sanitized_raw;
    double raw_yaw = 0.0;
    double raw_pitch = 0.0;
    double raw_forward = 0.0;

    if(!normalizeRootTargetJoy(*joy_msg, sanitized_raw, raw_yaw, raw_pitch, raw_forward))
      {
        BaseNavigator::joyStickControl(joy_msg);
        return;
      }

    suppressBaseNavigatorModeSwitches(sanitized_raw);

    sensor_msgs::JoyConstPtr sanitized_msg(new sensor_msgs::Joy(sanitized_raw));
    BaseNavigator::joyStickControl(sanitized_msg);

    if(getNaviState() != HOVER_STATE || !getTeleopFlag())
      {
        std::lock_guard<std::mutex> lock(root_target_mutex_);
        clearRootTargetStateLocked();
        return;
      }

    const double yaw_input = applyDeadzone(raw_yaw, root_target_deadzone_);
    const double pitch_input = applyDeadzone(raw_pitch, root_target_deadzone_);
    const double forward_input = (raw_forward > root_target_deadzone_) ? raw_forward : 0.0;
    const bool has_valid_pose_input = (yaw_input != 0.0) || (pitch_input != 0.0) || (forward_input != 0.0);

    geometry_msgs::PoseStamped target_pose;
    bool publish_target_pose = false;
    bool waiting_for_tail_pose = false;

    {
      std::lock_guard<std::mutex> lock(root_target_mutex_);

      if(!has_root_tail_pose_)
        {
          waiting_for_tail_pose = root_target_command_active_ || has_valid_pose_input;
          clearRootTargetStateLocked();
        }
      else
        {
          if(!root_target_command_active_)
            {
              if(!has_valid_pose_input) return;

              seedRootTargetFromTailPoseLocked();
              root_target_command_active_ = true;
            }

          const ros::Time command_stamp = joy_msg->header.stamp.isZero() ? ros::Time::now() : joy_msg->header.stamp;
          double dt = loop_du_;
          if(!last_root_target_command_stamp_.isZero())
            {
              dt = std::max(0.0, (command_stamp - last_root_target_command_stamp_).toSec());
            }
          dt = std::min(dt, 0.1);
          last_root_target_command_stamp_ = command_stamp;

          const double desired_yaw_rate = shapeSignedInput(yaw_input) * max_teleop_yaw_vel_ * root_target_yaw_gain_;
          const double desired_pitch_rate = shapeSignedInput(pitch_input) * root_target_pitch_rate_ * root_target_pitch_gain_;
          const double desired_forward_speed = shapeUnsignedInput(forward_input) * max_teleop_xy_vel_ * root_target_forward_gain_;

          current_yaw_rate_cmd_ = limitCommandRate(current_yaw_rate_cmd_, desired_yaw_rate, root_target_yaw_acc_limit_, dt);
          current_pitch_rate_cmd_ = limitCommandRate(current_pitch_rate_cmd_, desired_pitch_rate, root_target_pitch_acc_limit_, dt);
          current_forward_speed_cmd_ = limitCommandRate(current_forward_speed_cmd_, desired_forward_speed, root_target_forward_acc_limit_, dt);

          root_target_yaw_ = angles::normalize_angle(root_target_yaw_ + current_yaw_rate_cmd_ * dt);
          root_target_pitch_ = std::clamp(root_target_pitch_ + current_pitch_rate_cmd_ * dt,
                                          -max_teleop_rp_angle_, max_teleop_rp_angle_);

          tf::Quaternion target_orientation = tf::createQuaternionFromRPY(0.0, root_target_pitch_, root_target_yaw_);
          target_orientation.normalize();

          const tf::Vector3 body_x_direction = tf::Matrix3x3(target_orientation) * tf::Vector3(1.0, 0.0, 0.0);
          const double forward_distance = current_forward_speed_cmd_ * dt;

          root_target_pose_.pose.position.x += body_x_direction.x() * forward_distance;
          root_target_pose_.pose.position.y += body_x_direction.y() * forward_distance;
          root_target_pose_.pose.position.z += body_x_direction.z() * forward_distance;
          tf::quaternionTFToMsg(target_orientation, root_target_pose_.pose.orientation);
          root_target_pose_.header.stamp = ros::Time::now();
          root_target_pose_.header.frame_id = latest_root_tail_pose_.header.frame_id;

          target_pose = root_target_pose_;
          publish_target_pose = true;
        }
    }

    if(waiting_for_tail_pose)
      {
        ROS_WARN_THROTTLE(1.0, "[DragonRootTargetNavigator] Ignore joystick pose command until root/tail_pose is available.");
      }

    if(publish_target_pose) root_target_pose_pub_.publish(target_pose);
  }

  bool DragonRootTargetNavigator::handleJointResetCombo(const sensor_msgs::Joy& joy_msg)
  {
    const sensor_msgs::Joy joy_cmd = joyParse(joy_msg);
    if(joy_cmd.buttons.size() <= JOY_BUTTON_ACTION_CROSS)
      {
        root_target_joint_reset_combo_pressed_ = false;
        return false;
      }

    const bool reset_combo_pressed = joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] == 1
      && joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] == 1;
    if(!reset_combo_pressed)
      {
        root_target_joint_reset_combo_pressed_ = false;
        return false;
      }

    if(!root_target_joint_reset_combo_pressed_)
      {
        publishJointResetCommand();
        root_target_joint_reset_combo_pressed_ = true;
      }

    return true;
  }

  void DragonRootTargetNavigator::publishJointResetCommand()
  {
    sensor_msgs::JointState joint_reset_msg;
    joint_reset_msg.header.stamp = ros::Time::now();
    joint_reset_msg.position = {
      0.0,
      M_PI / 2.0,
      0.0,
      M_PI / 2.0,
      0.0,
      M_PI / 2.0
    };

    root_target_joint_reset_pub_.publish(joint_reset_msg);
    ROS_INFO("[DragonRootTargetNavigator] Published joint reset command from joystick combo.");
  }

  bool DragonRootTargetNavigator::normalizeRootTargetJoy(const sensor_msgs::Joy& joy_msg, sensor_msgs::Joy& sanitized_raw,
                                                         double& yaw, double& pitch, double& forward)
  {
    const size_t axes_size = joy_msg.axes.size();
    const size_t buttons_size = joy_msg.buttons.size();

    yaw = 0.0;
    pitch = 0.0;
    forward = 0.0;
    sanitized_raw = joy_msg;

    if(axes_size != PS3_AXIS_SIZE || buttons_size != PS3_BUTTON_SIZE)
      {
        if(axes_size != PS4_AXIS_SIZE || buttons_size != PS4_BUTTON_SIZE)
          {
            if(axes_size != BLT_AXIS_SIZE || buttons_size != BLT_BUTTON_SIZE)
              {
                if(axes_size != ROG1_AXIS_SIZE || buttons_size != ROG1_BUTTON_SIZE)
                  {
                    return false;
                  }
              }
          }
      }

    std::lock_guard<std::mutex> lock(root_target_mutex_);

    const ros::Time current_stamp = joy_msg.header.stamp.isZero() ? ros::Time::now() : joy_msg.header.stamp;
    if(!last_supported_joy_stamp_.isZero() && (current_stamp - last_supported_joy_stamp_).toSec() > joy_reconnect_reset_du_)
      {
        resetTriggerInitializationState();
      }
    last_supported_joy_stamp_ = current_stamp;

    if (axes_size == PS3_AXIS_SIZE && buttons_size == PS3_BUTTON_SIZE)
      {
        const double corrected_l2_raw = correctTriggerRaw(joy_msg.axes[JOY_AXIS_BUTTON_REAR_LEFT_2],
                                                          joy_msg.buttons[JOY_BUTTON_REAR_LEFT_2],
                                                          l2_trigger_initialized_);
        const double corrected_r2_raw = correctTriggerRaw(joy_msg.axes[JOY_AXIS_BUTTON_REAR_RIGHT_2],
                                                          joy_msg.buttons[JOY_BUTTON_REAR_RIGHT_2],
                                                          r2_trigger_initialized_);

        yaw = clampAxis(joy_msg.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS]);
        pitch = -clampAxis(joy_msg.axes[JOY_AXIS_STICK_LEFT_UPWARDS]);
        forward = normalizePs3Trigger(corrected_r2_raw, joy_msg.buttons[JOY_BUTTON_REAR_RIGHT_2]);

        sanitized_raw.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = 0.0;
        sanitized_raw.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = 0.0;
        sanitized_raw.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = 0.0;
        sanitized_raw.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = 0.0;
        sanitized_raw.buttons[JOY_BUTTON_REAR_LEFT_2] = 0;
        sanitized_raw.axes[JOY_AXIS_BUTTON_REAR_LEFT_2] = sanitizeDisabledTriggerRaw(corrected_l2_raw, root_target_trigger_init_epsilon_);
        sanitized_raw.axes[JOY_AXIS_BUTTON_REAR_RIGHT_2] = corrected_r2_raw;
        return true;
      }

    if (axes_size == PS4_AXIS_SIZE && buttons_size == PS4_BUTTON_SIZE)
      {
        const double corrected_l2_raw = correctTriggerRaw(joy_msg.axes[PS4_AXIS_BUTTON_REAR_LEFT_2],
                                                          joy_msg.buttons[PS4_BUTTON_REAR_LEFT_2],
                                                          l2_trigger_initialized_);
        const double corrected_r2_raw = correctTriggerRaw(joy_msg.axes[PS4_AXIS_BUTTON_REAR_RIGHT_2],
                                                          joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_2],
                                                          r2_trigger_initialized_);

        yaw = clampAxis(joy_msg.axes[PS4_AXIS_STICK_LEFT_LEFTWARDS]);
        pitch = -clampAxis(joy_msg.axes[PS4_AXIS_STICK_LEFT_UPWARDS]);
        forward = normalizeBipolarTrigger(corrected_r2_raw, joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_2]);

        sanitized_raw.axes[PS4_AXIS_STICK_LEFT_LEFTWARDS] = 0.0;
        sanitized_raw.axes[PS4_AXIS_STICK_LEFT_UPWARDS] = 0.0;
        sanitized_raw.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS] = 0.0;
        sanitized_raw.axes[PS4_AXIS_STICK_RIGHT_UPWARDS] = 0.0;
        sanitized_raw.buttons[PS4_BUTTON_REAR_LEFT_2] = 0;
        sanitized_raw.axes[PS4_AXIS_BUTTON_REAR_LEFT_2] = sanitizeDisabledTriggerRaw(corrected_l2_raw, root_target_trigger_init_epsilon_);
        sanitized_raw.axes[PS4_AXIS_BUTTON_REAR_RIGHT_2] = corrected_r2_raw;
        return true;
      }

    if (axes_size == BLT_AXIS_SIZE && buttons_size == BLT_BUTTON_SIZE)
      {
        const double corrected_l2_raw = correctTriggerRaw(joy_msg.axes[BLT_AXIS_BUTTON_REAR_LEFT_2],
                                                          joy_msg.buttons[BLT_BUTTON_REAR_LEFT_2],
                                                          l2_trigger_initialized_);
        const double corrected_r2_raw = correctTriggerRaw(joy_msg.axes[BLT_AXIS_BUTTON_REAR_RIGHT_2],
                                                          joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_2],
                                                          r2_trigger_initialized_);

        yaw = clampAxis(joy_msg.axes[BLT_AXIS_STICK_LEFT_LEFTWARDS]);
        pitch = -clampAxis(joy_msg.axes[BLT_AXIS_STICK_LEFT_UPWARDS]);
        forward = normalizeBipolarTrigger(corrected_r2_raw, joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_2]);

        sanitized_raw.axes[BLT_AXIS_STICK_LEFT_LEFTWARDS] = 0.0;
        sanitized_raw.axes[BLT_AXIS_STICK_LEFT_UPWARDS] = 0.0;
        sanitized_raw.axes[BLT_AXIS_STICK_RIGHT_LEFTWARDS] = 0.0;
        sanitized_raw.axes[BLT_AXIS_STICK_RIGHT_UPWARDS] = 0.0;
        sanitized_raw.buttons[BLT_BUTTON_REAR_LEFT_2] = 0;
        sanitized_raw.axes[BLT_AXIS_BUTTON_REAR_LEFT_2] = sanitizeDisabledTriggerRaw(corrected_l2_raw, root_target_trigger_init_epsilon_);
        sanitized_raw.axes[BLT_AXIS_BUTTON_REAR_RIGHT_2] = corrected_r2_raw;
        return true;
      }

    const double corrected_l2_raw = correctTriggerRaw(joy_msg.axes[ROG1_AXIS_BUTTON_REAR_LEFT_2], 0, l2_trigger_initialized_);
    const double corrected_r2_raw = correctTriggerRaw(joy_msg.axes[ROG1_AXIS_BUTTON_REAR_RIGHT_2], 0, r2_trigger_initialized_);

    yaw = clampAxis(joy_msg.axes[ROG1_AXIS_STICK_LEFT_LEFTWARDS]);
    pitch = -clampAxis(joy_msg.axes[ROG1_AXIS_STICK_LEFT_UPWARDS]);
    forward = normalizeBipolarTrigger(corrected_r2_raw, 0);

    sanitized_raw.axes[ROG1_AXIS_STICK_LEFT_LEFTWARDS] = 0.0;
    sanitized_raw.axes[ROG1_AXIS_STICK_LEFT_UPWARDS] = 0.0;
    sanitized_raw.axes[ROG1_AXIS_STICK_RIGHT_LEFTWARDS] = 0.0;
    sanitized_raw.axes[ROG1_AXIS_STICK_RIGHT_UPWARDS] = 0.0;
    sanitized_raw.axes[ROG1_AXIS_BUTTON_REAR_LEFT_2] = sanitizeDisabledTriggerRaw(corrected_l2_raw, root_target_trigger_init_epsilon_);
    sanitized_raw.axes[ROG1_AXIS_BUTTON_REAR_RIGHT_2] = corrected_r2_raw;
    return true;
  }

  double DragonRootTargetNavigator::correctTriggerRaw(double raw_axis, int button, bool& initialized) const
  {
    const double clamped_axis = clampAxis(raw_axis);
    if(!initialized)
      {
        if(button != 0 || std::fabs(clamped_axis) > root_target_trigger_init_epsilon_)
          {
            initialized = true;
          }
        else
          {
            return 1.0;
          }
      }

    return clamped_axis;
  }

  double DragonRootTargetNavigator::shapeUnsignedInput(double value) const
  {
    const double clamped_value = clampUnit(value);
    return clamped_value * clamped_value;
  }

  double DragonRootTargetNavigator::shapeSignedInput(double value) const
  {
    const double clamped_value = clampAxis(value);
    return std::copysign(clamped_value * clamped_value, clamped_value);
  }

  double DragonRootTargetNavigator::limitCommandRate(double current, double target, double limit, double dt) const
  {
    if(dt <= 0.0) return current;
    if(limit <= 0.0) return target;

    const double max_delta = limit * dt;
    return current + std::clamp(target - current, -max_delta, max_delta);
  }

  void DragonRootTargetNavigator::resetTriggerInitializationState()
  {
    l2_trigger_initialized_ = false;
    r2_trigger_initialized_ = false;
  }

  void DragonRootTargetNavigator::rootTailPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(root_target_mutex_);
    latest_root_tail_pose_ = *msg;

    // root/tail_pose is reported in the LINK convention, while root/target_pose expects FLU.
    // Convert the measured attitude in-place so the first seeded target is built in the same frame.
    tf::Quaternion tail_orientation_link;
    tf::quaternionMsgToTF(latest_root_tail_pose_.pose.orientation, tail_orientation_link);
    if(tail_orientation_link.length2() < 1e-12)
      {
        tail_orientation_link.setRPY(0.0, 0.0, 0.0);
      }
    else
      {
        tail_orientation_link.normalize();
      }

    tf::Quaternion link_to_flu = tf::createQuaternionFromYaw(M_PI);
    tf::Quaternion tail_orientation_flu = tail_orientation_link * link_to_flu;
    tail_orientation_flu.normalize();
    tf::quaternionTFToMsg(tail_orientation_flu, latest_root_tail_pose_.pose.orientation);

    has_root_tail_pose_ = true;
  }

  void DragonRootTargetNavigator::clearRootTargetStateLocked()
  {
    root_target_initialized_ = false;
    root_target_command_active_ = false;
    last_root_target_command_stamp_ = ros::Time(0);
    root_target_pitch_ = 0.0;
    root_target_yaw_ = 0.0;
    current_forward_speed_cmd_ = 0.0;
    current_yaw_rate_cmd_ = 0.0;
    current_pitch_rate_cmd_ = 0.0;
  }

  void DragonRootTargetNavigator::seedRootTargetFromTailPoseLocked()
  {
    if(!has_root_tail_pose_) return;

    root_target_pose_ = latest_root_tail_pose_;

    tf::Quaternion tail_orientation;
    tf::quaternionMsgToTF(latest_root_tail_pose_.pose.orientation, tail_orientation);
    if(tail_orientation.length2() < 1e-12)
      {
        tail_orientation.setRPY(0.0, 0.0, 0.0);
      }
    else
      {
        tail_orientation.normalize();
      }

    double roll, pitch, yaw;
    tf::Matrix3x3(tail_orientation).getRPY(roll, pitch, yaw);

    root_target_pitch_ = std::clamp(pitch, -max_teleop_rp_angle_, max_teleop_rp_angle_);
    root_target_yaw_ = angles::normalize_angle(yaw);

    tf::Quaternion target_orientation = tf::createQuaternionFromRPY(0.0, root_target_pitch_, root_target_yaw_);
    target_orientation.normalize();
    tf::quaternionTFToMsg(target_orientation, root_target_pose_.pose.orientation);

    root_target_pose_.header.stamp = ros::Time::now();
    root_target_pose_.header.frame_id = latest_root_tail_pose_.header.frame_id;

    root_target_initialized_ = true;
    last_root_target_command_stamp_ = ros::Time(0);
  }
}

PLUGINLIB_EXPORT_CLASS(aerial_robot_navigation::DragonRootTargetNavigator, aerial_robot_navigation::BaseNavigator);
