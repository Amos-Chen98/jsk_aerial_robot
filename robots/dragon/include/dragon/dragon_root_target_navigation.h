#pragma once

#include <dragon/dragon_navigation.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>

namespace aerial_robot_navigation
{
  class DragonRootTargetNavigator : public DragonNavigator
  {
  public:
    DragonRootTargetNavigator();
    ~DragonRootTargetNavigator() override = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    double loop_du) override;

    void update() override;

  private:
    ros::Publisher root_target_pose_pub_;
    ros::Publisher root_target_joint_reset_pub_;
    ros::Subscriber root_tail_pose_sub_;

    std::mutex root_target_mutex_;
    geometry_msgs::PoseStamped latest_root_tail_pose_;
    geometry_msgs::PoseStamped root_target_pose_;
    bool has_root_tail_pose_;
    bool root_target_initialized_;
    bool root_target_command_active_;
    bool root_target_joint_reset_combo_pressed_;
    ros::Time last_root_target_command_stamp_;
    ros::Time last_supported_joy_stamp_;

    bool l2_trigger_initialized_;
    bool r2_trigger_initialized_;

    double root_target_deadzone_;
    double root_target_trigger_init_epsilon_;
    double joy_reconnect_reset_du_;
    double root_target_forward_gain_;
    double root_target_yaw_gain_;
    double root_target_pitch_gain_;
    double root_target_forward_acc_limit_;
    double root_target_yaw_acc_limit_;
    double root_target_pitch_acc_limit_;
    double root_target_pitch_rate_;
    double root_target_pitch_;
    double root_target_yaw_;
    double current_forward_speed_cmd_;
    double current_yaw_rate_cmd_;
    double current_pitch_rate_cmd_;

    void joyStickControl(const sensor_msgs::JoyConstPtr& joy_msg) override;
    void rootTailPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

    bool handleJointResetCombo(const sensor_msgs::Joy& joy_msg);
    void publishJointResetCommand();
    bool normalizeRootTargetJoy(const sensor_msgs::Joy& joy_msg, sensor_msgs::Joy& sanitized_raw,
                                double& yaw, double& pitch, double& forward);
    double correctTriggerRaw(double raw_axis, int button, bool& initialized) const;
    double shapeUnsignedInput(double value) const;
    double shapeSignedInput(double value) const;
    double limitCommandRate(double current, double target, double limit, double dt) const;
    void resetTriggerInitializationState();
    void clearRootTargetStateLocked();
    void seedRootTargetFromTailPoseLocked();
  };
}
