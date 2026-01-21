// -*- mode: c++ -*-
#pragma once

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <vector>
#include <deque>
#include <memory>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <aerial_robot_estimation/state_estimation.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

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

struct DragonCopilotControlParams
{
  double max_copilot_x_vel;
  double max_copilot_y_vel;
  double max_copilot_z_vel;
  double max_copilot_rot_vel;
  double joy_stick_deadzone;
  bool hold_attitude_on_idle;
  bool snake_mode_enabled;
  double trajectory_sample_interval;
  double trajectory_buffer_max_length;
  double snake_ik_gain;

  DragonCopilotControlParams()
      : max_copilot_x_vel(0.0)
      , max_copilot_y_vel(0.0)
      , max_copilot_z_vel(0.0)
      , max_copilot_rot_vel(0.0)
      , joy_stick_deadzone(0.0)
      , hold_attitude_on_idle(true)
      , snake_mode_enabled(true)
      , trajectory_sample_interval(0.01)
      , trajectory_buffer_max_length(3.0)
      , snake_ik_gain(1.0)
  {
  }
};

class DragonCopilotControl
{
public:
  DragonCopilotControl(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                       double loop_du,
                       const DragonCopilotControlParams& params);
  ~DragonCopilotControl() = default;

  void updateControlState(const RootFrameCommand& root_cmd);
  Eigen::VectorXd computeJointPositions(const RootFrameCommand& root_cmd);

  Eigen::Vector3d computeCoGVelocity() const;
  nav_msgs::Odometry computeBaselinkTargetPose(const RootFrameCommand& root_cmd) const;
  visualization_msgs::MarkerArray getSnakeTrajectoryVizMsg();

  // Accessors for cached data needed by DragonCopilot (if any)
  const KDL::Frame& getWorldToRoot() const { return world_to_root_; }
  const KDL::Frame& getBaselinkToRoot() const { return baselink_to_root_; }
  const Eigen::Vector3d& getRootVelWorldEigen() const { return root_vel_world_eigen_; }
  const Eigen::Matrix3d& getRWorldToRoot() const { return R_world_to_root_; }
  double getJoyJoint1YawDq() const { return joy_joint1_yaw_dq_; }
  
  // Public for visualization
  const std::deque<TrajectoryPoint>& getTrajectoryBuffer() const { return trajectory_buffer_; }
  const std::vector<Eigen::Vector3d>& getSnakeTargetPositions() const { return snake_target_positions_world_; }
  const std::vector<Eigen::Vector3d>& getSnakeCurrentPositions() const { return snake_current_positions_world_; }

  void resetTrajectoryBuffer();

private:
  void cacheFrameTransforms();
  void cacheJacobians();
  void cacheLastLinkTailJacobian();
  void cacheRootFrameVelocities(const RootFrameCommand& root_cmd);
  void getJoint1DqFromJoystick(const RootFrameCommand& root_cmd);
  
  bool prepareTrajectoryData();
  void updateTrajectoryBuffer();
  std::vector<Eigen::Vector3d> getCurrentLinkTailPositionsWorld();
  std::vector<Eigen::Vector3d> computeSnakeTargetPositions();
  Eigen::VectorXd computeJointAnglesFromSnakeTarget();
  KDL::Frame computeExpectedLinkFrame(int link_index, const Eigen::VectorXd& desired_joint_positions);
  Eigen::Vector3d findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point, double target_distance);
  
  // Dependencies
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
  
  // Params
  double loop_du_;
  double max_copilot_x_vel_;
  double max_copilot_y_vel_;
  double max_copilot_z_vel_;
  double max_copilot_rot_vel_;
  double joy_stick_deadzone_;
  bool hold_attitude_on_idle_;

  // Robot Model Params
  int link_num_;
  double link_length_;
  std::vector<double> link_joint_lower_limits_;
  std::vector<double> link_joint_upper_limits_;
  int kdl_tree_joint_num_;
  int link_joint_num_;
  std::vector<int> link_joint_indices_;
  std::vector<std::string> link_names_;
  std::unique_ptr<KDL::TreeJntToJacSolver> jac_solver_;
  
  // State
  KDL::JntArray joint_positions_;
  uint8_t estimate_mode_; 
  
  // Cache
  KDL::Frame world_to_cog_;
  KDL::Frame world_to_root_;
  KDL::Frame root_to_baselink_;
  KDL::Frame baselink_to_root_;
  Eigen::Matrix3d R_world_to_root_;
  bool baselink_yaw_world_init_recorded_;
  double baselink_yaw_world_init_;
  bool baselink_pitch_world_init_recorded_;
  double baselink_pitch_world_init_;
  
  std::vector<KDL::Frame> link_frames_;
  std::vector<Eigen::MatrixXd> link_jacobians_linear_;
  
  KDL::Vector root_vel_world_;
  KDL::Vector root_omega_world_;
  Eigen::Vector3d root_pos_world_eigen_;
  Eigen::Vector3d root_vel_world_eigen_;
  Eigen::Vector3d link2_head_pos_world_eigen_;
  
  Eigen::Vector2d joint1_dq_;
  double joy_joint1_yaw_dq_;
  double accumulated_joint1_yaw_from_joy_;
  double accumulated_joint1_pitch_from_joy_;
  
  // Snake Mode
  bool snake_mode_enabled_;
  double trajectory_sample_interval_;
  double trajectory_buffer_max_length_;
  double snake_ik_gain_;
  double snake_max_joint_delta_;
  
  std::deque<TrajectoryPoint> trajectory_buffer_;
  double total_arc_length_;
  Eigen::Vector3d last_recorded_position_;
  bool trajectory_initialized_;
  std::vector<Eigen::Vector3d> snake_target_positions_world_;
  std::vector<Eigen::Vector3d> snake_current_positions_world_;
};

} // namespace aerial_robot_navigation
