// -*- mode: c++ -*-
#ifndef DRAGON_COPILOT_PLANNER_H
#define DRAGON_COPILOT_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <aerial_robot_msgs/FullStateTarget.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <kdl/jntarray.hpp>
#include <Eigen/Dense>
#include <deque>
#include <vector>

namespace dragon_planner
{

struct TrajectoryPoint
{
  Eigen::Vector3d position;
  double timestamp;
  
  TrajectoryPoint(const Eigen::Vector3d& pos, double time)
    : position(pos), timestamp(time) {}
};

class CopilotPlanner
{
public:
  CopilotPlanner();
  ~CopilotPlanner() = default;

private:
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber target_pose_sub_;
  ros::Publisher full_state_target_pub_;
  ros::Publisher trajectory_viz_pub_;
  ros::Timer control_timer_;

  // Robot model
  boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  
  // Robot parameters
  int link_num_;
  double link_length_;
  std::vector<int> link_joint_indices_;
  int link_joint_num_;
  
  // Trajectory tracking
  std::deque<TrajectoryPoint> trajectory_buffer_;
  double total_arc_length_;
  bool trajectory_initialized_;
  Eigen::Vector3d last_recorded_position_;
  double last_recorded_time_;
  
  // Control parameters
  double trajectory_sample_interval_;
  double trajectory_buffer_max_length_;
  double control_loop_rate_;
  bool snake_mode_enabled_;
  
  // Current state
  geometry_msgs::PoseStamped::ConstPtr latest_target_pose_;
  KDL::JntArray joint_positions_;
  Eigen::Vector3d root_pos_world_;
  std::vector<Eigen::Vector3d> latest_snake_targets_;
  
  // Callbacks
  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
  void controlTimerCallback(const ros::TimerEvent& event);
  
  // Initialization
  void initializeRobotModel();
  void loadParameters();
  
  // Trajectory management
  void updateTrajectoryBuffer(const Eigen::Vector3d& current_position);
  bool prepareTrajectoryData();
  
  // Snake motion computation
  std::vector<Eigen::Vector3d> computeSnakeTargetPositions();
  Eigen::Vector3d findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point, double target_distance);
  Eigen::VectorXd computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions);
  
  // Helper functions
  Eigen::Vector3d getRootPositionFromPose(const geometry_msgs::Pose& pose);
  
  // Visualization
  visualization_msgs::MarkerArray getTrajectoryVisualization();
};

}  // namespace dragon_planner

#endif  // DRAGON_COPILOT_PLANNER_H
