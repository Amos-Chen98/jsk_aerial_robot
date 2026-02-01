// copilot_planner.cpp
// Planner node that generates full state targets with snake-like motion

#include <dragon/copilot_planner.h>
#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <limits>
#include <cmath>

namespace dragon_planner
{

CopilotPlanner::CopilotPlanner()
  : nh_(""), pnh_("~"), total_arc_length_(0.0), trajectory_initialized_(false), last_recorded_time_(0.0)
{
  loadParameters();
  initializeRobotModel();

  // Subscribe to target pose
  target_pose_sub_ = nh_.subscribe("root/target_pose", 1, &CopilotPlanner::targetPoseCallback, this);

  // Advertise full state target
  full_state_target_pub_ = nh_.advertise<aerial_robot_msgs::FullStateTarget>("full_state_target", 1);

  // Advertise trajectory visualization
  trajectory_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("trajectory_visualization", 1);

  // Start control timer
  control_timer_ =
      nh_.createTimer(ros::Duration(1.0 / control_loop_rate_), &CopilotPlanner::controlTimerCallback, this);

  ROS_INFO("[CopilotPlanner] Node initialized");
  ROS_INFO("[CopilotPlanner] Snake mode enabled: %s", snake_mode_enabled_ ? "true" : "false");
  ROS_INFO("[CopilotPlanner] Link number: %d, Link length: %.3f", link_num_, link_length_);
}

void CopilotPlanner::loadParameters()
{
  pnh_.param("control_loop_rate", control_loop_rate_, 50.0);
  pnh_.param("trajectory_sample_interval", trajectory_sample_interval_, 0.05);
  pnh_.param("trajectory_buffer_max_length", trajectory_buffer_max_length_, 10.0);
  pnh_.param("snake_mode_enabled", snake_mode_enabled_, true);

  ROS_INFO("[CopilotPlanner] Loaded parameters:");
  ROS_INFO("  control_loop_rate: %.1f Hz", control_loop_rate_);
  ROS_INFO("  trajectory_sample_interval: %.3f m", trajectory_sample_interval_);
  ROS_INFO("  snake_mode_enabled: %s", snake_mode_enabled_ ? "true" : "false");
}

void CopilotPlanner::initializeRobotModel()
{
  // Create robot model with default parameters
  robot_model_.reset(new aerial_robot_model::transformable::RobotModel(true, false));

  if (!robot_model_)
  {
    ROS_ERROR("[CopilotPlanner] Failed to create robot model");
    ros::shutdown();
    return;
  }

  // Wait for robot model to be ready
  ros::Duration(1.0).sleep();

  // Get robot parameters
  auto transformable_model = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(robot_model_);
  if (!transformable_model)
  {
    ROS_ERROR("[CopilotPlanner] Failed to cast to transformable robot model");
    ros::shutdown();
    return;
  }

  link_num_ = robot_model_->getRotorNum();
  link_length_ = transformable_model->getLinkLength();
  link_joint_indices_ = transformable_model->getLinkJointIndices();
  link_joint_num_ = link_joint_indices_.size();

  // Initialize joint positions
  joint_positions_.resize(robot_model_->getJointNum());

  ROS_INFO("[CopilotPlanner] Robot model initialized successfully");
  ROS_INFO("  Link num: %d, Joint num: %d", link_num_, link_joint_num_);
}

void CopilotPlanner::targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  latest_target_pose_ = msg;
}

void CopilotPlanner::controlTimerCallback(const ros::TimerEvent& event)
{
  if (!latest_target_pose_)
  {
    return;  // No target received yet
  }

  // Get root position from target pose
  Eigen::Vector3d root_position = getRootPositionFromPose(latest_target_pose_->pose);

  // Update trajectory buffer
  updateTrajectoryBuffer(root_position);

  // Create full state target message
  aerial_robot_msgs::FullStateTarget full_state_msg;
  full_state_msg.header.stamp = ros::Time::now();

  // Set root state from target pose
  full_state_msg.root_state.header = latest_target_pose_->header;
  full_state_msg.root_state.pose.pose = latest_target_pose_->pose;

  // Compute joint positions
  Eigen::VectorXd desired_joint_positions;

  if (snake_mode_enabled_ && prepareTrajectoryData())
  {
    // Snake mode: compute joint angles to follow trajectory
    std::vector<Eigen::Vector3d> snake_targets = computeSnakeTargetPositions();
    latest_snake_targets_ = snake_targets;  // Save for visualization

    if (!snake_targets.empty())
    {
      desired_joint_positions = computeJointAnglesFromSnakeTarget(snake_targets);
    }
    else
    {
      // Fall back to neutral position
      desired_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
      if (link_joint_num_ > 1)
        desired_joint_positions(1) = -M_PI / 2.0;
      if (link_joint_num_ > 3)
        desired_joint_positions(3) = M_PI / 2.0;
      if (link_joint_num_ > 5)
        desired_joint_positions(5) = M_PI / 2.0;
    }
  }
  else
  {
    // Default position when not enough trajectory
    desired_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
    if (link_joint_num_ > 1)
      desired_joint_positions(1) = -M_PI / 2.0;
    if (link_joint_num_ > 3)
      desired_joint_positions(3) = M_PI / 2.0;
    if (link_joint_num_ > 5)
      desired_joint_positions(5) = M_PI / 2.0;
  }

  // Set joint state
  full_state_msg.joint_state.header = latest_target_pose_->header;
  full_state_msg.joint_state.name.resize(link_joint_num_);
  full_state_msg.joint_state.position.resize(link_joint_num_);

  for (int i = 0; i < link_joint_num_; i++)
  {
    full_state_msg.joint_state.name[i] = std::string("joint") + std::to_string(i + 1);
    full_state_msg.joint_state.position[i] = desired_joint_positions(i);
  }

  // Publish the message
  full_state_target_pub_.publish(full_state_msg);

  // Publish trajectory visualization
  visualization_msgs::MarkerArray viz_markers = getTrajectoryVisualization();
  trajectory_viz_pub_.publish(viz_markers);
}

void CopilotPlanner::updateTrajectoryBuffer(const Eigen::Vector3d& current_position)
{
  double current_time = ros::Time::now().toSec();

  if (!trajectory_initialized_)
  {
    trajectory_buffer_.push_back(TrajectoryPoint(current_position, current_time));
    last_recorded_position_ = current_position;
    last_recorded_time_ = current_time;
    trajectory_initialized_ = true;
    root_pos_world_ = current_position;
    ROS_INFO("[CopilotPlanner] Trajectory initialized at position: [%.3f, %.3f, %.3f]", current_position.x(),
             current_position.y(), current_position.z());
    return;
  }

  double distance = (current_position - last_recorded_position_).norm();

  if (distance >= trajectory_sample_interval_)
  {
    trajectory_buffer_.push_back(TrajectoryPoint(current_position, current_time));
    total_arc_length_ += distance;
    last_recorded_position_ = current_position;
    last_recorded_time_ = current_time;

    // Remove old points if buffer is too long
    while (!trajectory_buffer_.empty() && total_arc_length_ > trajectory_buffer_max_length_)
    {
      if (trajectory_buffer_.size() > 1)
      {
        double segment_length = (trajectory_buffer_[1].position - trajectory_buffer_[0].position).norm();
        total_arc_length_ -= segment_length;
        trajectory_buffer_.pop_front();
      }
      else
      {
        break;
      }
    }
  }

  root_pos_world_ = current_position;
}

bool CopilotPlanner::prepareTrajectoryData()
{
  double min_required_arc_length = (link_num_ - 1) * link_length_;
  bool trajectory_ready = (total_arc_length_ >= min_required_arc_length);

  if (!trajectory_ready)
  {
    ROS_DEBUG_THROTTLE(2.0, "[CopilotPlanner] Waiting for sufficient trajectory: %.3f / %.3f m", total_arc_length_,
                       min_required_arc_length);
  }

  return trajectory_ready;
}

std::vector<Eigen::Vector3d> CopilotPlanner::computeSnakeTargetPositions()
{
  std::vector<Eigen::Vector3d> target_positions;
  target_positions.reserve(link_num_ - 1);  // Compute tail positions for Link2, Link3, ..., LinkN (total: link_num_-1)

  if (trajectory_buffer_.size() < 2 || !latest_target_pose_)
  {
    return target_positions;
  }

  // Compute Link1 tail position (Link2 head) based on root pose orientation
  // Link1 extends along root's x-axis direction
  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  tf::Matrix3x3 root_rotation(root_quat);

  // Link1 extends in the x direction in root frame
  tf::Vector3 link1_direction = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d link1_tail =
      root_pos_world_ + Eigen::Vector3d(link1_direction.x(), link1_direction.y(), link1_direction.z()) * link_length_;

  // Now compute target positions for Link2 tail, Link3 tail, ..., LinkN tail
  // These are found on the trajectory at successive link_length distances
  Eigen::Vector3d current_head = link1_tail;

  for (int i = 2; i <= link_num_; i++)
  {
    // Find tail position of Link i on the trajectory
    Eigen::Vector3d target = findPointOnTrajectoryAtDistance(current_head, link_length_);
    target_positions.push_back(target);
    current_head = target;
  }
  // print the computed target positions
  ROS_INFO("[CopilotPlanner] Computed snake target positions:");
  for (size_t i = 0; i < target_positions.size(); i++)
  {
    ROS_INFO("  Link %zu tail target: [%.3f, %.3f, %.3f]", i + 2, target_positions[i].x(), target_positions[i].y(),
             target_positions[i].z());
  }
  return target_positions;
}

Eigen::Vector3d CopilotPlanner::findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point,
                                                                double target_distance)
{
  if (trajectory_buffer_.size() < 2)
    return from_point;

  size_t start_segment = 0;
  double min_dist_to_trajectory = std::numeric_limits<double>::max();

  // Find closest segment to from_point
  for (size_t i = 0; i < trajectory_buffer_.size() - 1; i++)
  {
    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;
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

  // Find point at target_distance using sampling method
  // Search backwards along trajectory (from newer to older points)
  Eigen::Vector3d best_point = trajectory_buffer_.front().position;  // Default to oldest point
  double best_distance_error = std::numeric_limits<double>::max();

  // Search from start_segment backwards (towards older trajectory points, index 0)
  for (int i = start_segment; i >= 0; i--)
  {
    if (i + 1 >= trajectory_buffer_.size())
      continue;

    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;

    Eigen::Vector3d seg = p2 - p1;
    double seg_length = seg.norm();

    if (seg_length < 1e-6)
      continue;

    // Sample multiple points along the segment
    for (double alpha = 0.0; alpha <= 1.0; alpha += 0.1)
    {
      Eigen::Vector3d candidate = p1 + alpha * seg;
      double dist = (from_point - candidate).norm();
      double error = std::abs(dist - target_distance);

      if (error < best_distance_error)
      {
        best_distance_error = error;
        best_point = candidate;
      }
    }

    // Early exit if we found a very good match
    if (best_distance_error < 0.01)
      return best_point;
  }

  return best_point;
}

Eigen::VectorXd CopilotPlanner::computeJointAnglesFromSnakeTarget(const std::vector<Eigen::Vector3d>& target_positions)
{
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  // joint index: [joint1_pitch, joint1_yaw, joint2_pitch, joint2_yaw, joint3_pitch, joint3_yaw, ...]
  // Set pitch joints to zero angles: we can assume the robot moves in horizontal plane for simplicity
  joint_positions(0) = 0.0;
  joint_positions(2) = 0.0;
  joint_positions(4) = 0.0;

  // Get link1 direction from root pose
  tf::Quaternion root_quat;
  tf::quaternionMsgToTF(latest_target_pose_->pose.orientation, root_quat);
  tf::Matrix3x3 root_rotation(root_quat);
  tf::Vector3 link1_direction_tf = root_rotation * tf::Vector3(1.0, 0.0, 0.0);
  Eigen::Vector3d link1_direction(link1_direction_tf.x(), link1_direction_tf.y(), link1_direction_tf.z());

  // Calculate link1 tail position (which is also link2 head position)
  Eigen::Vector3d link1_tail = root_pos_world_ + link1_direction * link_length_;

  // Store current link direction for iterative calculation
  Eigen::Vector3d current_link_direction = link1_direction;
  Eigen::Vector3d current_tail_position = link1_tail;

  // Calculate yaw joints iteratively
  // target_positions[0] = link2 tail target, target_positions[1] = link3 tail target, ...
  for (int i = 0; i < target_positions.size(); i++)
  {
    // Desired direction for the next link (from current tail to next tail target)
    Eigen::Vector3d desired_next_direction = (target_positions[i] - current_tail_position).normalized();

    // Calculate yaw angle in horizontal plane (XY plane)
    // Project both directions onto XY plane
    Eigen::Vector2d current_dir_xy(current_link_direction.x(), current_link_direction.y());
    Eigen::Vector2d desired_dir_xy(desired_next_direction.x(), desired_next_direction.y());

    current_dir_xy.normalize();
    desired_dir_xy.normalize();

    // Calculate the yaw angle using atan2
    // The yaw angle is the rotation needed from current direction to desired direction
    double current_yaw = std::atan2(current_dir_xy.y(), current_dir_xy.x());
    double desired_yaw = std::atan2(desired_dir_xy.y(), desired_dir_xy.x());
    double joint_yaw = desired_yaw - current_yaw;

    // Normalize angle to [-pi, pi]
    while (joint_yaw > M_PI)
      joint_yaw -= 2.0 * M_PI;
    while (joint_yaw < -M_PI)
      joint_yaw += 2.0 * M_PI;

    // Set the yaw joint angle
    // Joint index: joint(i+1)_yaw is at index (i*2 + 1)
    int yaw_joint_index = i * 2 + 1;
    if (yaw_joint_index < link_joint_num_)
    {
      joint_positions(yaw_joint_index) = joint_yaw;
    }

    // Update for next iteration
    current_link_direction = desired_next_direction;
    current_tail_position = target_positions[i];
  }

  // print the computed joint positions
  ROS_INFO("[CopilotPlanner] Computed joint positions:");
  for (int i = 0; i < joint_positions.size(); i++)
  {
    ROS_INFO("  Joint %d: %.3f rad", i + 1, joint_positions(i));
  }

  joint_positions(5) = 1.57;

  return joint_positions;
}

Eigen::Vector3d CopilotPlanner::getRootPositionFromPose(const geometry_msgs::Pose& pose)
{
  return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
}

visualization_msgs::MarkerArray CopilotPlanner::getTrajectoryVisualization()
{
  visualization_msgs::MarkerArray marker_array;
  ros::Time current_time = ros::Time::now();

  // Return empty if trajectory not initialized
  if (!trajectory_initialized_)
  {
    return marker_array;
  }

  // Trajectory line marker
  visualization_msgs::Marker trajectory_line;
  trajectory_line.header.frame_id = "world";
  trajectory_line.header.stamp = current_time;
  trajectory_line.ns = "root_trajectory";
  trajectory_line.id = 0;
  trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_line.action = visualization_msgs::Marker::ADD;
  trajectory_line.pose.orientation.w = 1.0;
  trajectory_line.scale.x = 0.03;  // Line width
  trajectory_line.color.r = 1.0;
  trajectory_line.color.g = 0.5;
  trajectory_line.color.b = 0.0;
  trajectory_line.color.a = 0.9;
  trajectory_line.lifetime = ros::Duration(0.0);  // Persistent

  // Add trajectory points to line strip
  for (const auto& point : trajectory_buffer_)
  {
    geometry_msgs::Point p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = point.position.z();
    trajectory_line.points.push_back(p);
  }

  if (!trajectory_line.points.empty())
  {
    marker_array.markers.push_back(trajectory_line);
  }

  // Current root position marker
  visualization_msgs::Marker root_marker;
  root_marker.header.frame_id = "world";
  root_marker.header.stamp = current_time;
  root_marker.ns = "root_position";
  root_marker.id = 1;
  root_marker.type = visualization_msgs::Marker::SPHERE;
  root_marker.action = visualization_msgs::Marker::ADD;
  root_marker.pose.position.x = root_pos_world_.x();
  root_marker.pose.position.y = root_pos_world_.y();
  root_marker.pose.position.z = root_pos_world_.z();
  root_marker.pose.orientation.w = 1.0;
  root_marker.scale.x = 0.1;
  root_marker.scale.y = 0.1;
  root_marker.scale.z = 0.1;
  root_marker.color.r = 1.0;
  root_marker.color.g = 0.0;
  root_marker.color.b = 0.0;
  root_marker.color.a = 1.0;
  root_marker.lifetime = ros::Duration(0.1);

  marker_array.markers.push_back(root_marker);

  // Trajectory sample points
  for (size_t i = 0; i < trajectory_buffer_.size(); i++)
  {
    visualization_msgs::Marker point_marker;
    point_marker.header.frame_id = "world";
    point_marker.header.stamp = current_time;
    point_marker.ns = "trajectory_points";
    point_marker.id = 100 + i;
    point_marker.type = visualization_msgs::Marker::SPHERE;
    point_marker.action = visualization_msgs::Marker::ADD;
    point_marker.pose.position.x = trajectory_buffer_[i].position.x();
    point_marker.pose.position.y = trajectory_buffer_[i].position.y();
    point_marker.pose.position.z = trajectory_buffer_[i].position.z();
    point_marker.pose.orientation.w = 1.0;
    point_marker.scale.x = 0.04;
    point_marker.scale.y = 0.04;
    point_marker.scale.z = 0.04;
    point_marker.color.r = 0.0;
    point_marker.color.g = 1.0;
    point_marker.color.b = 1.0;
    point_marker.color.a = 0.7;
    point_marker.lifetime = ros::Duration(0.0);

    marker_array.markers.push_back(point_marker);
  }

  // Trajectory info text
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "world";
  text_marker.header.stamp = current_time;
  text_marker.ns = "trajectory_info";
  text_marker.id = 2;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = root_pos_world_.x();
  text_marker.pose.position.y = root_pos_world_.y();
  text_marker.pose.position.z = root_pos_world_.z() + 0.3;
  text_marker.pose.orientation.w = 1.0;
  text_marker.scale.z = 0.1;  // Text height
  text_marker.color.r = 1.0;
  text_marker.color.g = 1.0;
  text_marker.color.b = 1.0;
  text_marker.color.a = 1.0;
  text_marker.lifetime = ros::Duration(0.1);

  char text_buffer[100];
  snprintf(text_buffer, sizeof(text_buffer), "Trajectory: %.2fm (%zu points)", total_arc_length_,
           trajectory_buffer_.size());
  text_marker.text = std::string(text_buffer);

  marker_array.markers.push_back(text_marker);

  // Snake target positions markers (Link2, Link3, ... LinkN targets)
  for (size_t i = 0; i < latest_snake_targets_.size(); i++)
  {
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "world";
    target_marker.header.stamp = current_time;
    target_marker.ns = "snake_targets";
    target_marker.id = 200 + i;
    target_marker.type = visualization_msgs::Marker::SPHERE;
    target_marker.action = visualization_msgs::Marker::ADD;
    target_marker.pose.position.x = latest_snake_targets_[i].x();
    target_marker.pose.position.y = latest_snake_targets_[i].y();
    target_marker.pose.position.z = latest_snake_targets_[i].z();
    target_marker.pose.orientation.w = 1.0;
    target_marker.scale.x = 0.08;
    target_marker.scale.y = 0.08;
    target_marker.scale.z = 0.08;
    target_marker.color.r = 1.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 1.0;  // Magenta color for snake targets
    target_marker.color.a = 0.9;
    target_marker.lifetime = ros::Duration(0.1);

    marker_array.markers.push_back(target_marker);

    // Add text label for each target
    visualization_msgs::Marker label_marker;
    label_marker.header.frame_id = "world";
    label_marker.header.stamp = current_time;
    label_marker.ns = "snake_target_labels";
    label_marker.id = 300 + i;
    label_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label_marker.action = visualization_msgs::Marker::ADD;
    label_marker.pose.position.x = latest_snake_targets_[i].x();
    label_marker.pose.position.y = latest_snake_targets_[i].y();
    label_marker.pose.position.z = latest_snake_targets_[i].z() + 0.15;
    label_marker.pose.orientation.w = 1.0;
    label_marker.scale.z = 0.08;
    label_marker.color.r = 1.0;
    label_marker.color.g = 1.0;
    label_marker.color.b = 1.0;
    label_marker.color.a = 1.0;
    label_marker.lifetime = ros::Duration(0.1);
    label_marker.text = "Link" + std::to_string(i + 2);  // Link2, Link3, ...

    marker_array.markers.push_back(label_marker);
  }

  return marker_array;
}

}  // namespace dragon_planner

int main(int argc, char** argv)
{
  ros::init(argc, argv, "copilot_planner");

  try
  {
    dragon_planner::CopilotPlanner planner;
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Exception in CopilotPlanner: %s", e.what());
    return 1;
  }

  return 0;
}
