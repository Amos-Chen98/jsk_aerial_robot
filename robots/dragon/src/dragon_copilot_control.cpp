// -*- mode: c++ -*-
#include <dragon/dragon_copilot_control.h>
#include <tf_conversions/tf_kdl.h>
#include <algorithm>
#include <limits>
#include <cmath>
#include <tf/transform_datatypes.h>

using namespace aerial_robot_model;

namespace aerial_robot_navigation
{

DragonCopilotControl::DragonCopilotControl(boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           double loop_du, const DragonCopilotControlParams& params)
  : robot_model_(robot_model)
  , estimator_(estimator)
  , loop_du_(loop_du)
  , max_copilot_x_vel_(params.max_copilot_x_vel)
  , max_copilot_y_vel_(params.max_copilot_y_vel)
  , max_copilot_z_vel_(params.max_copilot_z_vel)
  , max_copilot_rot_vel_(params.max_copilot_rot_vel)
  , joy_stick_deadzone_(params.joy_stick_deadzone)
  , hold_attitude_on_idle_(params.hold_attitude_on_idle)
  , estimate_mode_(estimator_->getEstimateMode())
  , snake_mode_enabled_(params.snake_mode_enabled)
  , trajectory_sample_interval_(params.trajectory_sample_interval)
  , trajectory_buffer_max_length_(params.trajectory_buffer_max_length)
  , snake_ik_gain_(params.snake_ik_gain)
  , total_arc_length_(0.0)
  , trajectory_initialized_(false)
  , joy_joint1_yaw_dq_(0.0)
  , accumulated_joint1_yaw_from_joy_(0.0)
  , accumulated_joint1_pitch_from_joy_(0.0)
  , baselink_yaw_world_init_(0.0)
  , baselink_yaw_world_init_recorded_(false)
  , baselink_pitch_world_init_(0.0)
  , baselink_pitch_world_init_recorded_(false)
{
  joint1_dq_ = Eigen::Vector2d::Zero();

  /* Initialize Jacobian computation components */
  const KDL::Tree& tree = robot_model_->getTree();
  jac_solver_.reset(new KDL::TreeJntToJacSolver(tree));
  kdl_tree_joint_num_ = tree.getNrOfJoints();
  int moveable_joint_num = robot_model_->getJointNum();

  joint_positions_.resize(moveable_joint_num);

  link_num_ = robot_model_->getRotorNum();

  auto transformable_model = boost::dynamic_pointer_cast<aerial_robot_model::transformable::RobotModel>(robot_model_);
  if (transformable_model)
  {
    link_length_ = transformable_model->getLinkLength();
    link_joint_indices_ = transformable_model->getLinkJointIndices();
    link_joint_num_ = link_joint_indices_.size();
    link_joint_lower_limits_ = transformable_model->getLinkJointLowerLimits();
    link_joint_upper_limits_ = transformable_model->getLinkJointUpperLimits();

    link_names_.reserve(link_num_);
    for (int i = 1; i <= link_num_; i++)
    {
      link_names_.push_back("link" + std::to_string(i));
    }
  }

  // Create snake_max_joint_delta_ initially based on max_rot_vel
  snake_max_joint_delta_ = max_copilot_rot_vel_ * loop_du_;
}

void DragonCopilotControl::updateControlState(const nav_msgs::Odometry& root_cmd)
{
  cacheFrameTransforms();
  cacheJacobians();
  cacheLastLinkTailJacobian();
  cacheRootFrameVelocities(root_cmd);
  getJoint1DqFromJoystick(root_cmd);
}

Eigen::VectorXd DragonCopilotControl::computeJointPositions(const nav_msgs::Odometry& root_cmd)
{
  // Compute joint commands
  Eigen::VectorXd desired_joint_positions = Eigen::VectorXd::Zero(link_joint_num_);

  // Get current joint positions
  Eigen::VectorXd current_link_joint_positions(link_joint_num_);
  for (int i = 0; i < link_joint_num_; i++)
  {
    int joint_idx = link_joint_indices_[i];
    current_link_joint_positions(i) = joint_positions_(joint_idx);
  }

  bool trajectory_ready = prepareTrajectoryData();
  bool has_x_motion = (std::abs(root_cmd.twist.twist.linear.x) > 1e-6);
  joy_joint1_yaw_dq_ = joint1_dq_(1);

  if (has_x_motion && snake_mode_enabled_ && trajectory_ready && !snake_target_positions_world_.empty())
  {
    desired_joint_positions = computeJointAnglesFromSnakeTarget();
  }
  else
  {
    desired_joint_positions(0) = current_link_joint_positions(0) + joint1_dq_(0);
    desired_joint_positions(1) = current_link_joint_positions(1) + joint1_dq_(1);
    desired_joint_positions(2) = current_link_joint_positions(2);
    desired_joint_positions(3) = current_link_joint_positions(3);
    desired_joint_positions(4) = current_link_joint_positions(4);
    desired_joint_positions(5) = current_link_joint_positions(5);
  }
  return desired_joint_positions;
}

void DragonCopilotControl::cacheFrameTransforms()
{
  joint_positions_ = robot_model_->getJointPositions();
  robot_model_->updateRobotModel(joint_positions_);

  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::COG, estimate_mode_),
                                         estimator_->getPos(Frame::COG, estimate_mode_))),
                  world_to_cog_);

  KDL::Frame world_to_baselink;
  tf::poseTFToKDL(tf::Pose(tf::Transform(estimator_->getOrientation(Frame::BASELINK, estimate_mode_),
                                         estimator_->getPos(Frame::BASELINK, estimate_mode_))),
                  world_to_baselink);

  if (!baselink_yaw_world_init_recorded_)
  {
    baselink_yaw_world_init_ = estimator_->getEuler(Frame::BASELINK, estimate_mode_).z();
    baselink_yaw_world_init_recorded_ = true;
    ROS_INFO("[DragonCopilot] Initial baselink yaw recorded: %.3f rad", baselink_yaw_world_init_);
  }

  if (!baselink_pitch_world_init_recorded_)
  {
    baselink_pitch_world_init_ = estimator_->getEuler(Frame::BASELINK, estimate_mode_).y();
    baselink_pitch_world_init_recorded_ = true;
    ROS_INFO("[DragonCopilot] Initial baselink pitch recorded: %.3f rad", baselink_pitch_world_init_);
  }

  const auto& seg_tf_map = robot_model_->getSegmentsTf();
  root_to_baselink_ = seg_tf_map.at(robot_model_->getBaselinkName());
  baselink_to_root_ = root_to_baselink_.Inverse();
  world_to_root_ = world_to_baselink * baselink_to_root_;

  KDL::Rotation world_to_root_rotation = world_to_root_.M;
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      R_world_to_root_(i, j) = world_to_root_rotation(i, j);
    }
  }

  root_pos_world_eigen_ << world_to_root_.p.x(), world_to_root_.p.y(), world_to_root_.p.z();

  link_frames_.clear();
  link_frames_.reserve(link_num_);
  for (int i = 0; i < link_num_; i++)
  {
    const std::string& link_name = link_names_[i];
    link_frames_.push_back(seg_tf_map.at(link_name));
  }

  if (link_num_ >= 2)
  {
    KDL::Vector link2_head_root = link_frames_[1].p;
    KDL::Vector link2_head_world = world_to_root_.M * link2_head_root + world_to_root_.p;
    link2_head_pos_world_eigen_ << link2_head_world.x(), link2_head_world.y(), link2_head_world.z();
  }
}

void DragonCopilotControl::cacheJacobians()
{
  link_jacobians_linear_.clear();
  link_jacobians_linear_.reserve(link_num_);

  if (!jac_solver_ || link_joint_indices_.empty())
  {
    ROS_ERROR_THROTTLE(1.0, "[DragonCopilot] Jacobian solver or link joint indices not initialized");
    return;
  }

  for (int i = 0; i < link_num_; i++)
  {
    const std::string& link_name = link_names_[i];
    KDL::Jacobian full_jacobian(kdl_tree_joint_num_);
    int status = jac_solver_->JntToJac(joint_positions_, full_jacobian, link_name);

    if (status < 0)
    {
      ROS_WARN_THROTTLE(1.0, "[DragonCopilot] Failed to compute Jacobian for %s", link_name.c_str());
      continue;
    }

    KDL::Jacobian reduced_jacobian(link_joint_num_);
    for (int row = 0; row < 6; row++)
    {
      for (int col_idx = 0; col_idx < link_joint_num_; col_idx++)
      {
        int joint_idx = link_joint_indices_[col_idx];
        reduced_jacobian(row, col_idx) = full_jacobian(row, joint_idx);
      }
    }
    Eigen::MatrixXd linear_jacobian = reduced_jacobian.data.block(0, 0, 3, link_joint_num_);
    link_jacobians_linear_.push_back(linear_jacobian);
  }
}

void DragonCopilotControl::cacheLastLinkTailJacobian()
{
  const std::string& last_link_name = link_names_[link_num_ - 1];
  KDL::Frame last_link_frame = link_frames_[link_num_ - 1];

  KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);
  KDL::Jacobian full_jacobian_last_head(kdl_tree_joint_num_);
  int status = jac_solver_->JntToJac(joint_positions_, full_jacobian_last_head, last_link_name);

  if (status < 0)
    return;

  KDL::Jacobian reduced_jacobian_head(link_joint_num_);
  for (int row = 0; row < 6; row++)
  {
    for (int col_idx = 0; col_idx < link_joint_num_; col_idx++)
    {
      int joint_idx = link_joint_indices_[col_idx];
      reduced_jacobian_head(row, col_idx) = full_jacobian_last_head(row, joint_idx);
    }
  }

  KDL::Vector offset_root = link_length_ * link_x_direction;
  Eigen::Vector3d offset_eigen(offset_root.x(), offset_root.y(), offset_root.z());
  KDL::Jacobian reduced_jacobian_tail(link_joint_num_);

  for (int col_idx = 0; col_idx < link_joint_num_; col_idx++)
  {
    Eigen::Vector3d v_head(reduced_jacobian_head(0, col_idx), reduced_jacobian_head(1, col_idx),
                           reduced_jacobian_head(2, col_idx));
    Eigen::Vector3d omega(reduced_jacobian_head(3, col_idx), reduced_jacobian_head(4, col_idx),
                          reduced_jacobian_head(5, col_idx));
    Eigen::Vector3d v_tail = v_head + omega.cross(offset_eigen);

    reduced_jacobian_tail(0, col_idx) = v_tail(0);
    reduced_jacobian_tail(1, col_idx) = v_tail(1);
    reduced_jacobian_tail(2, col_idx) = v_tail(2);
    reduced_jacobian_tail(3, col_idx) = omega(0);
    reduced_jacobian_tail(4, col_idx) = omega(1);
    reduced_jacobian_tail(5, col_idx) = omega(2);
  }

  Eigen::MatrixXd linear_jacobian_tail = reduced_jacobian_tail.data.block(0, 0, 3, link_joint_num_);
  link_jacobians_linear_.push_back(linear_jacobian_tail);
}

void DragonCopilotControl::cacheRootFrameVelocities(const nav_msgs::Odometry& root_cmd)
{
  KDL::Vector root_vel_body(root_cmd.twist.twist.linear.x, root_cmd.twist.twist.linear.y, 0.0);
  root_vel_world_ = world_to_root_.M * root_vel_body;
  root_vel_world_.z(root_vel_world_.z() + root_cmd.twist.twist.linear.z);
  root_vel_world_eigen_ << root_vel_world_.x(), root_vel_world_.y(), root_vel_world_.z();
  root_omega_world_ = KDL::Vector::Zero();
}

void DragonCopilotControl::getJoint1DqFromJoystick(const nav_msgs::Odometry& root_cmd)
{
  joint1_dq_(0) = std::clamp(root_cmd.twist.twist.angular.y * loop_du_, -snake_max_joint_delta_, snake_max_joint_delta_);
  joint1_dq_(1) = std::clamp(-root_cmd.twist.twist.angular.z * loop_du_, -snake_max_joint_delta_, snake_max_joint_delta_);

  accumulated_joint1_yaw_from_joy_ += joint1_dq_(1);
  accumulated_joint1_pitch_from_joy_ += joint1_dq_(0);
}

Eigen::Vector3d DragonCopilotControl::computeCoGVelocity() const
{
  KDL::Vector root_to_cog_offset_world = world_to_cog_.p - world_to_root_.p;
  KDL::Vector vel_from_rotation = root_omega_world_ * root_to_cog_offset_world;
  KDL::Vector des_cog_vel_world = root_vel_world_ + vel_from_rotation;

  return Eigen::Vector3d(des_cog_vel_world.x(), des_cog_vel_world.y(), des_cog_vel_world.z());
}

nav_msgs::Odometry DragonCopilotControl::computeBaselinkTargetPose(const nav_msgs::Odometry& root_cmd) const
{
  double des_baselink_r = 0.0;
  const double max_baselink_pitch = 1.5;
  double des_baselink_p = baselink_pitch_world_init_ - accumulated_joint1_pitch_from_joy_ * 0.5;
  des_baselink_p = std::clamp(des_baselink_p, -max_baselink_pitch, max_baselink_pitch);

  double des_baselink_y = baselink_yaw_world_init_ - accumulated_joint1_yaw_from_joy_ * 0.5;

  // When joint1_yaw reaches limits, compensate the joint1_yaw command to baselink yaw position
  if (link_joint_indices_.size() > 1 && link_joint_lower_limits_.size() > 1 && link_joint_upper_limits_.size() > 1)
  {
    double current_joint1_yaw = joint_positions_(link_joint_indices_[1]);
    double yaw_vel_cmd = root_cmd.twist.twist.angular.z;
    double threshold = 0.02;

    if ((current_joint1_yaw >= link_joint_upper_limits_[1] - threshold && yaw_vel_cmd < 0.0) ||
        (current_joint1_yaw <= link_joint_lower_limits_[1] + threshold && yaw_vel_cmd > 0.0))
    {
      des_baselink_y -= joint1_dq_(1);
    }
  }

  // Create Odometry message for target_rotation_motion topic
  nav_msgs::Odometry target_msg;
  target_msg.header.stamp = ros::Time::now();
  target_msg.header.frame_id = "baselink";  // Using baselink frame as per the callback logic

  // Set orientation (quaternion from RPY)
  tf::Quaternion q;
  q.setRPY(des_baselink_r, des_baselink_p, des_baselink_y);
  tf::quaternionTFToMsg(q, target_msg.pose.pose.orientation);

  // Set angular velocity to zero (using position control only)
  target_msg.twist.twist.angular.x = 0.0;
  target_msg.twist.twist.angular.y = 0.0;
  target_msg.twist.twist.angular.z = 0.0;

  return target_msg;
}

bool DragonCopilotControl::prepareTrajectoryData()
{
  updateTrajectoryBuffer();
  double min_required_arc_length = (link_num_ - 1) * link_length_;
  bool trajectory_ready = (total_arc_length_ >= min_required_arc_length);

  if (trajectory_ready)
  {
    snake_current_positions_world_ = getCurrentLinkTailPositionsWorld();
    snake_target_positions_world_ = computeSnakeTargetPositions();
  }
  else
  {
    ROS_INFO_THROTTLE(1.0, "[DragonCopilot] Waiting for trajectory buffer: %.2f / %.2f m (%.1f%%)", total_arc_length_,
                      min_required_arc_length, 100.0 * total_arc_length_ / min_required_arc_length);
  }
  return trajectory_ready;
}

void DragonCopilotControl::updateTrajectoryBuffer()
{
  const Eigen::Vector3d& current_position = link2_head_pos_world_eigen_;
  double current_time = ros::Time::now().toSec();

  if (!trajectory_initialized_)
  {
    trajectory_buffer_.push_front(TrajectoryPoint(current_position, current_time));
    last_recorded_position_ = current_position;
    trajectory_initialized_ = true;
    return;
  }

  double distance = (current_position - last_recorded_position_).norm();
  if (distance >= trajectory_sample_interval_)
  {
    trajectory_buffer_.push_front(TrajectoryPoint(current_position, current_time));
    total_arc_length_ += distance;
    last_recorded_position_ = current_position;

    while (total_arc_length_ > trajectory_buffer_max_length_ && trajectory_buffer_.size() > 2)
    {
      size_t last_idx = trajectory_buffer_.size() - 1;
      double segment_length =
          (trajectory_buffer_[last_idx].position - trajectory_buffer_[last_idx - 1].position).norm();
      total_arc_length_ -= segment_length;
      trajectory_buffer_.pop_back();
    }
  }
}

void DragonCopilotControl::resetTrajectoryBuffer()
{
  trajectory_buffer_.clear();
  total_arc_length_ = 0.0;
  trajectory_initialized_ = false;
  ROS_INFO("[DragonCopilot] Trajectory buffer reset");
}

std::vector<Eigen::Vector3d> DragonCopilotControl::getCurrentLinkTailPositionsWorld()
{
  std::vector<Eigen::Vector3d> tail_positions_world;
  tail_positions_world.reserve(link_num_ - 1);

  for (int i = 1; i < link_num_; i++)
  {
    KDL::Vector tail_pos_root;
    if (i < link_num_ - 1)
    {
      const KDL::Frame& next_link_frame = link_frames_[i + 1];
      tail_pos_root = next_link_frame.p;
    }
    else
    {
      const KDL::Frame& last_link_frame = link_frames_[i];
      KDL::Vector link_x_direction = last_link_frame.M * KDL::Vector(1.0, 0.0, 0.0);
      tail_pos_root = last_link_frame.p + link_length_ * link_x_direction;
    }
    KDL::Vector tail_pos_world = world_to_root_.M * tail_pos_root + world_to_root_.p;
    Eigen::Vector3d tail_pos;
    tail_pos << tail_pos_world.x(), tail_pos_world.y(), tail_pos_world.z();
    tail_positions_world.push_back(tail_pos);
  }
  return tail_positions_world;
}

std::vector<Eigen::Vector3d> DragonCopilotControl::computeSnakeTargetPositions()
{
  std::vector<Eigen::Vector3d> target_positions;
  target_positions.reserve(link_num_ - 1);

  if (trajectory_buffer_.size() < 2)
    return target_positions;

  double dt = loop_du_;
  double omega_norm = root_omega_world_.Norm();
  KDL::Rotation delta_rot = KDL::Rotation::Identity();
  if (omega_norm > 1e-6)
  {
    KDL::Vector axis = root_omega_world_ / omega_norm;
    delta_rot = KDL::Rotation::Rot(axis, omega_norm * dt);
  }
  KDL::Rotation predicted_root_rot = delta_rot * world_to_root_.M;
  Eigen::Vector3d root_pos_predicted = root_pos_world_eigen_ + root_vel_world_eigen_ * dt;

  KDL::Frame world_to_root_predicted;
  world_to_root_predicted.p = KDL::Vector(root_pos_predicted.x(), root_pos_predicted.y(), root_pos_predicted.z());
  world_to_root_predicted.M = predicted_root_rot;

  KDL::Frame link2_head_frame_world = world_to_root_predicted * link_frames_[1];
  Eigen::Vector3d link2_head_predicted(link2_head_frame_world.p.x(), link2_head_frame_world.p.y(),
                                       link2_head_frame_world.p.z());

  Eigen::Vector3d current_head = link2_head_predicted;
  for (int i = 1; i < link_num_; i++)
  {
    Eigen::Vector3d target_tail_world = findPointOnTrajectoryAtDistance(current_head, link_length_);
    target_positions.push_back(target_tail_world);
    current_head = target_tail_world;
  }
  return target_positions;
}

Eigen::Vector3d DragonCopilotControl::findPointOnTrajectoryAtDistance(const Eigen::Vector3d& from_point,
                                                                      double target_distance)
{
  if (trajectory_buffer_.size() < 2)
    return from_point;

  size_t start_segment = 0;
  double min_dist_to_trajectory = std::numeric_limits<double>::max();

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

  Eigen::Vector3d best_point = trajectory_buffer_.back().position;
  double best_distance_error = std::numeric_limits<double>::max();
  bool found_valid_point = false;

  for (size_t i = start_segment; i < trajectory_buffer_.size() - 1; i++)
  {
    const Eigen::Vector3d& p1 = trajectory_buffer_[i].position;
    const Eigen::Vector3d& p2 = trajectory_buffer_[i + 1].position;
    Eigen::Vector3d d = p2 - p1;
    Eigen::Vector3d f = p1 - from_point;
    double a = d.dot(d);
    double b = 2.0 * f.dot(d);
    double c = f.dot(f) - target_distance * target_distance;
    double discriminant = b * b - 4.0 * a * c;

    if (discriminant >= 0 && a > 1e-10)
    {
      double sqrt_disc = std::sqrt(discriminant);
      double t1 = (-b - sqrt_disc) / (2.0 * a);
      double t2 = (-b + sqrt_disc) / (2.0 * a);

      if (t1 >= 0.0 && t1 <= 1.0)
      {
        Eigen::Vector3d candidate = p1 + t1 * d;
        double dist_error = std::abs((candidate - from_point).norm() - target_distance);
        if (dist_error < best_distance_error)
        {
          best_distance_error = dist_error;
          best_point = candidate;
          found_valid_point = true;
        }
      }
      if (t2 >= 0.0 && t2 <= 1.0)
      {
        Eigen::Vector3d candidate = p1 + t2 * d;
        double dist_error = std::abs((candidate - from_point).norm() - target_distance);
        if (dist_error < best_distance_error)
        {
          best_distance_error = dist_error;
          best_point = candidate;
          found_valid_point = true;
        }
      }
    }
    if (found_valid_point && best_distance_error < 0.01)
      return best_point;
  }
  return best_point;
}

Eigen::VectorXd DragonCopilotControl::computeJointAnglesFromSnakeTarget()
{
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(link_joint_num_);
  for (int i = 0; i < link_num_ - 1; i++)
  {
    const Eigen::Vector3d& link_tail_target_world = snake_target_positions_world_[i];
    KDL::Vector link_tail_target_world_kdl(link_tail_target_world.x(), link_tail_target_world.y(),
                                           link_tail_target_world.z());
    KDL::Vector link_tail_target_root = world_to_root_.Inverse() * link_tail_target_world_kdl;

    KDL::Frame expected_link_frame;
    if (i == 0)
      expected_link_frame = link_frames_[1];
    else
      expected_link_frame = computeExpectedLinkFrame(i + 1, joint_positions);

    KDL::Vector link_tail_target_link = expected_link_frame.Inverse() * link_tail_target_root;
    double dx = link_tail_target_link.x();
    double dy = link_tail_target_link.y();
    double dz = link_tail_target_link.z();

    double desired_joint_pitch = std::atan2(-dz, dx);
    double dx_dz_norm = std::sqrt(dx * dx + dz * dz);
    double desired_joint_yaw = std::atan2(dy, dx_dz_norm);

    int joint_pitch_limit_idx = 2 * i;
    int joint_yaw_limit_idx = 2 * i + 1;
    desired_joint_pitch = std::clamp(desired_joint_pitch, link_joint_lower_limits_[joint_pitch_limit_idx],
                                     link_joint_upper_limits_[joint_pitch_limit_idx]);
    desired_joint_yaw = std::clamp(desired_joint_yaw, link_joint_lower_limits_[joint_yaw_limit_idx],
                                   link_joint_upper_limits_[joint_yaw_limit_idx]);

    joint_positions(2 * i) = desired_joint_pitch;
    joint_positions(2 * i + 1) = desired_joint_yaw;
  }

  joint_positions(0) += joint1_dq_(0);
  joint_positions(1) += joint1_dq_(1);
  joint_positions(4) = 0.0;
  joint_positions(5) = M_PI / 2.0;

  return joint_positions;
}

KDL::Frame DragonCopilotControl::computeExpectedLinkFrame(int link_index,
                                                          const Eigen::VectorXd& desired_joint_positions)
{
  KDL::Frame current_frame = KDL::Frame::Identity();
  for (int i = 1; i <= link_index; i++)
  {
    if (i == 1)
      current_frame = link_frames_[0];
    else
    {
      int joint_idx = i - 2;
      double joint_pitch = desired_joint_positions(2 * joint_idx);
      double joint_yaw = desired_joint_positions(2 * joint_idx + 1);
      KDL::Rotation joint_rotation = KDL::Rotation::RotY(joint_pitch) * KDL::Rotation::RotZ(joint_yaw);

      KDL::Vector translation = current_frame.M * KDL::Vector(link_length_, 0.0, 0.0);
      current_frame.p = current_frame.p + translation;
      current_frame.M = current_frame.M * joint_rotation;
    }
  }
  return current_frame;
}

visualization_msgs::MarkerArray DragonCopilotControl::getSnakeTrajectoryVizMsg()
{
  visualization_msgs::MarkerArray marker_array;
  ros::Time current_time = ros::Time::now();

  visualization_msgs::Marker trajectory_line;
  trajectory_line.header.frame_id = "world";
  trajectory_line.header.stamp = current_time;
  trajectory_line.ns = "snake_trajectory";
  trajectory_line.id = 0;
  trajectory_line.type = visualization_msgs::Marker::LINE_STRIP;
  trajectory_line.action = visualization_msgs::Marker::ADD;
  trajectory_line.pose.orientation.w = 1.0;
  trajectory_line.scale.x = 0.02;
  trajectory_line.color.r = 0.5;
  trajectory_line.color.g = 0.8;
  trajectory_line.color.b = 1.0;
  trajectory_line.color.a = 0.8;

  for (const auto& point : trajectory_buffer_)
  {
    geometry_msgs::Point p;
    p.x = point.position.x();
    p.y = point.position.y();
    p.z = point.position.z();
    trajectory_line.points.push_back(p);
  }
  marker_array.markers.push_back(trajectory_line);

  for (size_t i = 0; i < snake_target_positions_world_.size(); i++)
  {
    visualization_msgs::Marker target_sphere;
    target_sphere.header.frame_id = "world";
    target_sphere.header.stamp = current_time;
    target_sphere.ns = "snake_targets";
    target_sphere.id = i;
    target_sphere.type = visualization_msgs::Marker::SPHERE;
    target_sphere.action = visualization_msgs::Marker::ADD;
    target_sphere.pose.position.x = snake_target_positions_world_[i].x();
    target_sphere.pose.position.y = snake_target_positions_world_[i].y();
    target_sphere.pose.position.z = snake_target_positions_world_[i].z();
    target_sphere.pose.orientation.w = 1.0;
    target_sphere.scale.x = 0.05;
    target_sphere.scale.y = 0.05;
    target_sphere.scale.z = 0.05;
    target_sphere.color.r = 0.0;
    target_sphere.color.g = 1.0;
    target_sphere.color.b = 0.0;
    target_sphere.color.a = 0.8;
    marker_array.markers.push_back(target_sphere);
  }

  for (size_t i = 0; i < snake_current_positions_world_.size(); i++)
  {
    visualization_msgs::Marker current_sphere;
    current_sphere.header.frame_id = "world";
    current_sphere.header.stamp = current_time;
    current_sphere.ns = "snake_current";
    current_sphere.id = i;
    current_sphere.type = visualization_msgs::Marker::SPHERE;
    current_sphere.action = visualization_msgs::Marker::ADD;
    current_sphere.pose.position.x = snake_current_positions_world_[i].x();
    current_sphere.pose.position.y = snake_current_positions_world_[i].y();
    current_sphere.pose.position.z = snake_current_positions_world_[i].z();
    current_sphere.pose.orientation.w = 1.0;
    current_sphere.scale.x = 0.04;
    current_sphere.scale.y = 0.04;
    current_sphere.scale.z = 0.04;
    current_sphere.color.r = 1.0;
    current_sphere.color.g = 0.0;
    current_sphere.color.b = 0.0;
    current_sphere.color.a = 0.8;
    marker_array.markers.push_back(current_sphere);
  }

  return marker_array;
}

}  // namespace aerial_robot_navigation
