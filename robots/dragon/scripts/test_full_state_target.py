#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2026, JSK Lab
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the JSK Lab nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Test node for full_state_target functionality.

This node publishes a test message to the full_state_target topic
with hardcoded root_state and joint_state values.
"""

import rospy
from aerial_robot_msgs.msg import FullStateTarget
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3
import math


def publish_full_state_target():
    """
    Publish a test FullStateTarget message.
    """
    rospy.init_node('test_full_state_target_publisher', anonymous=True)

    pub = rospy.Publisher('/dragon/full_state_target', FullStateTarget, queue_size=10)

    # Wait for publisher to be ready
    rospy.sleep(0.5)

    # Create FullStateTarget message
    msg = FullStateTarget()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "world"

    # Set root_state (Odometry type)
    msg.root_state.header = msg.header
    msg.root_state.child_frame_id = "root"

    # Root position: (x=1.0m, y=0.5m, z=2.0m)
    msg.root_state.pose.pose.position = Point(x=1.0, y=0.5, z=2.0)

    # Root orientation: slight rotation around z-axis (yaw=0.3 rad ≈ 17 degrees)
    # Using quaternion for z-axis rotation
    yaw = 0.0
    msg.root_state.pose.pose.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(yaw/2.0),
        w=math.cos(yaw/2.0)
    )

    # Root velocity (not used in current implementation, but set to zero)
    msg.root_state.twist.twist.linear = Vector3(x=0.0, y=0.0, z=0.0)
    msg.root_state.twist.twist.angular = Vector3(x=0.0, y=0.0, z=0.0)

    # Set joint_state (JointState type)
    # Assuming 6 link joints for a dragon robot
    msg.joint_state.header = msg.header

    # Joint positions (6 values in radians)
    # Example configuration: slightly bent configuration
    half_pi = 1.56

    msg.joint_state.position = [-0.6, -half_pi, -0.6, 0, 0, half_pi]

    # Joint velocities (not used in current implementation, but set to zero)
    msg.joint_state.velocity = [0.0] * 6

    # Joint efforts (not used)
    msg.joint_state.effort = []

    # Log the message content
    rospy.loginfo("Publishing FullStateTarget message:")
    rospy.loginfo("  Root position: [%.2f, %.2f, %.2f]",
                  msg.root_state.pose.pose.position.x,
                  msg.root_state.pose.pose.position.y,
                  msg.root_state.pose.pose.position.z)
    rospy.loginfo("  Root orientation (quat): [%.3f, %.3f, %.3f, %.3f]",
                  msg.root_state.pose.pose.orientation.x,
                  msg.root_state.pose.pose.orientation.y,
                  msg.root_state.pose.pose.orientation.z,
                  msg.root_state.pose.pose.orientation.w)
    rospy.loginfo("  Joint positions: %s", msg.joint_state.position)

    # Publish the message
    pub.publish(msg)
    rospy.loginfo("Message published successfully!")

    # Wait a bit to ensure message is sent
    rospy.sleep(0.5)


if __name__ == '__main__':
    try:
        publish_full_state_target()
    except rospy.ROSInterruptException:
        pass
