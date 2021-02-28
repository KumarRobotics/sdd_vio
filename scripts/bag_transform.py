#!/usr/bin/env python
import rosbag
import sys
import os
import tf
import geometry_msgs
import nav_msgs

with rosbag.Bag('pose_stamped.bag', 'w') as outbag:
    cwd = os.getcwd()
    for topic, msg, t in rosbag.Bag(sys.argv[1]).read_messages():
        if "vicon" in topic:
            msg_out = geometry_msgs.msg.PoseStamped()
            msg_out.header = msg.header
            msg_out.pose.position.x = msg.transform.translation.x
            msg_out.pose.position.y = msg.transform.translation.y
            msg_out.pose.position.z = msg.transform.translation.z
            msg_out.pose.orientation = msg.transform.rotation
            outbag.write('/vicon/pose_stamped', msg_out, t)
        if topic == "/firefly_sbx/vio/odom":
            msg_out = geometry_msgs.msg.PoseStamped()
            msg_out.header = msg.header
            msg_out.pose = msg.pose.pose
            outbag.write('/msckf/pose_stamped', msg_out, t)
        if topic == "/rovio/odometry":
            msg_out = geometry_msgs.msg.PoseStamped()
            msg_out.header = msg.header
            msg_out.pose = msg.pose.pose
            outbag.write('/rovio/pose_stamped', msg_out, t)
        if topic == "/vins_estimator/odometry":
            msg_out = geometry_msgs.msg.PoseStamped()
            msg_out.header = msg.header
            msg_out.pose = msg.pose.pose
            outbag.write('/vins/pose_stamped', msg_out, t)
        outbag.write(topic, msg, t)


