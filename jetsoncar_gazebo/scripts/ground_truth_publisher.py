#!/usr/bin/env python

'''
Script to publish car ground-truth base_link transform relative to odom frame using the Gazebo pose of the base_link
'''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped, Vector3, Quaternion
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Header
import numpy as np
import math
import tf
from tf import transformations
import tf2_ros

class OdometryNode:
    # Set publishers
    #self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)

    def __init__(self, publish_frequency, world_frame, odom_frame, base_link_frame):
        # init internals
        self.world_frame = world_frame
        self.odom_frame = odom_frame
        self.base_link_frame = base_link_frame
        self.true_pose = Pose()
        self.true_twist = Twist()
        self.true_timestamp = None
        self.prev_timestamp = rospy.Time(0)
        self.tf_listener = tf.TransformListener()
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.interval = 1.0 / publish_frequency

        # Set the update rate
        #rospy.Timer(rospy.Duration(1.0 / publish_frequency), self.timer_callback)

        # Set subscribers
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.sub_robot_pose_update)

    def sub_robot_pose_update(self, msg):
        # Find the index of the jetsoncar base_link
        try:
            arrayIndex = msg.name.index('jetsoncar::base_link')
        except ValueError as e:
            # Wait for Gazebo to startup
            pass
        else:
            # Extract our current position information
            self.true_pose = msg.pose[arrayIndex]
            self.true_twist = msg.twist[arrayIndex]
            self.true_timestamp = rospy.Time.now()

    #def timer_callback(self, event):
        if self.true_timestamp is None:
            return

        if (self.true_timestamp - self.prev_timestamp) < rospy.Duration(self.interval):
            return

        self.prev_timestamp = self.true_timestamp

        # Lookup current Odometry transform to be able to invert this
        try:
            (trans,rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_link_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # Transform not found, set identity
            trans = [0, 0, 0]
            rot = [0, 0, 0, 1]

        True_quat = np.array([self.true_pose.orientation.x, self.true_pose.orientation.y, self.true_pose.orientation.z, self.true_pose.orientation.w])
        True_pos = np.array([self.true_pose.position.x, self.true_pose.position.y, self.true_pose.position.z])
        T_true = tf.transformations.quaternion_matrix(True_quat)
        T_true[0:3, 3] = np.array(True_pos)
        T_true_inv = np.linalg.inv(T_true)

        T_odom = tf.transformations.quaternion_matrix(np.array(rot))
        T_odom[0:3, 3] = np.array(trans)

        worldCorrectionTransform = np.matmul(T_odom, T_true_inv)
        worldCorrectionTransform_pos = worldCorrectionTransform[0:3, 3]
        worldCorrectionTransform_quat = tf.transformations.quaternion_from_matrix(worldCorrectionTransform)

        tfMsg = TransformStamped(
            header=Header(
                frame_id=self.odom_frame,
                stamp=self.true_timestamp
            ),
            child_frame_id=self.world_frame,
            transform=Transform(
                translation=Vector3(worldCorrectionTransform_pos[0], worldCorrectionTransform_pos[1], worldCorrectionTransform_pos[2]),
                rotation=Quaternion(worldCorrectionTransform_quat[0], worldCorrectionTransform_quat[1], worldCorrectionTransform_quat[2], worldCorrectionTransform_quat[3])
            )
        )
        self.tf_pub.sendTransform(tfMsg)

        #print "True Position:", True_pos

        # cmd = Odometry()
        # cmd.header.stamp = self.last_recieved_stamp
        # cmd.header.frame_id = 'map'
        # cmd.child_frame_id = 'base_link'
        # cmd.pose.pose = self.last_received_pose
        # cmd.twist.twist = self.last_received_twist
        # self.pub_odom.publish(cmd)
        #
        # tf = TransformStamped(
        #     header=Header(
        #         frame_id=cmd.header.frame_id,
        #         stamp=cmd.header.stamp
        #     ),
        #     child_frame_id=cmd.child_frame_id,
        #     transform=Transform(
        #         translation=cmd.pose.pose.position,
        #         rotation=cmd.pose.pose.orientation
        #     )
        # )
        # self.tf_pub.sendTransform(tf)
        #
        # tf = TransformStamped(
        #     header=Header(
        #         frame_id='map',
        #         stamp=self.last_recieved_stamp
        #     ),
        #     child_frame_id='rear_right_wheel',
        #     transform=Transform(
        #         translation=self.right_wheel_pose.position,
        #         rotation=self.right_wheel_pose.orientation
        #     )
        # )
        # self.tf_pub.sendTransform(tf)
        #
        # tf = TransformStamped(
        #     header=Header(
        #         frame_id='map',
        #         stamp=self.last_recieved_stamp
        #     ),
        #     child_frame_id='rear_left_wheel',
        #     transform=Transform(
        #         translation=self.left_wheel_pose.position,
        #         rotation=self.left_wheel_pose.orientation
        #     )
        # )
        # self.tf_pub.sendTransform(tf)

# Start the node
if __name__ == '__main__':
    rospy.init_node("jetsoncar_ground_truth_publisher")

    publish_frequency = float(rospy.get_param('~publish_frequency', '100.0'))
    print 'Publish frequency =', publish_frequency

    world_frame = rospy.get_param('~world_frame', 'world')
    odom_frame = rospy.get_param('~odom_frame', 'odom')
    base_link_frame = rospy.get_param('~base_link_frame', 'base_link')

    node = OdometryNode(publish_frequency, world_frame, odom_frame, base_link_frame)
    rospy.spin()
