#!/usr/bin/env python

'''
Script to parse /jetsoncar/joint_states topic and publish encoder values to /encoders
'''

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from jetsoncar_msgs.msg import Encoders
import numpy as np
import math
import tf2_ros

class EncoderNode:

    def __init__(self, publish_frequency):
        self.publish_frequency = publish_frequency

        # Set publishers
        self.pub_encoders = rospy.Publisher('/encoders', Encoders, queue_size=1)

        # Subscribe to joint_states
        rospy.Subscriber('/jetsoncar/joint_states', JointState, self.joint_states_update)

        # Default values
        self.recieve_timestamp = rospy.Time.now()
        self.angleFront = 0.0
        self.angleRear = 0.0
        self.velocityFront = 0.0
        self.velocityRear = 0.0
        self.steeringAngle = 0.0

        # Parameters
        self.front_to_rear_wheel_center = 0.42
        self.left_to_right_wheel_center = 0.285

    def joint_states_update(self, msg):
        self.recieve_timestamp = rospy.Time.now()

        try:
            # Find the index of the wheels
            idxFrontLeft = msg.name.index('front_left_wheel_joint')
            idxFrontRight = msg.name.index('front_right_wheel_joint')
            idxRearLeft = msg.name.index('rear_left_wheel_joint')
            idxRearRight = msg.name.index('rear_right_wheel_joint')
            idxSteeringLeft = msg.name.index('front_left_hinge_joint')
            idxSteeringRight = msg.name.index('front_right_hinge_joint')
        
            # Extract encoder angles in radians
            angleFrontLeft = msg.position[idxFrontLeft]
            angleFrontRight = msg.position[idxFrontRight]
            angleRearLeft = msg.position[idxRearLeft]
            angleRearRight = msg.position[idxRearRight]

            # Extract wheel velocities in rad/s
            velocityFrontLeft = msg.velocity[idxFrontLeft]
            velocityFrontRight = msg.velocity[idxFrontRight]
            velocityRearLeft = msg.velocity[idxRearLeft]
            velocityRearRight = msg.velocity[idxRearRight]

            # Extract wheel steering angles
            steeringLeft = msg.position[idxSteeringLeft]
            steeringRight = msg.position[idxSteeringRight]

            # Take the mean of the angles and velocities to ressemble center bicycle model velocity
            self.angleFront = (angleFrontLeft + angleFrontRight) / 2.0
            self.angleRear = (angleRearLeft + angleRearRight) / 2.0
            self.velocityFront = (velocityFrontLeft + velocityFrontRight) / 2.0
            self.velocityRear = (velocityRearLeft + velocityRearRight) / 2.0

            # Use the inverted Ackerman steering model to compute steering angle from one of the individual wheel steering angles
            # http://datagenetics.com/blog/december12016/index.html
            # steering = math.atan( front_to_rear_wheel_center / (turn_radius - left_to_right_wheel_center/2.0) )
            # tan(steering) == front_to_rear_wheel_center / (turn_radius - left_to_right_wheel_center/2.0)
            # front_to_rear_wheel_center / tan(steering) == (turn_radius - left_to_right_wheel_center/2.0)
            # front_to_rear_wheel_center / tan(steering) + left_to_right_wheel_center/2.0 == turn_radius
            R = self.front_to_rear_wheel_center / max(abs(steeringLeft), abs(steeringRight)) + self.left_to_right_wheel_center/2.0
            self.steeringAngle = math.atan(self.front_to_rear_wheel_center/R)

        except ValueError as e:
            # Wait for Gazebo to startup
            pass

    def spin(self):
        rate = rospy.Rate(self.publish_frequency)

        while not rospy.is_shutdown():
            # Construct and publish encoder message
            encoderMsg = Encoders()
            encoderMsg.receive_time = self.recieve_timestamp
            encoderMsg.mcu_time = 0 # not implemented in simulation
            encoderMsg.steering_angle = self.steeringAngle
            encoderMsg.front_angle = self.angleFront
            encoderMsg.rear_angle = self.angleRear
            encoderMsg.front_velocity = self.velocityFront
            encoderMsg.rear_velocity = self.velocityRear
            self.pub_encoders.publish(encoderMsg)

            rate.sleep()

# Start the node
if __name__ == '__main__':
    rospy.init_node("jetsoncar_encoders_simulator")

    publish_frequency = float(rospy.get_param('~publish_frequency', '100.0'))
    print 'Publish frequency =', publish_frequency

    node = EncoderNode(publish_frequency)
    node.spin()
