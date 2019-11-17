#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
import math

# Model Parameters
wheel_diameter = 0.155
front_to_rear_wheel_center = 0.42
left_to_right_wheel_center = 0.285

# Test values
speed = 10.0 # rad/s
turn_radius = 1.0 # meters

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    # Use Kinematic Bicycle model with rear-axis reference point
    # https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-the-kinematic-bicycle-model-Bi8yE
    # Steering angle: s
    # Front to rear distance: L
    # Turning radius: R
    # tan(s) = L/R

    # Apply Ackerman steering model to get left and right wheel steering angles
    # http://datagenetics.com/blog/december12016/index.html
    if (turn_radius >= 0):
        left_steering_angle = math.atan( front_to_rear_wheel_center / (turn_radius - left_to_right_wheel_center/2.0) )
        right_steering_angle = math.atan( front_to_rear_wheel_center / (turn_radius + left_to_right_wheel_center/2.0) )
    else:
        left_steering_angle = -math.atan( front_to_rear_wheel_center / (turn_radius + left_to_right_wheel_center/2.0) )
        right_steering_angle = -math.atan( front_to_rear_wheel_center / (turn_radius - left_to_right_wheel_center/2.0) )        

    pub_vel_left_front_wheel = rospy.Publisher('/jetsoncar/front_left_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/jetsoncar/front_right_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/jetsoncar/front_left_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/jetsoncar/front_right_hinge_position_controller/command', Float64, queue_size=1)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub_vel_left_front_wheel.publish(speed)
        pub_vel_right_front_wheel.publish(speed)
        pub_pos_left_steering_hinge.publish(left_steering_angle)
        pub_pos_right_steering_hinge.publish(right_steering_angle)
        print 'Left =', left_steering_angle, '| Right =', right_steering_angle
        rate.sleep()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
