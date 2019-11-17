/* Copyright (C) 2019-2020 Thomas Jespersen, TKJ Electronics. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the MIT License
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the MIT License for further details.
 *
 * Contact information
 * ------------------------------------------
 * Thomas Jespersen, TKJ Electronics
 * Web      :  http://www.tkjelectronics.dk
 * e-mail   :  thomasj@tkjelectronics.dk
 * ------------------------------------------
 */

#include <ros/ros.h>

/* Include standard libraries */
#include <string>
#include <boost/bind.hpp>
#include <cmath>

/* Include ROS libraries */
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>

/* Include message types */
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/LinkStates.h"
#include "ackermann_msgs/AckermannDrive.h"

/* Include generated Services */

/* Include generated Message Types */
#include <jetsoncar_msgs/Encoders.h>

/* Include generated Dynamic Reconfigure parameters */

// For CLion to update/capture the changes made to generated services, message types and parameters, open the "build" folder and run "make"


/* This node combines the functionality of "encoders_simulator.py" and "ground_truth_publisher.py" into C++ */
/*
 * Should include the encoders sensor output
 * And ground truth publishing
 * And command velocity input
 */

/* Global variables, but the functions are not multi-threaded so no need for mutexes */
std::string tfPrefix;

double min_steering = 0.003;
double max_steering = 1.0;

// Encoder variables
ros::Time encoderTime;
double wheel_radius = 1;
double front_to_rear_wheel_distance = 1;
double left_to_right_wheel_distance = 1;
double angleFront=0, angleRear=0;
double velocityFront=0, velocityRear=0;
double steeringAngle=0;

// Ground-truth variables
ros::Time true_timestamp(0);
geometry_msgs::Pose true_pose;
geometry_msgs::Twist true_velocity;

typedef struct GazeboWheelPublishers {
    struct {
        ros::Publisher left;
        ros::Publisher right;
    } front;
    struct {
        ros::Publisher left;
        ros::Publisher right;
    } rear;
    struct {
        ros::Publisher left;
        ros::Publisher right;
    } steering;
} GazeboWheelPublishers;

std_msgs::Float64 Float64(double value)
{
    std_msgs::Float64 msg;
    msg.data = value;
    return msg;
}

void ROS_Callback_joint_states(const sensor_msgs::JointState::ConstPtr& msg)
{
    double angleFrontLeft=0, velocityFrontLeft=0;
    double angleFrontRight=0, velocityFrontRight=0;
    double angleRearLeft=0, velocityRearLeft=0;
    double angleRearRight=0, velocityRearRight=0;
    double steeringLeft=0, steeringRight=0;
    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i].find("front_left_wheel_joint") != std::string::npos) {
            angleFrontLeft = msg->position[i];
            velocityFrontLeft = msg->velocity[i];
        }
        else if (msg->name[i].find("front_right_wheel_joint") != std::string::npos) {
            angleFrontRight = msg->position[i];
            velocityFrontRight = msg->velocity[i];
        }
        else if (msg->name[i].find("rear_left_wheel_joint") != std::string::npos) {
            angleRearLeft = msg->position[i];
            velocityRearLeft = msg->velocity[i];
        }
        else if (msg->name[i].find("rear_right_wheel_joint") != std::string::npos) {
            angleRearRight = msg->position[i];
            velocityRearRight = msg->velocity[i];
        }
        else if (msg->name[i].find("front_left_hinge_joint") != std::string::npos) {
            steeringLeft = msg->position[i];
        }
        else if (msg->name[i].find("front_right_hinge_joint") != std::string::npos) {
            steeringRight = msg->position[i];
        }
    }

    // Take the mean of the angles and velocities to ressemble center bicycle model velocity
    angleFront = (angleFrontLeft + angleFrontRight) / 2.0;
    angleRear = (angleRearLeft + angleRearRight) / 2.0;
    velocityFront = (velocityFrontLeft + velocityFrontRight) / 2.0;
    velocityRear = (velocityRearLeft + velocityRearRight) / 2.0;

    /* Use the inverted Ackerman steering model to compute steering angle from one of the individual wheel steering angles
     * http://datagenetics.com/blog/december12016/index.html
     * steering = math.atan( front_to_rear_wheel_center / (turn_radius - left_to_right_wheel_center/2.0) )
     * tan(steering) == front_to_rear_wheel_center / (turn_radius - left_to_right_wheel_center/2.0)
     * front_to_rear_wheel_center / tan(steering) == (turn_radius - left_to_right_wheel_center/2.0)
     * front_to_rear_wheel_center / tan(steering) + left_to_right_wheel_center/2.0 == turn_radius
    */
    if (std::abs(steeringLeft) >= min_steering && std::abs(steeringRight) >= min_steering) {
        double R = front_to_rear_wheel_distance / std::max(std::abs(steeringLeft), std::abs(steeringRight)) +
                   left_to_right_wheel_distance / 2.0;
        steeringAngle = std::atan(front_to_rear_wheel_distance / R);
        if (std::signbit(steeringRight) && std::abs(steeringRight) > std::abs(steeringLeft))
            steeringAngle *= -1;
    } else {
        steeringAngle = 0;
    }

    encoderTime = msg->header.stamp;
}

void ROS_Callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg, GazeboWheelPublishers& pub)
{
    ros::Time current_time = ros::Time::now();

    // Convert cmd_vel command into actuation commends sent to the Gazebo simulated wheel controllers
    double speed = msg->linear.x;
    double steering_angle = msg->angular.z; // misuse of using this to capture the steering angle

    double wheel_angular_velocity = speed / wheel_radius;

    if (std::abs(steering_angle) < min_steering) steering_angle = 0;
    else if (steering_angle > max_steering) steering_angle = max_steering;
    else if (steering_angle < -max_steering) steering_angle = -max_steering;

    ROS_INFO("Requested velocity %2.2f and steering %2.2f", speed, steering_angle);

    // Use Kinematic Bicycle model with rear-axis reference point
    // https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-the-kinematic-bicycle-model-Bi8yE
    // Steering angle: s
    // Front to rear distance: L
    // Turning radius: R
    // tan(s) = L/R
    double R = front_to_rear_wheel_distance / std::tan(std::abs(steering_angle));

    // Apply Ackerman steering model to get left and right wheel steering angles
    // http://datagenetics.com/blog/december12016/index.html
    double left_steering_angle, right_steering_angle;
    if (std::signbit(steering_angle) == 0) {
        left_steering_angle = std::atan(front_to_rear_wheel_distance / (R - left_to_right_wheel_distance / 2.0));
        right_steering_angle = std::atan(front_to_rear_wheel_distance / (R + left_to_right_wheel_distance / 2.0));
    } else {
        left_steering_angle = -std::atan(front_to_rear_wheel_distance / (R + left_to_right_wheel_distance / 2.0));
        right_steering_angle = -std::atan(front_to_rear_wheel_distance / (R - left_to_right_wheel_distance / 2.0));
    }

    pub.front.left.publish(Float64(wheel_angular_velocity));
    pub.front.right.publish(Float64(wheel_angular_velocity));
    pub.rear.left.publish(Float64(wheel_angular_velocity));
    pub.rear.right.publish(Float64(wheel_angular_velocity));
    pub.steering.left.publish(Float64(left_steering_angle));
    pub.steering.right.publish(Float64(right_steering_angle));
}


void ROS_Callback_cmd_ackermann(const ackermann_msgs::AckermannDrive::ConstPtr& msg, GazeboWheelPublishers& pub)
{
    ros::Time current_time = ros::Time::now();

    // Convert cmd_vel command into actuation commends sent to the Gazebo simulated wheel controllers
    double speed = msg->speed;
    double steering_angle = msg->steering_angle;

    double wheel_angular_velocity = speed / wheel_radius;

    if (std::abs(steering_angle) < min_steering) steering_angle = 0;
    else if (steering_angle > max_steering) steering_angle = max_steering;
    else if (steering_angle < -max_steering) steering_angle = -max_steering;

    ROS_INFO("Requested velocity %2.2f and steering %2.2f", speed, steering_angle);

    // Use Kinematic Bicycle model with rear-axis reference point
    // https://www.coursera.org/lecture/intro-self-driving-cars/lesson-2-the-kinematic-bicycle-model-Bi8yE
    // Steering angle: s
    // Front to rear distance: L
    // Turning radius: R
    // tan(s) = L/R
    double R = front_to_rear_wheel_distance / std::tan(std::abs(steering_angle));

    // Apply Ackerman steering model to get left and right wheel steering angles
    // http://datagenetics.com/blog/december12016/index.html
    double left_steering_angle, right_steering_angle;
    if (std::signbit(steering_angle) == 0) {
        left_steering_angle = std::atan(front_to_rear_wheel_distance / (R - left_to_right_wheel_distance / 2.0));
        right_steering_angle = std::atan(front_to_rear_wheel_distance / (R + left_to_right_wheel_distance / 2.0));
    } else {
        left_steering_angle = -std::atan(front_to_rear_wheel_distance / (R + left_to_right_wheel_distance / 2.0));
        right_steering_angle = -std::atan(front_to_rear_wheel_distance / (R - left_to_right_wheel_distance / 2.0));
    }

    pub.front.left.publish(Float64(wheel_angular_velocity));
    pub.front.right.publish(Float64(wheel_angular_velocity));
    pub.rear.left.publish(Float64(wheel_angular_velocity));
    pub.rear.right.publish(Float64(wheel_angular_velocity));
    pub.steering.left.publish(Float64(left_steering_angle));
    pub.steering.right.publish(Float64(right_steering_angle));
}

void publishWorldGroundTruth(const ros::TimerEvent& event, tf::TransformListener& tfListener, tf::TransformBroadcaster& tfBroadcaster)
{
    ros::Time current_time = ros::Time::now();
    std::string world_frame = tf::resolve(tfPrefix, "world");
    std::string odom_frame = tf::resolve(tfPrefix, "odom");
    std::string footprint_frame = tf::resolve(tfPrefix, "footprint");

    // Lookup
    tf::Vector3 odom_Translation = tf::Vector3(0,0,0);
    tf::Quaternion odom_Quaternion = tf::Quaternion::getIdentity();
    try{
        tf::StampedTransform transformStatic;
        tfListener.lookupTransform(odom_frame, footprint_frame, ros::Time(0), transformStatic);
        odom_Quaternion = transformStatic.getRotation();
        odom_Translation = transformStatic.getOrigin();
    }
    catch (tf::TransformException &ex) {
        //ROS_WARN("%s",ex.what());
        odom_Translation = tf::Vector3(0,0,0);
        odom_Quaternion = tf::Quaternion::getIdentity();
    }

    if (true_timestamp.isValid()) {
        // Construct the true transform of footprint to world (footprint frame represented in world frame)
        tf::Transform footprint_world;
        footprint_world.setIdentity();
        footprint_world.setOrigin(tf::Vector3(true_pose.position.x,
                true_pose.position.y,
                true_pose.position.z));
        footprint_world.setRotation(tf::Quaternion(true_pose.orientation.x,
                true_pose.orientation.y,
                true_pose.orientation.z,
                true_pose.orientation.w));
        //base_link_world.setRotation(tf::createQuaternionFromYaw(yaw));

        // Construct the transform of footprint to odom (footprint frame represented in odom frame)
        tf::Transform footprint_odom;
        footprint_odom.setIdentity();
        footprint_odom.setOrigin(odom_Translation);
        footprint_odom.setRotation(odom_Quaternion);

        // Compute the transform of world frame defined in odometry frame
        // (footprint to odom)  *  (footprint to world)^-1
        // (footprint to odom)  *  (world to footprint)
        // == world to odom
        tf::Transform worldCorrectionTransform = footprint_odom * footprint_world.inverse();
        if (!std::isnan(worldCorrectionTransform.getRotation().w()) && !std::isnan(worldCorrectionTransform.getOrigin().x())) // ensure rotation is valid
            tfBroadcaster.sendTransform(tf::StampedTransform(worldCorrectionTransform, current_time, odom_frame, world_frame));
    }
}

void ROS_Callback_gazebo_states(const gazebo_msgs::LinkStates::ConstPtr& msg)
{
    ros::Time current_time = ros::Time::now();

    for (size_t i = 0; i < msg->name.size(); i++) {
        if (msg->name[i].find("jetsoncar::footprint") != std::string::npos) {
            true_pose = msg->pose[i];
            true_velocity = msg->twist[i];
            true_timestamp = current_time;
        }
    }
}

void publishEncoderMessage(const ros::TimerEvent& event, ros::Publisher& pub_encoders)
{
    jetsoncar_msgs::Encoders encoders_msg;
    encoders_msg.receive_time = encoderTime;
    encoders_msg.mcu_time = 0;
    encoders_msg.steering_angle = steeringAngle;
    encoders_msg.front_angle = angleFront;
    encoders_msg.rear_angle = angleRear;
    encoders_msg.front_velocity = velocityFront;
    encoders_msg.rear_velocity = velocityRear;
    pub_encoders.publish(encoders_msg);
}

int main(int argc, char **argv) {
	std::string nodeName = "driver_simulator_node";
	ros::init(argc, argv, nodeName.c_str());
	ros::NodeHandle n; // default/current namespace node handle
    ros::NodeHandle nParam("~"); // default/current namespace node handle for parameters

    tfPrefix = tf::getPrefixParam(n);
    tf::TransformBroadcaster tfBroadcaster;
    tf::TransformListener tfListener;

    /* Load parameters */
    double publish_rate = 100;
    if (!nParam.getParam("publish_rate", publish_rate)) {
        ROS_WARN_STREAM("Publish rate not set (Parameter: publish_rate). Defaults to: " << publish_rate);
    }
    if (!nParam.getParam("wheel_radius", wheel_radius)) {
        ROS_WARN_STREAM("Wheel radius not set (Parameter: wheel_radius). Defaults to: " << wheel_radius);
    }
    if (!nParam.getParam("front_to_rear_wheel_distance", front_to_rear_wheel_distance)) {
        ROS_WARN_STREAM("Wheel distance not set (Parameter: front_to_rear_wheel_distance). Defaults to: " << front_to_rear_wheel_distance);
    }
    if (!nParam.getParam("left_to_right_wheel_distance", left_to_right_wheel_distance)) {
        ROS_WARN_STREAM("Wheel distance not set (Parameter: front_to_rear_wheel_distance). Defaults to: " << left_to_right_wheel_distance);
    }
    if (!nParam.getParam("min_steering", min_steering)) {
        ROS_WARN_STREAM("Minimum steering angle not set (Parameter: min_steering). Defaults to: " << min_steering);
    }
    if (!nParam.getParam("max_steering", max_steering)) {
        ROS_WARN_STREAM("Maximum steering angle not set (Parameter: max_steering). Defaults to: " << max_steering);
    }

    /* Configure publishers */
    ros::Publisher pub_encoders = n.advertise<jetsoncar_msgs::Encoders>("encoders", 50);
    ros::Publisher pub_gazebo_vel_left_front_wheel = n.advertise<std_msgs::Float64>("/jetsoncar/front_left_wheel_velocity_controller/command", 50);
    ros::Publisher pub_gazebo_vel_right_front_wheel = n.advertise<std_msgs::Float64>("/jetsoncar/front_right_wheel_velocity_controller/command", 50);
    ros::Publisher pub_gazebo_vel_left_rear_wheel = n.advertise<std_msgs::Float64>("/jetsoncar/rear_left_wheel_velocity_controller/command", 50);
    ros::Publisher pub_gazebo_vel_right_rear_wheel = n.advertise<std_msgs::Float64>("/jetsoncar/rear_right_wheel_velocity_controller/command", 50);
    ros::Publisher pub_gazebo_pos_left_steering_hinge = n.advertise<std_msgs::Float64>("/jetsoncar/front_left_hinge_position_controller/command", 50);
    ros::Publisher pub_gazebo_pos_right_steering_hinge = n.advertise<std_msgs::Float64>("/jetsoncar/front_right_hinge_position_controller/command", 50);

    GazeboWheelPublishers pub_gazebo;
    pub_gazebo.front.left = pub_gazebo_vel_left_front_wheel;
    pub_gazebo.front.right = pub_gazebo_vel_right_front_wheel;
    pub_gazebo.rear.left = pub_gazebo_vel_left_rear_wheel;
    pub_gazebo.rear.right = pub_gazebo_vel_right_rear_wheel;
    pub_gazebo.steering.left = pub_gazebo_pos_left_steering_hinge;
    pub_gazebo.steering.right = pub_gazebo_pos_right_steering_hinge;

    /* Configure subscribers */
    ros::Subscriber sub_cmd_vel = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(&ROS_Callback_cmd_vel, _1, pub_gazebo));
    ros::Subscriber sub_cmd_ackermann = n.subscribe<ackermann_msgs::AckermannDrive>("cmd_ackermann", 1, boost::bind(&ROS_Callback_cmd_ackermann, _1, pub_gazebo));
    ros::Subscriber sub_joint_states = n.subscribe<sensor_msgs::JointState>("/jetsoncar/joint_states", 1, boost::bind(&ROS_Callback_joint_states, _1));
    ros::Subscriber sub_gazebo_states = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, boost::bind(&ROS_Callback_gazebo_states, _1));

#if 0 // publisher loop designed with timers instead
    /* Start publisher loop */
	ros::Rate loop_rate(publish_rate);
	while (ros::ok())
	{
        ros::spinOnce(); // walks the callback queue and calls registered callbacks for any outstanding events (incoming msgs, svc reqs, timers)

        publishEncoderMessage(pub_encoders);
        publishWorldGroundTruth(tfListener, tfBroadcaster);
		
		loop_rate.sleep();
	}
#endif

    ros::spinOnce();

    ros::Timer timer1 = n.createTimer(ros::Duration(1.0 / publish_rate), boost::bind(&publishEncoderMessage, _1, boost::ref(pub_encoders)));
    ros::Timer timer2 = n.createTimer(ros::Duration(1.0 / publish_rate), boost::bind(&publishWorldGroundTruth, _1, boost::ref(tfListener), boost::ref(tfBroadcaster)));

    ros::spin();
}