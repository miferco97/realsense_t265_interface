#pragma once
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include <string>
#include <memory>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "ros_utils_lib/ros_utils.hpp"
#include "ros_utils_lib/control_utils.hpp"
#include "robot_process.h"

#include "tf/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define RS_OFFSET_X 0.11f 
#define RS_OFFSET_Y 0.01f
#define RS_OFFSET_Z 0.0f
#define RS_OFFSET_R 0.0f
// #define RS_OFFSET_P (-45.0f/180.0f *M_PI)
#define RS_OFFSET_P (-47.0f/180.0f *M_PI)
#define RS_OFFSET_YAW 0.0f

#define DEBUG 1

class RealsenseT265Interface: public RobotProcess{
public:
    RealsenseT265Interface(){};
private:

	std::string n_space_;
	std::string estimated_pose_topic_;
	std::string estimated_speed_topic_;
	std::string realsense_odom_topic_;
	
    tf2_ros::TransformBroadcaster tf2_broadcaster_;
    tf2_ros::Buffer tf2_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener_ptr_;


    // tf2_ros::StaticTransformBroadcaster tf2_static_broadcaster_;
    std::vector<geometry_msgs::TransformStamped> tf2_fix_transforms_;
    geometry_msgs::TransformStamped odom_rs_to_base_link_rs_transform_;

    ros::NodeHandle nh_;

    ros::Publisher pose_pub_;
    ros::Publisher twist_pub_;
    ros::Subscriber odom_sub_;

    void odometryCallback(const nav_msgs::Odometry & );
    geometry_msgs::PoseStamped vio_pose_; 
    geometry_msgs::TwistStamped vio_twist_; 

    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun(){
        publishTFs();
        publishEstimatedPose();
        
    }
        
    void publishTFs(){
        auto timestamp = ros::Time::now();

        for (geometry_msgs::TransformStamped transform:tf2_fix_transforms_){
            transform.header.stamp = timestamp;
            tf2_broadcaster_.sendTransform(transform);
        }
        odom_rs_to_base_link_rs_transform_.header.stamp = timestamp;
        tf2_broadcaster_.sendTransform(odom_rs_to_base_link_rs_transform_);

    }
    void publishEstimatedPose();
};

