#include "realsense_t265_interface.hpp"

#include "ros/param.h"

geometry_msgs::TransformStamped getTransformation(const std::string& _frame_id,
                                                  const std::string& _child_frame_id,
                                                  double _translation_x, double _translation_y,
                                                  double _translation_z, double _roll,
                                                  double _pitch, double _yaw) {
  geometry_msgs::TransformStamped transformation;

  transformation.header.frame_id = _frame_id;
  transformation.child_frame_id = _child_frame_id;
  transformation.transform.translation.x = _translation_x;
  transformation.transform.translation.y = _translation_y;
  transformation.transform.translation.z = _translation_z;
  tf2::Quaternion q;
  q.setRPY(_roll, _pitch, _yaw);
  transformation.transform.rotation.x = q.x();
  transformation.transform.rotation.y = q.y();
  transformation.transform.rotation.z = q.z();
  transformation.transform.rotation.w = q.w();

  return transformation;
}

void RealsenseT265Interface::setUp() {
  // ros_utils_lib::getPrivateParam<std::string>("~namespace"					,
  // n_space_
  // ,"drone1"); ros_utils_lib::getPrivateParam<std::string>("~estimated_pose_topic" 	    ,
  // estimated_pose_topic_ 			,"self_localization/pose");
  // ros_utils_lib::getPrivateParam<std::string>("~estimated_speed_topic" 	    ,
  // estimated_speed_topic_ 			,"self_localization/speed");
  // ros_utils_lib::getPrivateParam<std::string>("~realsense_odom_topic" 	    ,
  // realsense_odom_topic_ 			,"realsense_t265/odom/sample");

  ros::param::get("~namespace", n_space_);
  ros::param::get("~estimated_pose_topic", estimated_pose_topic_);
  ros::param::get("~estimated_speed_topic", estimated_speed_topic_);
  ros::param::get("~estimated_odom_topic", estimated_odom_topic_);
  ros::param::get("~realsense_odom_topic", realsense_odom_topic_);

  tf2_fix_transforms_.clear();
  tf2_fix_transforms_.emplace_back(getTransformation("map", "odom", 0, 0, 0, 0, 0, 0));
  tf2_fix_transforms_.emplace_back(
      getTransformation("odom", "odom_rs", RS_OFFSET_X, RS_OFFSET_Y, RS_OFFSET_Z, 0, 0, 0));
  tf2_fix_transforms_.emplace_back(getTransformation("base_link_rs", "base_link_rs_NED", 0, 0, 0,
                                                     RS_OFFSET_R, RS_OFFSET_P, RS_OFFSET_YAW));
  tf2_fix_transforms_.emplace_back(getTransformation("base_link_rs_NED", "base_link", -RS_OFFSET_X,
                                                     -RS_OFFSET_Y, -RS_OFFSET_Z, 0, 0, 0));

  odom_rs_to_base_link_rs_transform_.header.frame_id = "odom_rs";
  odom_rs_to_base_link_rs_transform_.child_frame_id = "base_link_rs";
  odom_rs_to_base_link_rs_transform_.transform.rotation.w = 1.0f;
  tf2_listener_ptr_ = std::make_unique<tf2_ros::TransformListener>(tf2_buffer_);
}

void RealsenseT265Interface::start() {
  odom_sub_ = nh_.subscribe("/" + n_space_ + "/" + realsense_odom_topic_, 1,
                            &RealsenseT265Interface::odometryCallback, this);
  pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>("/" + n_space_ + "/" + estimated_pose_topic_, 1);
  twist_pub_ =
      nh_.advertise<geometry_msgs::TwistStamped>("/" + n_space_ + "/" + estimated_speed_topic_, 1);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/" + n_space_ + "/" + estimated_odom_topic_, 1);
}

void RealsenseT265Interface::stop() {
  odom_sub_.shutdown();
  pose_pub_.shutdown();
  twist_pub_.shutdown();
}

void RealsenseT265Interface::publishEstimatedPose() {
  geometry_msgs::TwistStamped odom_twist;

  geometry_msgs::Vector3 vio_twist_linear_vect;
  geometry_msgs::Vector3 vio_twist_angular_vect;

  geometry_msgs::Vector3 odom_twist_linear_vect;
  geometry_msgs::Vector3 base_link_twist_angular_vect;

  auto timestamp = ros::Time::now();

  vio_pose_.header.stamp = timestamp;
  vio_twist_.header.stamp = timestamp;
  odom_twist.header.stamp = timestamp;

  vio_pose_.header.frame_id = "odom";
  odom_twist.header.frame_id = "odom";

  vio_twist_linear_vect = vio_twist_.twist.linear;
  vio_twist_angular_vect = vio_twist_.twist.angular;

  try {
    // obtain mav pose in odom frame
    auto pose_transform = tf2_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    vio_pose_.pose.position.x = pose_transform.transform.translation.x;
    vio_pose_.pose.position.y = pose_transform.transform.translation.y;
    vio_pose_.pose.position.z = pose_transform.transform.translation.z;
    vio_pose_.pose.orientation = pose_transform.transform.rotation;

    // obtain mav linear speed in odom frame
    auto linear_twist_rotation_transform =
        tf2_buffer_.lookupTransform("odom", "base_link_rs", ros::Time(0));
    // to avoid magnitud changes in speed, translation vector is fixed to [0,0,0]
    linear_twist_rotation_transform.transform.translation.x = 0.0f;
    linear_twist_rotation_transform.transform.translation.y = 0.0f;
    linear_twist_rotation_transform.transform.translation.z = 0.0f;

    tf2::doTransform(vio_twist_linear_vect, odom_twist_linear_vect,
                     linear_twist_rotation_transform);
    odom_twist.twist.linear = odom_twist_linear_vect;

    // obtain mav angular speed in base_link frame
    auto angular_twist_rotation_transform =
        tf2_buffer_.lookupTransform("base_link_rs_NED", "base_link_rs", ros::Time(0));
    tf2::doTransform(vio_twist_angular_vect, base_link_twist_angular_vect,
                     angular_twist_rotation_transform);
    odom_twist.twist.angular = base_link_twist_angular_vect;

    // publish pose and twist
    pose_pub_.publish(vio_pose_);
    twist_pub_.publish(odom_twist);

    // generate odom message using the same pose and twist but using speed in base_link frame
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    odom_msg.pose.pose = vio_pose_.pose;
    odom_msg.twist.twist.angular = odom_twist.twist.angular;
    // tranform speed from odom to base_link frame
    auto linear_transform_from_odom_to_base_link =
        tf2_buffer_.lookupTransform("odom", "base_link", ros::Time(0));
    tf2::doTransform(odom_twist.twist.linear, odom_msg.twist.twist.linear,
                     linear_transform_from_odom_to_base_link);

    odom_pub_.publish(odom_msg);

  } catch (tf2::TransformException& ex) {
    ROS_WARN("Failure %s\n", ex.what());  // Print exception which was caught
  }
}

void RealsenseT265Interface::odometryCallback(const nav_msgs::Odometry& _msg) {
  vio_pose_.pose = _msg.pose.pose;
  vio_twist_.twist = _msg.twist.twist;

  odom_rs_to_base_link_rs_transform_.transform.translation.x = _msg.pose.pose.position.x;
  odom_rs_to_base_link_rs_transform_.transform.translation.y = _msg.pose.pose.position.y;
  odom_rs_to_base_link_rs_transform_.transform.translation.z = _msg.pose.pose.position.z;
  odom_rs_to_base_link_rs_transform_.transform.rotation = _msg.pose.pose.orientation;
}
