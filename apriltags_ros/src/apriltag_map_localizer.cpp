#include <apriltags_ros/apriltag_detector.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

void printTransforms(tf::Transform transform) {
  tf::Vector3 t = transform*tf::Vector3(0, 0, 0); 
  tf::Quaternion r = transform*tf::Quaternion(0, 0, 0, 1);
  
  ROS_INFO("t1, t2, t3 = %f, %f %f \ntheta, ux, uy, uz = %f, %f, %f, %f",
      t.getX(), t.getY(), t.getZ(), 
      r.getAngle(), r.getAxis().getX(), r.getAxis().getY(), r.getAxis().getZ());
      
}

void tagsCb(const apriltags_ros::AprilTagDetectionArrayConstPtr tags, ros::NodeHandle& nh, ros::Publisher& tags_mapped_pub, ros::Publisher& tags_mapped_posearray_pub, tf::Transform& camera_to_map,  std::string& camera, std::string& robot_base, std::string& map) {

  apriltags_ros::AprilTagDetectionArray mapped_tags_msg;
  std::vector<apriltags_ros::AprilTagDetection> mapped_tags;
  geometry_msgs::PoseArray pose_array_msg;
  std::vector<geometry_msgs::Pose> pose_array;

//get coords of map with in relation to map
  for (size_t i = 0; i < tags->detections.size(); i++) {
    apriltags_ros::AprilTagDetection tag = tags->detections[i];
    //tag0 is the robot
    if (tag.id == 0) {
      continue;
    }
    char buffer[10];
    sprintf(buffer, "tag_%u", tag.id);
    std::string tag_string(buffer);

    tf::Pose tag_to_camera;
    tf::poseMsgToTF(tag.pose.pose, tag_to_camera);
    tf::Transform tag_to_map = camera_to_map * tag_to_camera; 
    //tf::StampedTransform tag_to_map_stamped(tag_to_map, ros::Time::now(), map, tag_string); 
    //tf::Stamped<tf::Pose> tag_pose_map(tag_to_map_stamped);

    geometry_msgs::PoseStamped poseStamped;
    tf::poseTFToMsg(tag_to_map, poseStamped.pose);
/*
    tf::Vector3 t = tag_to_map.getOrigin();
    tf::Quaternion q = tag_to_map.getRotation();
    poseStamped.pose.position.x = t.getX();
    poseStamped.pose.position.y = t.getY();
    poseStamped.pose.position.z = 0.0;
    poseStamped.pose.orientation.w = q.getW();
    poseStamped.pose.orientation.x = q.getAxis().getX();
    poseStamped.pose.orientation.y = q.getAxis().getY();
    poseStamped.pose.orientation.z = q.getAxis().getZ();
    */
    poseStamped.header = std_msgs::Header();
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = map; 
    
    apriltags_ros::AprilTagDetection mapped_tag;
    mapped_tag.id = tag.id;
    mapped_tag.pose = poseStamped;
    mapped_tags.push_back(mapped_tag);
    pose_array.push_back(poseStamped.pose);
  }

  mapped_tags_msg.detections = mapped_tags;
  pose_array_msg.poses = pose_array;
  pose_array_msg.header = std_msgs::Header();
  pose_array_msg.header.stamp = ros::Time::now();
  pose_array_msg.header.frame_id = map;

  tags_mapped_pub.publish(mapped_tags_msg);
  tags_mapped_posearray_pub.publish(pose_array_msg);

}


/** This node localizes the apriltags coordinates onto a map that the Scarab is in
  * It assumes tag 0 is on the Scarab, and subscribes to the Scarab's pose.
  */
int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_map_localizer");
  ros::NodeHandle nh("~");

  //load parameters
  std::string camera, robot_tag, robot_base, map, tag_detections_topic;
  nh.param("camera", camera, std::string("camera"));
  nh.param("robot_tag", robot_tag, std::string("scarab"));
  nh.param("map", map, std::string("map_hokuyo"));
  nh.param("robot_base", robot_base, std::string("scarab/base_link"));
  nh.param("tag_detections_topic", tag_detections_topic, std::string("/tag_detections"));

  //transform X_to_Y means X's frame in terms of Y's frame
  tf::TransformListener tl(nh);
  ros::Duration timeout(5.0);

  if(!(tl.waitForTransform(camera, robot_tag, ros::Time(0), timeout))) {
      ROS_ERROR("No transform from %s to %s", robot_tag.c_str(), camera.c_str());
  }
  if(!(tl.waitForTransform(map, robot_base, ros::Time(0), timeout))) {
      ROS_ERROR("No transform from %s to %s", robot_base.c_str(), map.c_str());
  }

  //get transfrom from map to camera by referencing tag on robot, and robot's pose
  tf::StampedTransform robot_tag_to_camera, robot_base_to_map;
  tl.lookupTransform(camera, robot_tag, ros::Time(0), robot_tag_to_camera);//tag on robot
  tl.lookupTransform(map, robot_base, ros::Time(0), robot_base_to_map);//robot's pose
  tf::Transform camera_to_robot_tag = robot_tag_to_camera.inverse();

  //aright tag which is oriented 180 degs about x axis
  
  tf::Transform x_180 = tf::Transform(tf::Quaternion(1, 0, 0, 0)); 
  tf::Transform y_90 = tf::Transform(tf::Quaternion(0, 0.7071, 0, 0.7071));
  tf::Transform z_90 = tf::Transform(tf::Quaternion(0, 0, 0.7071, 0.7071));
  tf::Transform robot_tag_to_base = y_90 * z_90 * x_180;
/*
  tf::Quaternion q; 
  q.setRPY(3.1416, 1.5708, 1.5708);
  tf::Transform robot_tag_to_base = tf::Transform(q);
*/
  tf::Transform camera_to_map = camera_to_robot_tag * robot_tag_to_base * robot_base_to_map; 

  //subscribe to tag detections 
  ROS_INFO("subscribing to %s for tag detections", tag_detections_topic.c_str());
  ros::Publisher tags_pub = nh.advertise<apriltags_ros::AprilTagDetectionArray>("/tag_detections_mapped", 1);
  ros::Publisher tags_posearray_pub = nh.advertise<geometry_msgs::PoseArray>("/tag_detections_mapped_pose", 1);
  ros::Subscriber tags_sub = nh.subscribe<apriltags_ros::AprilTagDetectionArray>(tag_detections_topic, 1, boost::bind(tagsCb, _1, nh, tags_pub, tags_posearray_pub, camera_to_map, camera, robot_base, map)); 

  ros::spin();

}
