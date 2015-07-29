#include <apriltags_ros/apriltag_detector.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>

void printTransform(tf::Transform transform) {
  tf::Vector3 to = transform.getOrigin();
  ROS_INFO("translation: %.3f %.3f %.3f", to.getX(), to.getY(), to.getZ());
  ROS_INFO("rotation: ");
  tf::Matrix3x3 rot = transform.getBasis();
  for (int i = 0; i < 3; i++) {
    tf::Vector3 v = rot.getRow(i);
    ROS_INFO("%.3f %.3f %.3f", v.getX(), v.getY(), v.getZ());
  }


  tf::Vector3 t = transform*tf::Vector3(0, 0, 0); 
  tf::Quaternion r = transform*tf::Quaternion(0, 0, 0, 1);
  
  ROS_INFO("t1, t2, t3 = %f, %f %f \ntheta, ux, uy, uz = %f, %f, %f, %f",
      t.getX(), t.getY(), t.getZ(), 
      r.getAngle(), r.getAxis().getX(), r.getAxis().getY(), r.getAxis().getZ());
      
}

void tagsCb(const apriltags_ros::AprilTagDetectionArrayConstPtr tags, ros::Publisher& tags_mapped_pub, ros::Publisher& tags_mapped_posearray_pub, tf::Transform& camera_to_map,  std::string& camera, std::string& robot_base, std::string& map) {

  apriltags_ros::AprilTagDetectionArray mapped_tags_msg;
  std::vector<apriltags_ros::AprilTagDetection> mapped_tags;
  geometry_msgs::PoseArray pose_array_msg;
  std::vector<geometry_msgs::Pose> pose_array;
  ros::Time time_now = ros::Time::now(); //provide same time for all tags

//convet coords of tag in camera coords to map coords
  for (size_t i = 0; i < tags->detections.size(); i++) {
    apriltags_ros::AprilTagDetection tag = tags->detections[i];
    //tag0 is the robot
    /*
    if (tag.id == 0) {
      continue;
    }
    */
    //get tag coords in camera frame and apply transform to get map coords
    tf::Pose tag_to_camera;
    tf::poseMsgToTF(tag.pose.pose, tag_to_camera);
    tf::Transform tag_to_map = camera_to_map * tag_to_camera;

    //transform is equal to pose
    geometry_msgs::PoseStamped poseStamped;
    tf::poseTFToMsg(tag_to_map, poseStamped.pose);

    //convert to apriltag format msg 
    poseStamped.header = std_msgs::Header();
    poseStamped.header.stamp = time_now;
    poseStamped.header.frame_id = map; 
    
    apriltags_ros::AprilTagDetection mapped_tag;
    mapped_tag.id = tag.id;
    mapped_tag.pose = poseStamped;
    mapped_tags.push_back(mapped_tag);
    pose_array.push_back(poseStamped.pose);
  }
  //publish apriltag msg and poseArray for visualization
  mapped_tags_msg.detections = mapped_tags;
  tags_mapped_pub.publish(mapped_tags_msg);

  pose_array_msg.poses = pose_array;
  pose_array_msg.header = std_msgs::Header();
  pose_array_msg.header.stamp = time_now;
  pose_array_msg.header.frame_id = map;
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
  ros::Duration timeout(10.0);

  if(!(tl.waitForTransform(camera, robot_tag, ros::Time(0), timeout))) {
      ROS_ERROR("No transform from %s to %s", robot_tag.c_str(), camera.c_str());
  }
  if(!(tl.waitForTransform(map, robot_base, ros::Time(0), timeout))) {
      ROS_ERROR("No transform from %s to %s", robot_base.c_str(), map.c_str());
  }

  //get transfrom from camera to map by referencing tag on robot, and robot's pose
  tf::StampedTransform robottag_to_camera, robotbase_to_map;
  tl.lookupTransform(camera, robot_tag, ros::Time(0), robottag_to_camera);//tag on robot
  tl.lookupTransform(map, robot_base, ros::Time(0), robotbase_to_map);//robot's pose
 // tf::Transform camera_to_map = robot_base_to_map * robot_tag_to_camera.inverse();
  /*
  ROS_INFO("robot to camera transform");
  printTransform(robot_tag_to_camera);
  ROS_INFO("camera to robot transform");
  printTransform(robot_tag_to_camera.inverse());
  ROS_INFO("robot to map transform");
  printTransform(robot_base_to_map);
*/
  tf::Transform camera_to_robottag = robottag_to_camera.inverse();

  //aright tag which is oriented 180 degs about x axis
 /* 
  tf::Transform x_180 = tf::Transform(tf::Quaternion(1, 0, 0, 0)); 
  tf::Transform y_90 = tf::Transform(tf::Quaternion(0, 0.7071, 0, 0.7071));
*/
  tf::Transform z_minus_90 = tf::Transform(tf::Quaternion(0, 0, -0.7071, 0.7071));
  
  tf::Matrix3x3 robottag_to_robotbase_rot(  
      0, 0, 1.0,
     1.0, 0, 0,
      0, 1.0, 0);
  tf::Transform robottag_to_robotbase(robottag_to_robotbase_rot);
 // robottag_to_robotbase = z_minus_90 * robottag_to_robotbase;
  

 // tf::Transform robot_tag_to_base = y_90 * z_90 * x_180;

  //tf::Quaternion q; 
  //q.setRPY(3.1416, 1.5708, 1.5708);
 // tf::Transform robot_tag_to_base = tf::Transform(q);

 // tf::Transform camera_to_map = robot_base_to_map * (robot_tag_to_base * camera_to_robot_tag); 
  tf::Transform camera_to_map = z_minus_90 * robotbase_to_map * robottag_to_robotbase * camera_to_robottag;
  
  ROS_INFO("camera to map transform");
  printTransform(camera_to_map);

  //subscribe to tag detections 
  ROS_INFO("subscribing to %s for tag detections", tag_detections_topic.c_str());
  //init publisher apriltag msg with pose in map coords and a poseArray for visualization
  ros::Publisher tags_pub = nh.advertise<apriltags_ros::AprilTagDetectionArray>("/tag_detections_mapped", 1);
  ros::Publisher tags_posearray_pub = nh.advertise<geometry_msgs::PoseArray>("/tag_detections_mapped_pose", 1);

  //init subscriber to tag detections in camera coords and pass publishers to callback
  ros::Subscriber tags_sub = nh.subscribe<apriltags_ros::AprilTagDetectionArray>(tag_detections_topic, 1, boost::bind(tagsCb, _1, tags_pub, tags_posearray_pub, camera_to_map, camera, robot_base, map)); 

  ros::spin();

}
