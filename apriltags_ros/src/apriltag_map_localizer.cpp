#include <apriltags_ros/apriltag_detector.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

/** This node localizes the apriltags coordinates onto a map that the Scarab is in
  * It assumes tag 0 is on the Scarab, and subscribes to the Scarab's pose.
  */
int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_map_localizer");
  ros::NodeHandle nh;

  tf::StampedTransform camera_to_robot;
  std::string camera, robot_tag;
  nh.param("camera", camera, std::string("camera"));
  nh.param("robot_tag", robot_tag, std::string("scarab"));
  ros::Duration timeout(5.0);
  tf::TransformListener tl(nh);
  if(!(tl.waitForTransform(camera, robot_tag, ros::Time(0), timeout))) {
      ROS_ERROR("No transform between %s and %s", camera.c_str(), robot_tag.c_str());
  }
  tl.lookupTransform(camera, robot_tag, ros::Time(0), 
    camera_to_robot);
  tf::Transform robot_to_camera = camera_to_robot.inverse();
  tf::Vector3 camera_position = robot_to_camera(tf::Vector3(0, 0, 0));
  tf::Quaternion camera_orientation = robot_to_camera * tf::Quaternion(0, 0, 0, 1); 
  ROS_INFO("camera_positoin: %f, %f, %f, ori: %f, %f, %f, %f", camera_position.getX(), camera_position.getY(), camera_position.getZ(), camera_orientation.getAngle(), camera_orientation.getAxis().getX(), camera_orientation.getAxis().getY(), camera_orientation.getAxis().getZ());
  
  ros::spin();

}
