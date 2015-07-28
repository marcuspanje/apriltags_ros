#include <apriltags_ros/apriltag_detector.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>




void printTransforms(tf::Transform transform) {
  tf::Vector3 t = transform*tf::Vector3(0, 0, 0); 
  tf::Quaternion r = transform*tf::Quaternion(0, 0, 0, 1);
  
  ROS_INFO("t1, t2, t3 = %f, %f %f \ntheta, ux, uy, uz = %f, %f, %f, %f",
      t.getX(), t.getY(), t.getZ(), 
      r.getAngle(), r.getAxis().getX(), r.getAxis().getY(), r.getAxis().getZ());
      
}

void tagsCb(AprilTagDetectionArrayConstPtr tags, ros::Publisher& pub, tf::TransformListener tl, tf::Transform& camera_to_map, std::string camera, std::string robot_base) {

  AprilTagDetectionArray mapped_tags_msg;
  std::vector<AprilTagDetection> mapped_tags;
  mapped_tags_msg.header = Header h(); 
  mapped_tags_msg.header.time = ros::Time::now();

//get coords of map with in relation to map
  for (AprilTagDetection tag : tags) {
    std::string tag_string = "tag_" + tag.id;
    tf::Transform tag_to_camera;
    tl.lookupTransform(camera, tag_string, ros::Time(0), tag_to_camera);        
    tf::Transform tag_to_map = camera_to_map * tag_to_camera; 
    geometry_msgs::Pose pose;
    tf::poseTFToMsg(tag_to_map, pose); 
    AprilTagDetection mapped_tag;
    mapped_tag.id = id;
    mapped_tag.pose = pose;
    mapped_tags.push_back(mapped_tag);
  }

  mapped_tags_msg.detections = mapped_tags;

  pub(mapped_tags_msg);

}


/** This node localizes the apriltags coordinates onto a map that the Scarab is in
  * It assumes tag 0 is on the Scarab, and subscribes to the Scarab's pose.
  */
int main(int argc, char **argv){
  ros::init(argc, argv, "apriltag_map_localizer");
  ros::NodeHandle nh;

  //load parameters
  std::string camera, robot_tag, robot_base, map, tag_detections_topic;
  nh.param("camera", camera, std::string("camera"));
  nh.param("robot_tag", robot_tag, std::string("scarab"));
  nh.param("map", map, std::string("map_hokuyo"));
  nh.param("map", robot_base, std::string("base_link"));
  nh.param("tag_detections_topic", tag_detections_topic, std::string("tag_detections"));

  //transform X_to_Y means X's frame in terms of Y's frame
  tf::StampedTransform robot_to_camera_tag;
  tf::StampedTransform tag1_to_camera;
  tf::StampedTransform tag3_to_camera;

  ros::Duration timeout(5.0);
  tf::TransformListener tl(nh);
  if(!(tl.waitForTransform(camera, robot_tag, ros::Time(0), timeout))) {
      ROS_ERROR("No transform between %s and %s", camera.c_str(), robot_tag.c_str());
  }
  if(!(tl.waitForTransform(map, robot_base, ros::Time(0), timeout))) {
      ROS_ERROR("No transform between %s and %s", map.c_str(), robot_base.c_str());
  }

  tl.lookupTransform(camera, robot_tag, ros::Time(0), robot_tag_to_camera);
  tl.lookupTransform(map, robot_base, ros::Time(0), robot_base_to_map);

  tl.lookupTransform(camera, "tag_1", ros::Time(0), tag1_to_camera);
  tl.lookupTransform(camera, "tag_3", ros::Time(0), tag3_to_camera);

  tf::Transform camera_to_robot_tag = robot_tag_to_camera.inverse();

  tf::Transform right_robot_tag = tf::Transform(tf::Quaternion(1, 0, 0, 0));//180 about x 
  tf::Transform 90_about_y = tf::Transform(tf::Quaternion(0, 0.7071, 0, 0.7071));
  tf::Transform 90_about_z = tf::Transform(tf::Quaternion(0, 0, 0.7071, 0.7071));
  tf::Transform robot_tag_to_base = 90_about_y * 90_about_z * right_robot_tag;

  tf::Transform camera_to_map = camera_to_robot_tag * robot_tag_to_base * robot_base_to_map; 

  //subscribe to tag detectoins 
  ros::Publisher tags_pub = nh.advertise<AprilTagDetectionArray>("tag_detections_mapped", 1);
  ros::Subscriber tags_sub = nh.subscribe<AprilTagDetectionArray>(tag_detections_topic, 1, tl, boost::bind(tagsCb, _1, tags_pub, camera_to_map, camera, robot_base)); 

  tf::Transform tag1_to_robot = camera_to_robot*tag1_to_camera;
  tf::Transform tag3_to_robot = camera_to_robot*tag3_to_camera;

  
 
  printTransforms(camera_to_robot);
  printTransforms(tag1_to_robot);
  printTransforms(tag3_to_robot);

  ros::spin();

}
