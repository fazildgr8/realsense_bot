#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
// #include <darknet_ros_msgs/BoundingBox.h>
// #include <darknet_ros_msgs/BoundingBoxes.h>
// #include <darknet_ros_msgs/ObjectCount.h>


void object_pose_broadcaster(float pose[3],std::string object_name){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose[0],pose[1],pose[2]));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object_name));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "objects_broadcaster");
  ros::NodeHandle node;

  float pose[3] = {1,2,3};

  while (ros::ok())
  {
    object_pose_broadcaster(pose,"object");
  }
  ros::spin();
  return 0;
};