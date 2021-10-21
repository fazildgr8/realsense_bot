#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>

float pose[3];

void object__pose_broadcaster(string object_name){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(1,2,3));
  // transform.setRotation();
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_broadcaster");
  ros::NodeHandle node;
  while (1)
  {
    // pose = {1,2,3};
    object__pose_broadcaster();
  }
  
  ros::spin();
  return 0;
};