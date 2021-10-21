#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <ros/console.h>


int count = 0;

typedef struct {
  float x, y, w, h, prob;
  int num, Class;
} RosBox_;

void object_pose_broadcaster(float pose[3],std::string object_name){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose[0],pose[1],pose[2]));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object_name));
}

// void boundingboxes_cllbk(const darknet_ros_msgs::BoundingBoxes& msg){
//   std::vector<std::vector<RosBox_> > rosBoxes_;
//   std::vector<int> rosBoxCounter_;
//   rosBoxes_ =  msg.bounding_boxes;

// }

void foundobjects_cllbk(const darknet_ros_msgs::ObjectCount &msg){
  count = msg.count;
  ROS_DEBUG_STREAM("Count" << count);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "objects_broadcaster");
  ros::NodeHandle node;
  // ros::Subscriber sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, boundingboxes_cllbk);
  ros::Subscriber sub = node.subscribe("/darknet_ros/found_object", 1000, foundobjects_cllbk);
  float pose[3] = {1,2,3};

  while (ros::ok())
  {
    object_pose_broadcaster(pose,"object");
    
  }
  ros::spin();
  return 0;
};