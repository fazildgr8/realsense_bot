#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>

std_msgs::Int8 count;
std::vector<float> object_pose[3];
std::string object_class = "bottle";

void object_pose_broadcaster(float pose[3],std::string object_name){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose[0],pose[1],pose[2]));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", object_name));
}

std::vector<std_msgs::Int64> ImgCenter(int xmin,int ymin,int xmax,int ymax){
                // float xmin_t = xmin.data;
                // float ymin_t = ymin.data;
                // float xmax_t = xmax.data;
                // float ymax_t = ymax.data;
                std_msgs::Int64 x;
                x.data = xmax - (xmax-xmin)/2;
                std_msgs::Int64 y;
                y.data = ymax - (ymax-ymin)/2;
                std::vector<std_msgs::Int64> loc;
                loc.push_back(x);
                loc.push_back(y);
                return loc;
}

void boundingboxes_cllbk(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

  if(count.data > 0){

    for (int i=0; i<msg->bounding_boxes.size(); ++i)
    {
      const darknet_ros_msgs::BoundingBox &data = msg->bounding_boxes[i];
      std::vector<std_msgs::Int64> loc = ImgCenter(data.xmin,data.ymin,data.xmax,data.ymax);
      ROS_INFO_STREAM("Id: "<<data.id <<" Class: " << data.Class 
                      << " Pribability: " << data.probability <<" Pixel Loc: ("<<loc[0].data<<","<<loc[1].data<<")");
    }
  }
  

}



void foundobjects_cllbk(const darknet_ros_msgs::ObjectCount::ConstPtr& msg){
  count.data = msg->count;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "objects_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber boxes_sub = node.subscribe("/darknet_ros/bounding_boxes", 1000, boundingboxes_cllbk);
  ros::Subscriber boxcount_sub = node.subscribe("/darknet_ros/found_object", 1000, foundobjects_cllbk);
  ros::Publisher count_pub = node.advertise<std_msgs::Int8>("count", 1000);
  ros::Rate loop_rate(10);
  float i = 0;
  while (ros::ok())
  {
  float pose[3] = {0,0,i};
  object_pose_broadcaster(pose,"object");
  count_pub.publish(count);
  i = i + 0.01;
  if(i>3){
    i = 0;
  }
    // ROS_INFO("Objects Count: [%d]", count);
    ros::spinOnce();
  }
  
  return 0;
}