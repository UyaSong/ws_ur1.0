#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/Vector3.h"

class get_dist_vector{
public:
  get_dist_vector(){
    dist_vector.x = 0;
    dist_vector.y = 0;
    dist_vector.z = 0;
  };

  geometry_msgs::Vector3 dist_vector;

  void registerNodeHandle(ros::NodeHandle& _nh){
    nh = _nh;
  };

  void registerPubSub(){
    vector_pub = nh.advertise<geometry_msgs::Vector3>("/vector_I_get", 1);
    vector_sub = nh.subscribe("/kinect_merge/vector_closest_frame", 1000, &get_dist_vector::chatterCallback, this);
  };
  
  void chatterCallback(const geometry_msgs::Vector3::ConstPtr& msg){
    ROS_INFO("I heard: [%f %f %f]", msg->x, msg->y, msg->z);
    dist_vector.x = msg->x;
    dist_vector.y = msg->y;
    dist_vector.z = msg->z;
    vector_pub.publish(dist_vector);
  }

private:
    ros::Publisher vector_pub;
    ros::Subscriber vector_sub;
    ros::NodeHandle nh;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "get_dist_vector");
  ros::NodeHandle n;
  get_dist_vector s8;

  s8.registerNodeHandle(n);
  s8.registerPubSub();

  ros::spin();
  return 0;
}