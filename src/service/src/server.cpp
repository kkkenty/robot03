#include "ros/ros.h"
#include "service/time.h"

bool tell(service::time::Request &req, 
	 service::time::Response &res){
  res.x = req.t;
  ROS_INFO("getting time: [%lf]", req.t);
  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "server");
  ros::NodeHandle n;
  
  ros::ServiceServer service = n.advertiseService("time", tell);
  ROS_INFO("Ready to get time.");
  ros::spin();
  
  return 0;
}
