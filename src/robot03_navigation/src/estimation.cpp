#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

sensor_msgs::LaserScan scan;
const double pi = 3.1415926535, diag = 6.0573;
const int w_den = 180, p_den = 180; // 量子化誤差3cm

void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg){
  scan = *msg;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "estimation");
  ros::NodeHandle nh;
  ros::Subscriber sub_urg;
  sub_urg = nh.subscribe("scan", 5, cbScan);
  
  int i, j, k, wp[w_den][p_den]={};
  double theta = 0.0, x = 0.0, y = 0.0, w, p;
  ros::Rate rate(20);
  
  for(i=0;i<w_den;i++){
    for(j=0;j<p_den;j++){
      wp[i][j] = 0;
    }
  }
  
  while(ros::ok()){
    ros::spinOnce();
    
    if(scan.ranges.size() > 0){ // msgがある場合
      for(i=0; i<scan.ranges.size(); i++){
        if(!(scan.ranges[i] < scan.range_min || 
             scan.ranges[i] > scan.range_max || 
             std::isnan(scan.ranges[i]))){
          alpha = scan.angle_min + (double)i * scan.angle_increment;
          x = scan.ranges[i] * cos(theta);
          y = scan.ranges[i] * sin(theta);
          
          for(j=0; j<w_den; j++){
            w = pi*(double)j/(double)den;
            p = x*cos(w) + y*sin(w);
            k = (int)round(p/diag*(double)p_den);
            wp[j][k]++;
          }
        }
      }     
    }
    
    for(i=0;i<w_den;i++){
      for(j=0;j<p_den;j++){
        if(max < wp[i][j]){
          max = wp[i][j];
          w = pi*(double)i/(double)den;
          p = x*cos(w) + y*sin(w);
      }
    }
    
  }
  return 0;
}
