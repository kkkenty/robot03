#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <cmath> // M_PI
#include <visualization_msgs/Marker.h>

sensor_msgs::LaserScan scan;
const double diag = 6.0573;

void cbScan(const sensor_msgs::LaserScan::ConstPtr &msg){
  scan = *msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "estimation");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber urg_sub = nh.subscribe("scan", 5, cbScan);
  ros::Publisher hough_pub = nh.advertise<visualization_msgs::Marker>("hough", 10);
  int w_den = 180, p_den = 180; // 量子化誤差3cm
  //int kmean = 3;
  
  int i, j, k, wp[w_den][p_den];
  double x = 0.0, y = 0.0, theta = 0.0, w, p;
  geometry_msgs::Point path;
  ros::Rate rate(20);
  
  while(ros::ok()){
    ros::spinOnce();
    
    // wp領域の初期化 //
    for(i=0;i<w_den;i++){
      for(j=0;j<p_den;j++){
        wp[i][j] = 0;
      }
    }

    // lineの初期化 //
    visualization_msgs::Marker line;
    line.header.frame_id = "base_laser";
    line.header.stamp = ros::Time::now();
    line.ns = "hough";
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.id = 3;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.scale.x = 0.01; 
    line.color.r = line.color.a = 1.0;
    
    // センサ情報からwp領域に変換 //
    if(scan.ranges.size() > 0){ // msgがある場合
      for(i=0; i<scan.ranges.size(); i++){
        if(!(scan.ranges[i] < scan.range_min || 
             scan.ranges[i] > scan.range_max || 
             std::isnan(scan.ranges[i]))){
          theta = scan.angle_min + (double)i * scan.angle_increment;
          x = scan.ranges[i] * cos(theta);
          y = scan.ranges[i] * sin(theta);
          
          for(j=0; j<w_den; j++){
            w = M_PI * (double)j / (double)w_den;
            p = x * cos(w) + y * sin(w);
            k = (int)round(p / diag * (double)p_den);
            wp[j][k]++;
          }
        }
      }
    }
    
    // 最大値のパラメータを検出 //
    int max = 0;
    for(i=0;i<w_den;i++){
      for(j=0;j<p_den;j++){
        if(max < wp[i][j]){
          max = wp[i][j];
          w = M_PI * (double)i / (double)w_den;
          p = diag * (double)j / (double)p_den;
        }
      }
    }
    
    // 直線データに変換 //
    double len = 3.0, px[3], py[3];
    for(i=0;i<3;i++){
      if(!(w == 0 || w == M_PI)){
        px[i] = len - (len * (double)i);
        py[i] = p / sin(w) - px[i] * cos(w) / sin(w);
      }
      else{
        py[i] = len - (len * (double)i);
        px[i] = p / cos(w) - py[i] * sin(w) / cos(w);
      }
      path.x = px[i];
      path.y = py[i];
      line.points.push_back(path);
    }
    
    // pub //
    hough_pub.publish(line);
  }
  return 0;
}
