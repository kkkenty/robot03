// Arduinoからエンコーダ情報を得て、odom情報をtf,SLAMに配信
#include <ros/ros.h>
#include <msgs/Motor.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
//-------------------
// global変数
//-------------------
int FRIQUENCE = 100;
const double pi = 3.14159265;
double d = 0.155;
msgs::Motor vel, pst;
//------------------
// 関数、Cb関数の定義
//------------------
geometry_msgs::Pose setPose(float x, float y, float yaw){
  geometry_msgs::Pose output;
  output.position.x = x;
  output.position.y = y;
  output.position.z = 0;
  output.orientation.x = 0;
  output.orientation.y = 0;
  output.orientation.z = sin(yaw / 2);
  output.orientation.w = cos(yaw / 2);
  return output;
}
void getvel(msgs::Motor fake_vel){ // 正確なv,wを取得
  vel.left = 0.12 * pi * 1000 / 3292 * fake_vel.left;
  vel.right = 0.12 * pi * 1000 / 3292 * fake_vel.right;
}
void getpst(msgs::Motor fake_pst){ // 正確なdv,dwを取得
  pst.left = 0.12 * pi / 3292 * fake_pst.left;
  pst.right = 0.12 * pi / 3292 * fake_pst.right;
}
//------------------
// クラスの定義
//------------------
class tf_odom
{
  private:
    double x = 0.0, y = 0.0, th = 0.0, r = 0.0, del_l = 0.0, th_half = 0.0, dt = 0.0, V = 0.0, W = 0.0;
    double linear_x = 0.0, angular_z = 0.0;
    tf::TransformBroadcaster br;
    tf::Transform tf_base, tf_laser;
    tf::Quaternion q_base, q_laser;
    ros::Time ros_pre = ros::Time::now();
  public:
    //nav_msgs::Odometry odom;
    
    void get_odom();
    void send_odom(); // odom情報を計算
};
void tf_odom::get_odom(){
  th += (double)(pst.right - pst.left) / 2.0 / d;
  r = (double)(pst.right + pst.left) / (pst.right - pst.left) * d;
  th_half = th / 2;
  del_l = (double)2 * r * sin(th_half);
  x += (double)del_l * cos(th_half);
  y += (double)del_l * sin(th_half);
}
void tf_odom::send_odom(){
  
  ros::Time ros_now = ros::Time::now();
  dt = ros_now.toSec() - ros_pre.toSec();
  //ROS_INFO("ROS::Time: %lf", dt);
  
  //static double fg = 0;
  //fg = (double)ros_now - (double)ros_pre;
  
  /*
  x = (pst.left + pst.right) / 2.0 * cos(th);
  y = (pst.left + pst.right) / 2.0 * sin(th);  
  th = (pst.right - pst.left) / 2.0 / d;
  */
  
  /*
  th += (pst.right - pst.left) / 2.0 / d;
  r = (pst.right + pst.left) / (pst.right - pst.left) * d;
  del_l = 2 * r * sin(th / 2);
  x += del_l * cos(th / 2);
  y += del_l * sin(th / 2);
  */
  
  V = (vel.left + vel.right) / 2.0;
  W = (vel.right - vel.left) / 2.0 / d;
  x += V * dt * cos(th);
  y += V * dt * sin(th);
  th += W * dt;
  
  //ROS_INFO("x:%lf, y:%lf, th:%lf", x, y, th);
  
  tf_base.setOrigin(tf::Vector3(x, y, 0.0));
  q_base.setRPY(0, 0, th);
  tf_base.setRotation(q_base);
  tf_laser.setOrigin(tf::Vector3(-0.07, -0.07, 0.085));
  q_laser.setRPY(0, 0, pi);
  tf_laser.setRotation(q_laser);
  
  br.sendTransform(tf::StampedTransform(tf_base, ros_now, "odom", "base_link"));
  br.sendTransform(tf::StampedTransform(tf_laser, ros_now, "base_link", "base_laser"));
  
  ros_pre = ros_now;  
  
  /*
  odom.header.stamp = ros_now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose = setPose(x, y, th);
  odom.twist.twist.linear.x = (vel.right + vel.left) / 2.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = (vel.right - vel.left) / 2.0 / d;
  */
}
//---------------------
// main関数
//---------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_publisher");
  tf_odom odom;
  vel.left = 0.0; vel.right = 0.0; pst.left = 0.0; pst.right = 0.0; // inisialize
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("d", d);
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  ros::Subscriber vel_sub = nh.subscribe("vel_sensor", 1, getvel);
  ros::Subscriber pst_sub = nh.subscribe("pst_sensor", 1, getpst);
  //ros::Publisher odm_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher ctr_pub = nh.advertise<msgs::Motor>("vel_FB", 1);
  ros::Rate loop_rate(FRIQUENCE);

  while (ros::ok())
  {
  
    //odom.get_odom();
    odom.send_odom();
    //odm_pub.publish(odom.odom);
    ctr_pub.publish(vel);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
