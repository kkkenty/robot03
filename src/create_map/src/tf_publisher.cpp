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
    double x = 0.0, y = 0.0, th = 0.0;
    double linear_x = 0.0, angular_z = 0.0;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
  public:
    nav_msgs::Odometry odom;
    void send_odom(); // odom情報を計算
};
void tf_odom::send_odom(){
  
  ros::Time ros_now = ros::Time::now();
  x = (pst.left + pst.right) / 2.0 * cos(th);
  y = (pst.left + pst.right) / 2.0 * sin(th);
  th = (pst.right - pst.left) / 2.0 / d;
  
  transform.setOrigin(tf::Vector3(x, y, 0.0));
  q.setRPY(0, 0, th);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros_now, "odom", "base_link"));
  
  odom.header.stamp = ros_now;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose = setPose(x, y, th);
  odom.twist.twist.linear.x = (vel.right + vel.left) / 2.0;
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = (vel.right - vel.left) / 2.0 / d;
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
  ros::Subscriber vel_sub = nh.subscribe("vel_sensor", 1, getvel);
  ros::Subscriber pst_sub = nh.subscribe("pst_sensor", 1, getpst);
  ros::Publisher odm_pub = nh.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Publisher ctr_pub = nh.advertise<msgs::Motor>("vel_FB", 1);
  ros::Rate loop_rate(FRIQUENCE);
  while (ros::ok())
  {
  
    odom.send_odom();
    odm_pub.publish(odom.odom);
    ctr_pub.publish(vel);

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
