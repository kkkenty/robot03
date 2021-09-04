#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <custom_msgs/motor_pwm.h>

custom_msgs::motor_pwm PWM;
double v = 0, w = 0, k_PWM = 200.0, k_v = 1.0, k_w = 1.0, d = 0.135;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  v = joy_msg.axes[1]; w = joy_msg.axes[3];
  PWM.L = k_PWM * ((k_v * v) - (k_w * w * d));
  PWM.R = k_PWM * ((k_v * v) + (k_w * w * d));
  if(joy_msg.buttons[0]){
    PWM.L = 0; PWM.R = 0;
  }
  ROS_INFO("PWM.L:%lf", (double)PWM.L);
  ROS_INFO("PWM.R:%lf", (double)PWM.R);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("k_PWM", k_PWM);
  pnh.getParam("k_v", k_v);
  pnh.getParam("k_w", k_w);
  pnh.getParam("d", d);
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);
  ros::Publisher pub = nh.advertise<custom_msgs::motor_pwm>("power", 1);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    pub.publish(PWM);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
