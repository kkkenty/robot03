#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <custom_msgs/motor_pwm.h>

custom_msgs::motor_pwm PWM;
int MAX_L = 255, MAX_R = 255;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  PWM.L = MAX_L * joy_msg.axes[1];
  PWM.R = MAX_R * joy_msg.axes[2];
  if(joy_msg.buttons[0]){
    PWM.L = 0; PWM.R = 0;
  }
  //ROS_INFO("PWM.L:%d", (int)PWM.L);
  //ROS_INFO("PWM.R:%d", (int)PWM.R);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("MAX_L", MAX_L);
  pnh.getParam("MAX_R", MAX_R);
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
