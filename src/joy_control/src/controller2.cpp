#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <msgs/Motor.h>

msgs::Motor PWM;
float v = 0.0, w = 0.0, kP = 200.0, kv = 1.0, kw = 1.0, d = 0.135;

void joy_callback(const sensor_msgs::Joy &joy_msg)
{
  v = kv * joy_msg.axes[1]; w = kw * joy_msg.axes[3];
  PWM.left  = kP * (v - (w * d));
  PWM.right = kP * (v + (w * d));
  if(joy_msg.buttons[0]){
    PWM.left = 0.0; PWM.right = 0.0;
  }
  //ROS_INFO("PWM.L:%lf", (double)PWM.L);
  //ROS_INFO("PWM.R:%lf", (double)PWM.R);
  ROS_INFO("Started!");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("kP", kP);
  pnh.getParam("kv", kv);
  pnh.getParam("kw", kw);
  pnh.getParam("d", d);
  ros::Subscriber sub = nh.subscribe("joy", 10, joy_callback);
  ros::Publisher pub = nh.advertise<msgs::Motor>("power", 1);
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    pub.publish(PWM);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}