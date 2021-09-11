// joystickから目標速度、Arduinoから現在速度をsubし、PWM値をpubする
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <msgs/Motor.h>
#include <msgs/PID.h>
//-------------------
// global変数
//-------------------
msgs::Motor control;
msgs::PID param;
const float LIMIT = 0.15, ACC = 1;
float kp = 25.0, ki = 2.0, kd = 0.1; 
float v = 0.0, w = 0.0, kv = 5.0, kw = 10.0, d = 0.2;
int state = 0, FRIQUENCE = 20;
//------------------
// クラスの定義
//------------------
class feedback
{
  private:
    float SPEED_SUM = 0.0, SPEED_ACC = 0.0, SPEED_ERROR = 0.0, SPEED_ERROR_PRE = 0.0, SPEED_PRE = 0.0;
  public:
    float PWM = 0.0, SPEED_GOAL = 0.0, SPEED_NOW = 0.0;
    float PID(); // PID制御でPWM計算
    float daikei(); // 台形制御でPWM計算
};
float feedback::PID(){
  SPEED_ERROR = SPEED_GOAL - SPEED_NOW; // P項 or 偏差
  SPEED_SUM += (SPEED_ERROR + SPEED_ERROR_PRE) / 2.0; // I項
  SPEED_ACC = (float)(SPEED_NOW - SPEED_PRE) * (float)FRIQUENCE; // D項
  //ROS_INFO("lf", SPEED_ACC);
  param.p = kp * SPEED_ERROR;
  param.i = ki * ki * SPEED_SUM;
  param.d = kd * SPEED_ACC;
  PWM = param.p + param.i - param.d;
  SPEED_ERROR_PRE = SPEED_ERROR;
  SPEED_PRE = SPEED_NOW;
  return PWM;
}
float feedback::daikei(){
  SPEED_ERROR = SPEED_GOAL - SPEED_NOW; // P項 or 偏差
  if(SPEED_ERROR > LIMIT){
    PWM += ACC;
  }
  else if(SPEED_ERROR < -LIMIT){
    PWM -= ACC;
  }
  else{ // -LIMIT <= SPEED_ERROR[i] <= LIMIT
  }
  return PWM;
}
// -----------------
// クラスの宣言
//------------------
feedback LEFT, RIGHT;
//------------------
// CallBack関数の定義
//------------------
void joyCb(const sensor_msgs::Joy &joy_msg)
{
  v = kv * joy_msg.axes[1]; w = kw * joy_msg.axes[3];
  LEFT.SPEED_GOAL  = v - (w * d);
  RIGHT.SPEED_GOAL = v + (w * d);
  if(joy_msg.buttons[0]){
    state++;
    if(state % 2 == 1){
      ROS_INFO("STOPPING!");
    }
    else if(state % 2 == 0){
      ROS_INFO("RESTARTING!");
    }
  }
  //ROS_INFO("LEFT.SPEED_GOAL:%lf", (double)PWM.L);
  //ROS_INFO("RIGHT.SPEED_GOAL:%lf", (double)PWM.R);
}
void ardCb(const msgs::Motor &sensor)
{
  LEFT.SPEED_NOW  = sensor.left;
  RIGHT.SPEED_NOW = sensor.right;
}
//---------------------
// main関数
//---------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller3");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.getParam("kv", kv);
  pnh.getParam("kw", kw);
  pnh.getParam("d", d);
  pnh.getParam("kp", kp);
  pnh.getParam("ki", ki);
  pnh.getParam("kd", kd);
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
  ros::Subscriber ard_sub = nh.subscribe("speed", 1, ardCb);	
  ros::Publisher ard_pub = nh.advertise<msgs::Motor>("control", 1);
  ros::Publisher scr_pub = nh.advertise<msgs::PID>("param", 1);
  ros::Rate loop_rate(FRIQUENCE);
  
  while (ros::ok())
  {
    control.left  = LEFT.PID();
    control.right = RIGHT.PID();
    if(state % 2 == 1){
      control.left  = 0.0;
      control.right = 0.0;
    }
    ard_pub.publish(control);
    scr_pub.publish(param);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
