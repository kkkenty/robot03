#include "motor_control.h"
#include <ros.h>
#include "msgs/Motor.h"
#include "std_msgs/Int32.h"
//------------------------------------------------
//  グローバル変数定義
//------------------------------------------------
const float PERIOD = 10.0; // ms -> 1000/PERIOD[Hz]
msgs::Motor SPEED;
int mode = 0;

encoder enc_left(PIN_A_LEFT, PIN_B_LEFT);
encoder enc_right(PIN_A_RIGHT, PIN_B_RIGHT);
  motor mot_left(PIN_PWM_LEFT, PIN_DIR_LEFT);
  motor mot_right(PIN_PWM_RIGHT, PIN_DIR_RIGHT);

ros::NodeHandle nh;
ros::Publisher pub("speed", &SPEED);

void controlCb(const msgs::Motor& get_msg){
  mot_left.PWM = (int)get_msg.left;
  mot_right.PWM = (int)get_msg.right;
}
void directCb(const msgs::Motor& get_msg){
  mot_left.PWM = (int)get_msg.left;
  mot_right.PWM = (int)get_msg.right;
}
void LEDCb(const std_msgs::Int32& msg){
  mode = (int)msg.data;
}

ros::Subscriber<msgs::Motor> sub("control", &controlCb);
ros::Subscriber<std_msgs::Int32> LEDsub("LED", &LEDCb);

void setup() {
  Serial.begin(57600);
  attachInterrupt(3, counter0, CHANGE);
  attachInterrupt(2, counter0, CHANGE);
  attachInterrupt(0, counter1, CHANGE);
  attachInterrupt(1, counter1, CHANGE);
  pinMode(LEDPIN, OUTPUT);
  digitalWrite(LEDPIN, LOW);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  nh.subscribe(LEDsub);
}

void loop() {
  // 現在の速度の取得
  enc_left.getSPEED(0);
  enc_right.getSPEED(1);
  SPEED.left = SPEED_NOW[0];
  SPEED.right = SPEED_NOW[1];
  // 速度の出力
  mot_left.Write();
  mot_right.Write();
  // LED出力
  if(mode){
    digitalWrite(LEDPIN, HIGH);
  }
  else{
    digitalWrite(LEDPIN, LOW);
  }  
  // ROS処理
  pub.publish(&SPEED);
  nh.spinOnce();
  delay(PERIOD);
}