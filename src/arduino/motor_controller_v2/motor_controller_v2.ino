// PWM値を取得し、エンコーダが検出した速度を配信する
#include "motor_control.h"
#include <ros.h>
#include "msgs/Motor.h"
//------------------------------------------------
//  グローバル変数定義
//------------------------------------------------
const float PERIOD = 10.0; // ms -> 1000/PERIOD[Hz]
msgs::Motor SPEED; // /encoderにpublishするための変数

encoder enc_left(PIN_A_LEFT, PIN_B_LEFT);
encoder enc_right(PIN_A_RIGHT, PIN_B_RIGHT);
  motor mot_left(PIN_PWM_LEFT, PIN_DIR_LEFT);
  motor mot_right(PIN_PWM_RIGHT, PIN_DIR_RIGHT);

ros::NodeHandle nh;
ros::Publisher pub("encoder", &SPEED);

void powerCb(const msgs::Motor& get_msg){
  mot_left.PWM = (int)get_msg.left;
  mot_right.PWM = (int)get_msg.right;
}

ros::Subscriber<msgs::Motor> sub("power", &powerCb);

void setup() {
  Serial.begin(57600);
  attachInterrupt(3, counter0, CHANGE);
  attachInterrupt(2, counter0, CHANGE);
  attachInterrupt(0, counter1, CHANGE);
  attachInterrupt(1, counter1, CHANGE);
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
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
  // ROS処理
  pub.publish(&SPEED);
  nh.spinOnce();
  delay(PERIOD);
}
