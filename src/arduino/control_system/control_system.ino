#include <ros.h>
#include <msgs/Motor.h>

#define PIN_A_left 20 
#define PIN_B_left 21
#define CUT_PIN_A_left 2 // [2->0, 3->1, 21->2, 20->3, 19->4, 18->5]
#define CUT_PIN_B_left 3
#define PIN_PWM_left 10
#define PIN_DIR_left 11
#define PIN_A_right 3 
#define PIN_B_right 2
#define CUT_PIN_A_right 1 // [2->0, 3->1, 21->2, 20->3, 19->4, 18->5]
#define CUT_PIN_B_right 0
#define PIN_PWM_right 8
#define PIN_DIR_right 9

const float PERIOD = 10.0; // ms -> 1000/PERIOD[Hz]

class Encoder{
  public:
  volatile long value; // エンコーダの値(割り込みで変化)
  int nowSig_A = 0, nowSig_B = 0, oldSig_A = 0, oldSig_B = 0; // A,B相の信号
  int nowState = 0, oldState = 0; // A,B相の状態
  float SPEED_NOW = 0.0, SPEED_GOAL = 0.0, SPEED_ERROR = 0.0, LIMIT = 0.15, kp = 25.0, ki = 2.0, kd = 0.1;
  int PWM = 0, ACC = 1;
  void encoder();
};

void Encoder::encoder(){
  nowSig_A = digitalRead(PIN_A);
  nowSig_B = digitalRead(PIN_B);
  
  if(nowSig_A != oldSig_A || nowSig_B != oldSig_B){
    if     (nowSig_A == 0 && nowSig_B == 0) nowState = 0;
    else if(nowSig_A == 1 && nowSig_B == 0) nowState = 1;
    else if(nowSig_A == 1 && nowSig_B == 1) nowState = 2;
    else if(nowSig_A == 0 && nowSig_B == 1) nowState = 3;

    if((oldState == 0 && nowState == 1) || 
       (oldState == 1 && nowState == 2) ||
       (oldState == 2 && nowState == 3) || 
       (oldState == 3 && nowState == 0)){
      value++;
    }
    else if((oldState == 0 && nowState == 3) || 
            (oldState == 3 && nowState == 2) ||
            (oldState == 2 && nowState == 1) || 
            (oldState == 1 && nowState == 0)){
      value--;
    }
    oldSig_A = nowSig_A;
    oldSig_B = nowSig_B;
    oldState = nowState;    
  }
}

void powerCb(const msgs::){
  
}

ros::NodeHandle nh;
ros::Subscriber<msgs::Motor> sub("power", &powerCb);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(57600);
  pinMode(PIN_A_left, INPUT);
  pinMode(PIN_B_left, INPUT);
  pinMode(PIN_PWM_left, OUTPUT);
  pinMode(PIN_DIR_left, OUTPUT);
  attachInterrupt(CUT_PIN_A_left, encoder, CHANGE);
  attachInterrupt(CUT_PIN_B_left, encoder, CHANGE);
  pinMode(PIN_A_right, INPUT);
  pinMode(PIN_B_right, INPUT);
  pinMode(PIN_PWM_right, OUTPUT);
  pinMode(PIN_DIR_right, OUTPUT);
  attachInterrupt(CUT_PIN_A_right, encoder, CHANGE);
  attachInterrupt(CUT_PIN_B_right, encoder, CHANGE);
}

void loop() {
  static unsigned long time_pre = millis();
  static long value_pre = value;
  unsigned long time_now;
  long value_now;

  // 目標速度の取得
  if(Serial.available()>0){
    delay(10);
    SPEED_GOAL = Serial.parseInt();
    //Serial.println(SPEED_GOAL);
    while(Serial.available()>0){
      char t = Serial.read();
    }
  }

  // 現在の速度の取得
  time_now = millis();
  value_now = value;
  static float dt = 0.0;
  if((dt = time_now - time_pre) > 50){
    SPEED_NOW = (float)(value_now - value_pre) / dt;
    SPEED_ERROR = SPEED_GOAL - SPEED_NOW;
    Serial.println(SPEED_NOW);
    time_pre = time_now;
    value_pre = value_now;
  }
  /*
  // 台形制御でPWMの計算
  if(SPEED_ERROR > LIMIT){
    PWM += ACC;
  }
  else if(SPEED_ERROR < -LIMIT){
    PWM -= ACC;
  }
  else{ // -LIMIT <= SPEED_ERROR <= LIMIT

  }
  */
  // PID制御でPWM計算
  static float SPEED_SUM = 0.0, SPEED_ERROR_PRE = 0.0, SPEED_PRE = 0.0, SPEED_ACC = 0.0;
  SPEED_SUM += (SPEED_ERROR + SPEED_ERROR_PRE) / 2.0;
  SPEED_ACC = (float)(SPEED_NOW - SPEED_PRE) / PERIOD * 1000.0;
  //Serial.println(SPEED_ACC);
  PWM = kp * SPEED_ERROR + ki * SPEED_SUM - kd * SPEED_ACC;
  SPEED_ERROR_PRE = SPEED_ERROR;
  SPEED_PRE = SPEED_NOW;
  
  // 速度の出力  
  if(PWM > 255 ) PWM = 255;
  else if(PWM < -255) PWM = -255;
  
  if(PWM >= 0){
    analogWrite(PIN_PWM, PWM);
    digitalWrite(PIN_DIR, LOW);
  }
  else if(PWM < 0){
    analogWrite(PIN_PWM, -PWM);
    digitalWrite(PIN_DIR, HIGH);
  }
  //Serial.println(PWM);
  delay(PERIOD);
}
