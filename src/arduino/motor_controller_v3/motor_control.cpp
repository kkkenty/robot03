#include "motor_control.h"
//------------------------------------------------
//  グローバル変数定義
//------------------------------------------------
float SPEED_NOW[NUM] = {0}; // publishに使用
// 割り込み処理(class宣言外)に使われるため，global変数
volatile long value[NUM] = {0}; // エンコーダの値(割り込みで変化)
volatile int nowSig_A[NUM] = {0}, nowSig_B[NUM] = {0}, oldSig_A[NUM] = {0}, oldSig_B[NUM] = {0}; // A,B相の信号
volatile int nowState[NUM] = {0}, oldState[NUM] = {0}; // A,B相の状態 
//------------------------------------------------
//  概　要：encoderのコンストラクタ
//  引  数：A相ピン番号，B相ピン番号
//------------------------------------------------
encoder::encoder(int PA, int PB){
  A = PA; B = PB;
  pinMode(A, INPUT);
  pinMode(B, INPUT);
  time_pre = millis();
}
//------------------------------------------------
//  概　要：現在の速度の取得
//  引  数：0/1(左/右)
//------------------------------------------------
void encoder::getSPEED(int i){
  time_now = millis();
  value_now = value[i];
  //Serial.println(value_now);
  if((dt = time_now - time_pre) > 50){ // 20Hz
    SPEED_NOW[i] = (float)(value_now - value_pre) / dt;
    //Serial.println(SPEED_NOW[i]);
    time_pre = time_now;
    value_pre = value_now;
  }
}
//------------------------------------------------
//  概　要：motorのコンストラクタ
//  引  数：PWMピン番号，DIRピン番号
//------------------------------------------------
motor::motor(int P1, int P2){
  POW = P1; DIR = P2;
  pinMode(POW, OUTPUT);
  pinMode(DIR, OUTPUT);
}
//------------------------------------------------
//  概　要：モータに出力を送る
//------------------------------------------------
void motor::Write(){
  if(PWM > 255 ) PWM = 255; // 上限・下限の設定
  else if(PWM < -255) PWM = -255;
  
  if(PWM >= 0){
    analogWrite(POW, PWM);
    digitalWrite(DIR, LOW);
  }
  else if(PWM < 0){
    analogWrite(POW, -PWM);
    digitalWrite(DIR, HIGH);
  }
  //Serial.println(PWM);
}
//------------------------------------------------
//  概　要：左エンコーダのカウンタ（割り込み）
//------------------------------------------------
void counter0(){
  nowSig_A[0] = digitalRead(PIN_A_LEFT);
  nowSig_B[0] = digitalRead(PIN_B_LEFT);
  
  if(nowSig_A[0] != oldSig_A[0] || nowSig_B[0] != oldSig_B[0]){
    if     (nowSig_A[0] == 0 && nowSig_B[0] == 0) nowState[0] = 0;
    else if(nowSig_A[0] == 1 && nowSig_B[0] == 0) nowState[0] = 1;
    else if(nowSig_A[0] == 1 && nowSig_B[0] == 1) nowState[0] = 2;
    else if(nowSig_A[0] == 0 && nowSig_B[0] == 1) nowState[0] = 3;

    if((oldState[0] == 0 && nowState[0] == 1) || 
       (oldState[0] == 1 && nowState[0] == 2) ||
       (oldState[0] == 2 && nowState[0] == 3) || 
       (oldState[0] == 3 && nowState[0] == 0)){
      value[0]++;
    }
    else if((oldState[0] == 0 && nowState[0] == 3) || 
            (oldState[0] == 3 && nowState[0] == 2) ||
            (oldState[0] == 2 && nowState[0] == 1) || 
            (oldState[0] == 1 && nowState[0] == 0)){
      value[0]--;
    }
    oldSig_A[0] = nowSig_A[0];
    oldSig_B[0] = nowSig_B[0];
    oldState[0] = nowState[0];    
  }
}
//------------------------------------------------
//  概　要：右エンコーダのカウンタ（割り込み）
//------------------------------------------------
void counter1(){
  nowSig_A[1] = digitalRead(PIN_A_RIGHT);
  nowSig_B[1] = digitalRead(PIN_B_RIGHT);
  
  if(nowSig_A[1] != oldSig_A[1] || nowSig_B[1] != oldSig_B[1]){
    if     (nowSig_A[1] == 0 && nowSig_B[1] == 0) nowState[1] = 0;
    else if(nowSig_A[1] == 1 && nowSig_B[1] == 0) nowState[1] = 1;
    else if(nowSig_A[1] == 1 && nowSig_B[1] == 1) nowState[1] = 2;
    else if(nowSig_A[1] == 0 && nowSig_B[1] == 1) nowState[1] = 3;

    if((oldState[1] == 0 && nowState[1] == 1) || 
       (oldState[1] == 1 && nowState[1] == 2) ||
       (oldState[1] == 2 && nowState[1] == 3) || 
       (oldState[1] == 3 && nowState[1] == 0)){
      value[1]++;
    }
    else if((oldState[1] == 0 && nowState[1] == 3) || 
            (oldState[1] == 3 && nowState[1] == 2) ||
            (oldState[1] == 2 && nowState[1] == 1) || 
            (oldState[1] == 1 && nowState[1] == 0)){
      value[1]--;
    }
    oldSig_A[1] = nowSig_A[1];
    oldSig_B[1] = nowSig_B[1];
    oldState[1] = nowState[1];    
  }
}
