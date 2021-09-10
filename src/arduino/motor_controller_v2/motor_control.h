#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
//------------------------------------------------
#include "Arduino.h"
//------------------------------------------------
//  マクロ定義(Macro definition)
//------------------------------------------------
#define PIN_A_LEFT 20 //-> 3
#define PIN_B_LEFT 21 //-> 2
#define PIN_PWM_LEFT 10
#define PIN_DIR_LEFT 11
#define PIN_A_RIGHT 2 //-> 0
#define PIN_B_RIGHT 3 //-> 1
#define PIN_PWM_RIGHT 8
#define PIN_DIR_RIGHT 9
#define NUM 2 // モータの個数 0がleft, 1がright
//------------------------------------------------
//  extern宣言
//------------------------------------------------
extern float SPEED_NOW[NUM]; // publishに使用
//------------------------------------------------
//  クラス定義(Class definition)
//------------------------------------------------
class encoder{
  private:
    int A, B;
    float dt = 0.0;
    unsigned long time_pre = 0, time_now = 0;
    long value_pre = 0, value_now = 0;
  public:
    encoder(int PA, int PB);
    void getSPEED(int i); // 現在の速度の取得
};
class motor{
  private:
    int POW, DIR;
  public:
    int PWM = 0;
    motor(int P1, int P2);
    void Write(); // 速度の出力
};
//------------------------------------------------
//  プロトタイプ宣言(Prototype declaration)
//------------------------------------------------
void counter0();
void counter1();
//------------------------------------------------
#endif
