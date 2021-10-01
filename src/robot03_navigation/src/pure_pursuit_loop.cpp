#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Joy.h>

// global変数
const int pt = 5; //目標地点の個数
double goal[pt][2] = {{-0.5, -0.2}, {1.0, -0.2}, {1.0, -1.2}, {-0.5, -1.2}, {-0.5, -0.2}}; // sister's room
//double goal[pt][2] = {{0.3, -0.5}, {4.0, -0.5}, {4.0, -2.2}, {0.3, -2.2}, {0.3, -0.5}}; // robosa
double path[pt-1][2]; // 目標経路
int FRIQUENCE = 20, den = 20, ahed = 5, stap = 0; // 経路分割数、lookaheddistance、停止変数
double vel = 0.1; // ロボットの速度

// 第1,2引数と第3,4引数の点間距離を算出
double dis(double x, double y, double ax, double ay){
  return sqrt(pow(ax-x, 2) + pow(ay-y, 2));
}

// 最も近い点を選択(局所解は無いと仮定)
// 第1,2引数：現在座標、第3,4引数：現在の経路番号、位置(参照渡し)
void dismin(const double &x, const double &y, int &cr, int &ad){
  int mode = 0, count = 0; // 計算状態変数、カウント変数
  double mindis = 0.0, nowdis = 0.0, duty = 0.0; // 最小経路値、現在経路値、経路分割の割合
  static int i = 0, j = 0;
  while(count <= (pt - 1) * den){ // 全探索しない限り周回
    for(; i < pt - 1; i++){ // 目標経路の更新
      for(; j < den; j++){ // 位置の更新
        duty = (double)j / (double)den;
        nowdis = dis(x, y, goal[i][0]+(path[i][0]*duty), goal[i][1]+(path[i][1]*duty));
        //cout << "nowdis:" << nowdis << endl;
        count++;
        if(mode == 0){ // 初期化
          mindis = nowdis;
          ad = j;  cr = i;
          mode = 1;
          continue;
        }
        if(mindis > nowdis){ // 最短距離の更新
          mindis = nowdis;
          ad = j;  cr = i;
          mode = 2;
        }
        else if(mode == 2){ // 最短経路の更新が途絶えたら終了
          j = ad; i = cr; // 現在の位置を保持
          return;
        }
        if(count > (pt - 1) * den){
          j = ad; i = cr; // 現在の位置を保持
          return;
        }
      }
      j = 0; // 位置のリセット
    }
    i = 0; // 経路のリセット
  }
  /*
  if(mode == 2){ // 最短経路が現れない=最終地点
      ad = den; cr = pt - 2;
  }
  */
}

void joyCb(const sensor_msgs::Joy &joy_msg)
{
  if(joy_msg.buttons[3]){
    if(stap == 0){
      ROS_INFO("NAVI STOPPING!");
      stap = 1;
    }
    else if(stap == 1){
      ROS_INFO("NAVI RESTARTING!");
      stap = 0;
    }
  }
  if(joy_msg.buttons[4]){
    vel += 0.05;
    if(vel > 0.45){
      vel = 0.45;
    }
    ROS_INFO("Now, velocity is [%lf]", vel);
  }
  if(joy_msg.buttons[5]){
    vel -= 0.05;
    if(vel < 0.05){
      vel = 0.05;
    }
    ROS_INFO("Now, velocity is [%lf]", vel);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "pure_pursuit_loop");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // 変数宣言
  int stage = 0; // 周回変数
  double x = 0.0, y = 0.0, yaw = 0.0; // robot's pose
  double gx = 0.0, gy = 0.0, gduty = 0.0; // ahed's pose
  double alpha = 0.0, L = 0.0; // 方位誤差、距離
  static int cr = 0, ad = 0, gcr = 0, gad = 0; // 現在の経路番号、位置、目標店の経路番号、位置
  for(int i= 0;i < pt - 1; i++){
    path[i][0] = goal[i+1][0] - goal[i][0]; // x
    path[i][1] = goal[i+1][1] - goal[i][1]; // y
    //cout << "x:" << path[i][0] << ", y:" << path[i][1] << endl;
  }
  tf::StampedTransform tf;
  geometry_msgs::Twist cmd;
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  pnh.getParam("den", den);
  pnh.getParam("ahed", ahed);
  pnh.getParam("vel", vel);
  
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
  tf::TransformListener listener;
  ros::Rate rate(FRIQUENCE);
  
  while(nh.ok()){
    rate.sleep();
    ros::spinOnce();
    // ロボット座標の取得
    try{
      //listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(0.5));
      listener.lookupTransform("/map", "/base_link", ros::Time(0), tf);
      x = tf.getOrigin().x();
      y = tf.getOrigin().y();
      yaw = tf::getYaw(tf.getRotation());
    }
    catch(tf::TransformException ex){
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    // 最も近い点を選択
    dismin(x, y, cr, ad);
    if(stage%20 == 0){
      ROS_INFO("we reached cource [%2d] and adress [%2d]", cr, ad);
    }
    stage++;
    
    // 目標点の設定
    gcr = cr;
    gad = ad + ahed;
    if(gad >= den){ // 経路の更新
      gcr++;
      gad -= den;
    }
    if(gcr >= pt - 1){ // 最終経路での処置
      gcr = 0;
    }
    //cout << "gcr:" << gcr << ", gad:" << gad << endl;
    gduty = (double)gad / (double) den;
    gx = goal[gcr][0] + (path[gcr][0] * gduty);
    gy = goal[gcr][1] + (path[gcr][1] * gduty);  
    
    // 目標点との相対的な角度、距離の算出
    alpha = atan2(gy - y, gx - x) - yaw;
    L = dis(x, y, gx, gy);
    
    // 車速と角速度の算出
    cmd.linear.x = vel;
    cmd.angular.z = 2.0 * vel * sin(alpha) / L;
    if(stap){ // naviの停止コマンド
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }
    
    // 車速の配信
    cmd_pub.publish(cmd);
  }
  return 0;
}
