#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <sensor_msgs/Joy.h>
#include <visualization_msgs/Marker.h>

//** global変数 **// 
const int pt = 5; //目標地点の個数
// sister's room
//double goal[pt][2] = {{-0.5, -0.2}, {1.0, -0.2}, {1.0, -1.2}, {-0.5, -1.2}, {-0.5, -0.2}}; 
// robosa
double goal[pt][2] = {{0.3, -0.6}, {4.0, -0.6}, {4.0, -2.1}, {0.3, -2.1}, {0.3, -0.6}}; 
int FRIQUENCE = 20, den = 100, ahed = 5, stap = 0; // 経路分割数、lookaheddistance、停止変数
double vel = 0.35; // ロボットの速度

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
  ros::init(argc, argv, "pure_pursuit_loop_v2");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  tf::StampedTransform tf;
  geometry_msgs::Twist cmd;
  pnh.getParam("FRIQUENCE", FRIQUENCE);
  pnh.getParam("den", den);
  pnh.getParam("ahed", ahed);
  pnh.getParam("vel", vel);
  
  int i;
  double x = 0.0, y = 0.0, yaw = 0.0; // robot's pose
  double gx = 0.0, gy = 0.0, gduty = 0.0; // ahed's pose
  double alpha = 0.0, L = 0.0, sumpath = 0.0; // 方位誤差、距離、総経路距離
  static int cr = 0, ad = 0, gcr = 0, gad = 0; // 現在の経路番号、位置、目標店の経路番号、位置
    
  // 点線経路の作成 //
  double dotpath[den][2]; // 点線の座標
  double path[pt-1]; // 各経路長
  int npath[pt-1], sumn = 0; // 全体に対する経路への整数変換、経路の分割数
  for(i= 0;i<pt-1;i++){
    path[i] = sqrt(pow(goal[i+1][0]-goal[i][0], 2) + pow(goal[i+1][1]-goal[i][1], 2));
    sumpath += path[i];
  }
  for(i=0;i<pt-1;i++){
    npath[i] = abs( (int)(path[i] / sumpath * (double)den) );
    sumn += npath[i];
  }
  
  visualization_msgs::Marker line, npoint, gpoint;
  line.header.frame_id = npoint.header.frame_id = gpoint.header.frame_id = "map";
  line.header.stamp = npoint.header.stamp = gpoint.header.stamp = ros::Time::now();
  line.ns = npoint.ns = gpoint.ns = "line_and_points";
  line.action = npoint.action = gpoint.action = visualization_msgs::Marker::ADD;
  line.pose.orientation.w = npoint.pose.orientation.w = gpoint.pose.orientation.w = 1.0;
  line.id = 0;  npoint.id = 1;  gpoint.id = 2;
  line.type = visualization_msgs::Marker::LINE_STRIP;
  npoint.type = gpoint.type = visualization_msgs::Marker::POINTS;
  line.scale.x = 0.02;
  npoint.scale.x = npoint.scale.y = gpoint.scale.x = gpoint.scale.y = 0.1; 
  line.color.g = 1.0;  line.color.b = 1.0;  line.color.a = 1.0;
  npoint.color.g = 1.0;  npoint.color.a = 1.0;
  gpoint.color.r = 1.0;  gpoint.color.a = 1.0;
  
  geometry_msgs::Point p;
  for(i=0;i<pt;i++){
    p.x = goal[i][0]; p.y = goal[i][1];
    line.points.push_back(p);
  }
  
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 10);
  ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyCb);
  tf::TransformListener listener;
  ros::Rate rate(FRIQUENCE);
  
  while(nh.ok()){
    rate.sleep();
    ros::spinOnce();
    // 点Markerの初期化(もっと良い初期化の方法は無いかな...)
    visualization_msgs::Marker npoint, gpoint;
    npoint.header.frame_id = gpoint.header.frame_id = "map";
    npoint.header.stamp = gpoint.header.stamp = ros::Time::now();
    npoint.ns = gpoint.ns = "line_and_points";
    npoint.action = gpoint.action = visualization_msgs::Marker::ADD;
    npoint.pose.orientation.w = gpoint.pose.orientation.w = 1.0;
    npoint.id = 1;  gpoint.id = 2;
    npoint.type = gpoint.type = visualization_msgs::Marker::POINTS;
    npoint.scale.x = npoint.scale.y = gpoint.scale.x = gpoint.scale.y = 0.1; 
    npoint.color.g = 1.0;  npoint.color.a = 1.0;
    gpoint.color.r = 1.0;  gpoint.color.a = 1.0;
    
    // ロボット座標の取得
    try{
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
    gduty = (double)ad / (double)den;
    p.x = goal[cr][0] + (path[cr][0] * gduty);
    p.y = goal[cr][1] + (path[cr][1] * gduty); 
    npoint.points.push_back(p);
    
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
    gduty = (double)gad / (double)den;
    p.x = gx = goal[gcr][0] + (path[gcr][0] * gduty);
    p.y = gy = goal[gcr][1] + (path[gcr][1] * gduty);
    gpoint.points.push_back(p);
    
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
    marker_pub.publish(line);
    marker_pub.publish(npoint);
    marker_pub.publish(gpoint);
  }
  return 0;
}
