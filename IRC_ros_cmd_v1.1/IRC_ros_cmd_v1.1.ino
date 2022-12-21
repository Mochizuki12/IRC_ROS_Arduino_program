/*License
  SPDX-License-Identifier:MIT
  Copyright (C) 2022  Yuya Mochizuki & Yusuke Yamasaki. All Rights Reserved.
*/
/*
 ###########################################################################################
 #  LDX-277 Servo motor for using Arduino                                                  #
 #  Arduino Servo.h (Servo -> fservo)                                                      #
 #  PWM Pin 11                                                                             #
 #  1deg = 1.5                                                                             #
 #  180deg = 150                                                                           #
 #  90deg = 90                                                                             #
 #  45deg = 30                                                                             #
 ###########################################################################################

 ###########################################################################################
 #  Tact switch Pin number                                                                 #
 #  D42                                                                                    #
 #                                                                                         #
 #  DIP Switch Pin number                                                                  #
 #  D30                                                                                    #
 #  D32                                                                                    #
 #  D34                                                                                    #
 ###########################################################################################

 ###########################################################################################
 #  ROS Subscriber                                                                         #
 #                                                                                         #
 #                                                                                         #
 #                                                                                         #
 #                                                                                         #
 #                                                                                         #
 #                                                                                         #
 ###########################################################################################

*/


#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Wire.h>
#include <Servo.h>
#include <avr/io.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

//使用する名前空間の宣言
using namespace std_msgs;
using namespace ros;


//サーボ
Servo fservo;

//ノードハンドル
NodeHandle nh;

//パブリッシャー設定
Float32MultiArray pub_odometry_msg;
Int16MultiArray pub_distance_msg;
Bool pub_crossline_msg;
Publisher pub_distance("IRC_distance", &pub_distance_msg);
Publisher pub_odometry("IRC_odometry", &pub_odometry_msg);
Publisher pub_crossline("IRC_crossline", &pub_crossline_msg);

//定数の指定
#define CPR 400 //カウント/回転｜一回転で何回カウントされるか
#define wheel 0.071 //車輪の直径
#define pi 3.14159265358979 //円周率
#define wheel_base 0.228 //車輪間長
#define rad_to_degree 57.295779 //ラジアンから角度への変換係数
#define stop_distance 10000000 //停止する距離を指定
#define dt 0.010048 //サンプリング時間
#define reflect_sensor_threshold 600 //ライントレースのオン閾値
#define sensor_weight 4  //フォトリフレクタの重み
#define sensor_bias 2 //フォトリフレクタの重みのバイアス
#define crossline_threshold 3 //クロスラインの検出に必要なセンサの数
#define crossline_detect_deadtime 1000  //次のクロスライン検出可能になるための不感時間（1000 = 1s)

//PIDゲイン
#define V_KP 100
#define V_KI 300
#define V_KD 0.001
#define LT_KP 13
#define LT_KI 4
#define LT_KD 0.2

//エンコーダ外部割り込みピン指定
#define encode_LA 18
#define encode_LB 19
#define encode_RA 3
#define encode_RB 2

//*変数宣言*################################################################################################

//ターゲット速度の指定
float v_target = 0.0;
float line_target = 0.0;
float omega_target = 0.0;

//外部割込みハンドラ内の変数を指定
volatile long R_encoderValue = 0;
volatile int  R_post_encode = 0;

volatile long L_encoderValue = 0;
volatile int  L_post_encode = 0;

float enc_to_rev = 0,
      wheel_length = pi * wheel,
      d = 0.114,
      dtheta = 0,
      Lv = 0,
      Rv = 0,
      R = 0,
      dx = 0,
      dy = 0,
      L_encoder1,
      L_encoder2,
      R_encoder1,
      R_encoder2,
      degree = 0,
      CPRG = 0.0025,
      theta = 0,
      omega = 0,
      Lrps = 0,
      Rrps = 0;

volatile float v  = 0,
               xw = 0,
               yw = 0, 
               degreew = 0,
               RvPrev = 0,
               LvPrev = 0,
               RvFilt = 0,
               LvFilt = 0;

float P[2] = {0.0,0.0},
      I[2] = {0.0,0.0},
      D[2] = {0.0,0.0},
      vel_diff[2] = {0.0,0.0},
      old_vel_diff[2] = {0.0,0.0},
      Lpid = 0,
      Rpid = 0;

float linetrace_diff[2] = {0.0,0.0},
      old_linetrace_diff[2] = {0.0,0.0},
      reflect_sensor[8],
      linetrace_min[8],
      linetrace_max[8],
      min_calib_flag = 0,
      max_calib_flag = 0,
      linetrace_L = 0,
      linetrace_R = 0;

int i = 0, start_flag = 0;

long current_t = 0,
     prev_t = 0;

bool crossline_find = false;

//使用するコードのON・OFF
bool linetrace_EN_ = true,
     pub_distance_EN = false,
     omega_in = false,
     prev_crossline_find = false;
//##########################################################################################################

//ROSコールバック関数

//サーボモータの角度指定
void servo_msgCb( const Int8 & cmd_servo_msg )
{
  fservo.write(cmd_servo_msg.data);
}

//目標速度の指定
void vel_msgCb( const Float32 & IRCcmd_vel_msg )
{
  v_target = IRCcmd_vel_msg.data;
  omega_in = false;
}

//目標角速度の指定
void anglevel_msgCb( const Float32 & IRCcmd_anglevel_msg )
{
  omega_target = IRCcmd_anglevel_msg.data;
  omega_target = omega_target * pi / 180 * wheel_base * 0.5;
  omega_in = true;
}

//ファンモータのON・OFF
void Fanmotor_msgCb( const Bool & cmd_Fanmotor_msg )
{
  if(cmd_Fanmotor_msg.data){
    analogWrite(8, 120);
    digitalWrite(48, HIGH);
  }
}

//ライントレースの使用
void linetrace_EN_msgCb( const Bool & linetrace_EN_msg )
{
  linetrace_EN_ = linetrace_EN_msg.data;
}

//オドメトリリセット
void odometry_RESET_msgCb( const Bool & odometry_RESET_msg)
{
  if(odometry_RESET_msg.data)
  {
    xw = 0;
    yw = 0;
    degreew = 0;
  }
}

//ROSサブスクライバ
Subscriber<Float32> IRCcmd_vel("IRCcmd_vel", vel_msgCb );
Subscriber<Float32> IRCcmd_anglevel("IRCcmd_anglevel", anglevel_msgCb );
Subscriber<Int8> IRCservo_value("IRC_servo", servo_msgCb );
Subscriber<Bool> IRCFanmotor_EN("IRC_Fanmotor", Fanmotor_msgCb );
Subscriber<Bool> IRClinetrace_EN("IRC_linetrace", linetrace_EN_msgCb );
Subscriber<Bool> IRCodometry_RESET("IRC_odometry_RESET", odometry_RESET_msgCb );


void setup()
{
  //Serial.begin(115200);
  Wire.begin();

  //ボーレートの設定
  nh.getHardware()->setBaud(57600);
  
  //ノード初期化
  nh.initNode();

  //ROSパブリッシャ
  //距離
  nh.advertise(pub_distance);
  pub_distance_msg.data = malloc(sizeof(int) * 2);
  pub_distance_msg.data_length = 2;

  //オドメトリ
  nh.advertise(pub_odometry);
  pub_odometry_msg.data = malloc(sizeof(int) * 3);
  pub_odometry_msg.data_length = 3;

  //クロスライン検出
  nh.advertise(pub_crossline);

  //足周りモータ  
  //L_MOTOR
  pinMode(44, OUTPUT);  // DIR_ROL
  pinMode(6, OUTPUT);  // PWM
  digitalWrite(44, HIGH);  // L_MOTOR_DIR_ROL
    
  //R_MOTOR
  pinMode(46, OUTPUT);  // DIR_ROL
  pinMode(7, OUTPUT);  // PWM
  digitalWrite(46, LOW);  // R_MOTOR_DIR_ROL

  //FAN_MOTOR
  pinMode(48, OUTPUT); //Fan MD DIR
  pinMode(8, OUTPUT);  //Fan MD PWM
  digitalWrite(48, LOW);

  pinMode(encode_LA,INPUT);
  pinMode(encode_LB,INPUT);
  digitalWrite(encode_LA, HIGH);  //内部プルアップ20k
  digitalWrite(encode_LB, HIGH);
  attachInterrupt(digitalPinToInterrupt(encode_LA), updateEncoderL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encode_LB), updateEncoderL, CHANGE);

  //右エンコーダ
  pinMode(encode_RA,INPUT);
  pinMode(encode_RB,INPUT);
  digitalWrite(encode_RA, HIGH);  //内部プルアップ20kfor(int i = 0;i < 8;i++)
  digitalWrite(encode_RB, HIGH);
  attachInterrupt(digitalPinToInterrupt(encode_RA), updateEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encode_RB), updateEncoderR, CHANGE);

  //スイッチ
  pinMode(42, INPUT);
  pinMode(32, INPUT);
  pinMode(34, INPUT);

  //ライントレーサ（フォトリフレクタ）ピン設定
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);

  //サブスクライバのノード設定
  nh.subscribe(IRCservo_value);
  nh.subscribe(IRCcmd_vel);
  nh.subscribe(IRCFanmotor_EN);
  nh.subscribe(IRClinetrace_EN);
  nh.subscribe(IRCcmd_anglevel);
  nh.subscribe(IRCodometry_RESET);

  //サーボモータPWMピン設定
  fservo.attach(11);

  //足回り＋ファンモータのPWM周波数の設定(Timer4)
  TCCR4B = (TCCR4B & 0b11111000) | 0x02;

  //PID制御のタイマー割り込みの設定(Timer2(8bit)・10ms・100Hz)
  //初期化
  TCCR2A = 0; 
  TCCR2B = 0;
  OCR2A = 157;//割り込みを発生させるカウンタの設定
  TCCR2A |= (1 << WGM21);//CTCモードに設定
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);//分周比の設定(1024)
  TIMSK2 |= (1 << OCIE2A);

  delay(1000);
}


//メインループ
void loop()
{
  
  //チャタリング防止＆スタート
  if(start_flag == 0 && !digitalRead(34) && !digitalRead(32))
  {
    delay(10);
    if(digitalRead(42)==HIGH)
    {

      while(digitalRead(42)==HIGH && i<=51){
        i++;
      }

      if(i >= 50){
       //Serial.println(i);
       delay(2000);
       start_flag = 1; 
      }

    }
  }

  //ライントレーサキャリブレーション・左のスイッチ＋青（最小値の設定)
  else if(start_flag == 0 && digitalRead(34) && !digitalRead(32))
  {
    delay(10);
    if(digitalRead(42)==HIGH)
    {

      while(digitalRead(42)==HIGH && i<=51){
        i++;
      }

      if(i >= 50){
        if(min_calib_flag == 0){
          for(int i = 0;i < 8;i++){
            linetrace_min[i] = analogRead(i);
          }
        }
        min_calib_flag = 1;
        //Serial.println(linetrace_min[0]);
      }

    }
  }

    //ライントレーサキャリブレーション・真ん中のスイッチ＋青（最大値の設定)
  else if(start_flag == 0 && !digitalRead(34) && digitalRead(32))
  {
    delay(10);
    if(digitalRead(42)==HIGH)
    {

      while(digitalRead(42)==HIGH && i<=51){
        i++;
      }

      if(i >= 50){
        if(max_calib_flag == 0){
          for(int i = 0;i < 8;i++){
            linetrace_max[i] = analogRead(i);
          }
        }
        max_calib_flag = 1;
        //Serial.println(linetrace_max[0]);
      }

    }
  }
  
  if(start_flag == 1){

    pub_odometry_msg.data[0] = xw;
    pub_odometry_msg.data[1] = yw;
    pub_odometry_msg.data[2] = degreew;
    //pub_crossline_msg.data = crossline_find;
    //pub_odometry.publish( &pub_odometry_msg );
    //pub_crossline.publish( &pub_crossline_msg );
  
    //測距センサの使用
    if(pub_distance_EN == 1)
    {
      pub_distance_msg.data[0] = readDistance(0x52);
      pub_distance_msg.data[1] = readDistance(0x5D);
      pub_distance.publish( &pub_distance_msg );
    }

  }
  nh.spinOnce();
  delay(1);
}


//PID制御ISR
ISR (TIMER2_COMPA_vect){

  if(start_flag == 1){
    
    noInterrupts();
    R_encoder1 = R_encoderValue;
    L_encoder1 = L_encoderValue;
    interrupts();
  
    Rrps = (R_encoder1 - R_encoder2) / dt / CPR;
    Lrps = (L_encoder1 - L_encoder2) / dt / CPR;
    R_encoder2 = R_encoder1;
    L_encoder2 = L_encoder1;

    Lv = wheel_length * Lrps;
    Rv = wheel_length * Rrps;
    LvFilt = 0.854*LvFilt + 0.0728*Lv + 0.0728*LvPrev;
    LvPrev = Lv;
    RvFilt = 0.854*RvFilt + 0.0728*Rv + 0.0728*RvPrev;
    RvPrev = Rv;
    
    if(omega_in)
    {
      vel_diff[0] = omega_target - LvFilt;
      vel_diff[1] = -omega_target - RvFilt;
    }else{
      vel_diff[0] = v_target - LvFilt;
      vel_diff[1] = v_target - RvFilt;
    }
    
    P[0] = V_KP * vel_diff[0];
    I[0] += V_KI * vel_diff[0] * dt;
    D[0] = V_KD * (vel_diff[0] - old_vel_diff[0]) / dt;
    P[1] = V_KP * vel_diff[1];
    I[1] += V_KI * vel_diff[1] * dt;
    D[1] = V_KD * (vel_diff[1] - old_vel_diff[1]) / dt;

    old_vel_diff[0] = vel_diff[0];
    old_vel_diff[1] = vel_diff[1];

    Lpid = (P[0] + I[0] + D[0]) ;
    Rpid = (P[1] + I[1] + D[1]) ;

    if(linetrace_EN_){
      Linetrace_control();
    }

    constrain(Lpid, -255, 255);
    constrain(Rpid, -255, 255);

    if(Lpid < 0){
      digitalWrite(44, LOW);
      analogWrite(6, -Lpid);  // L
    }else{
      digitalWrite(44, HIGH);
      analogWrite(6, Lpid);  // L
    }

    if(Rpid < 0){
      digitalWrite(46, HIGH);
      analogWrite(7, -Rpid);  // R
    }else{
      digitalWrite(46, LOW);
      analogWrite(7, Rpid);  // R
    }
  }
  

  //オドメトリ------------------------------------------------------------
  v = (LvFilt + RvFilt) * 0.5;

  //距離から角度・角速度に変換
  omega = (RvFilt - LvFilt) / wheel_base; //角速度ω
  dtheta = omega * dt;              //微小角度dθ
  degree = dtheta * 180 / pi;  //角度θ
  
  //変位計算
  if(abs(omega) <= 1e-16){
    dx = v * dt * cos(theta);
    dy = v * dt * sin(theta);
  }
  else{
    dx = 2 * v / omega * cos(theta + dtheta*0.5) * sin(dtheta*0.5);
    dy = 2 * v / omega * sin(theta + dtheta*0.5) * sin(dtheta*0.5);
  }


  //オドメトリ変換｜X軸，Y軸
    xw += dx;
    yw += dy;

  //オドメトリ変換｜角度
  degreew += degree;
  if(0 > degreew)
  {
    degreew = 360;
  }else if(360 < degreew){
    degreew = 0;
  }
  theta += dtheta;

  /*
  Serial.print(xw);
  Serial.print(",");
  Serial.print(yw);
  Serial.print(",");
  Serial.print(degreew);
  Serial.print(",");
  Serial.println(omega);
  */
}

void Linetrace_control(){
  
  int reflect_sensor_cnt = 0;
  int crossline_reflect_sensor_cnt = 0;

  linetrace_L = linetrace_R = 0;

  for(int j = 0;j < 8;j++)
  {
    reflect_sensor[j] = analogRead(j);

    if(reflect_sensor_threshold < reflect_sensor[j])
    {
      if(j == 0 || j == 1 || j == 6 || j == 7){
        crossline_reflect_sensor_cnt++;
      }
      reflect_sensor_cnt++;
    }
  }
  
  for(int i = 2;i < 4;i++){
    linetrace_L += (reflect_sensor[i] - linetrace_min[i])/(linetrace_max[i] - linetrace_min[i]) * (sensor_bias + 2 * (sensor_weight - i));
    linetrace_R += (reflect_sensor[7-i] - linetrace_min[7-i])/(linetrace_max[7-i] - linetrace_min[7-i]) * (sensor_bias + 2 * (sensor_weight - i));
    //debug
    //Serial.println(analogRead(3));
    //Serial.println(analogRead(4));
  }

  current_t = millis();
  if(reflect_sensor_cnt >= crossline_threshold)
  {
    if(((current_t - prev_t) > crossline_detect_deadtime) && (crossline_reflect_sensor_cnt >= 1)){
      pub_crossline_msg.data = true;
      pub_crossline.publish( &pub_crossline_msg );
      prev_t = current_t;
    }
    
  }else{
    pub_crossline_msg.data = false;
    pub_crossline.publish( &pub_crossline_msg );
    linetrace_diff[0] = (linetrace_R - linetrace_L);
    linetrace_diff[1] = -(linetrace_diff[0]);

    P[0] = LT_KP * linetrace_diff[0];
    I[0] += LT_KI * linetrace_diff[0] * dt;
    D[0] = LT_KD * (linetrace_diff[0] - old_linetrace_diff[0]) / dt;
    P[1] = LT_KP * linetrace_diff[1];
    I[1] += LT_KI * linetrace_diff[1] * dt;
    D[1] = LT_KD * (linetrace_diff[1] - old_linetrace_diff[1]) / dt;

    old_linetrace_diff[0] = linetrace_diff[0];
    old_linetrace_diff[1] = linetrace_diff[1];
    
    Lpid += (P[0] + I[0] + D[0]);
    Rpid += (P[1] + I[1] + D[1]);

  }

}


//I2CToF測距センサから距離を取得する関数
int readDistance(int ADDRESS)
{
  
  uint16_t distance;
  uint16_t distance_tmp;
  uint8_t data_cnt;
  
  Wire.beginTransmission(ADDRESS);
  Wire.write(0xD3);
  Wire.endTransmission(false);
  Wire.requestFrom(ADDRESS, 2);
  data_cnt = 0;
  distance = 0;
  distance_tmp = 0;
  while(Wire.available())
  {
    distance_tmp = Wire.read();
    distance = (distance << (data_cnt*8)) | distance_tmp;
    data_cnt++;
  }
  return distance;
  
}




//左モータのエンコーダの値を更新するプログラム
void updateEncoderL(){

  int MSB, LSB, L_encode, rev_dir;
  
  MSB = digitalRead(encode_LA);
  LSB = digitalRead(encode_LB);
 
  L_encode = (MSB << 1) | LSB;
  rev_dir  = (L_post_encode << 2) | L_encode;
 
  if(rev_dir == 0b1101 || rev_dir == 0b1011 || rev_dir == 0b0100 || rev_dir == 0b0010 ){
    L_encoderValue --;
  }
  if(rev_dir == 0b1110 || rev_dir == 0b0111 || rev_dir == 0b0001 || rev_dir == 0b1000){
    L_encoderValue ++;
  }
 
  L_post_encode = L_encode;
}


//右モータのエンコーダの値を更新するプログラム
void updateEncoderR(){

  int MSB, LSB, R_encode, rev_dir;
  
  MSB = digitalRead(encode_RA);
  LSB = digitalRead(encode_RB);
 
  R_encode = (MSB << 1) | LSB;
  rev_dir  = (R_post_encode << 2) | R_encode;
 
  if(rev_dir == 0b1101 || rev_dir == 0b1011 || rev_dir == 0b0100 || rev_dir == 0b0010 ){
    R_encoderValue --;
  }
  if(rev_dir == 0b1110 || rev_dir == 0b0111 || rev_dir == 0b0001 || rev_dir == 0b1000){
    R_encoderValue ++;
  }
 
  R_post_encode = R_encode;
}