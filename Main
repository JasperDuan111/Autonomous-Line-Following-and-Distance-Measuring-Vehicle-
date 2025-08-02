#include <MsTimer2.h>            // 定时中断 
#include <PinChangeInterrupt.h>  // 引脚电平变化中断 
 
/* 电机控制板引脚定义 */ 
unsigned int RIN1 = 5, RIN2 = 11, LIN1 = 6, LIN2 = 3; 
const int sensorPins[4] = { A3, A2, A1, A5 }; 
bool sensorStates[4]; 
/* 速度／编码器变量 */ 
int count_L = 0, count_R = 0; 
int Velocity_Left = 0, Velocity_Right = 0; 
float Target_L = 0, Target_R = 0; 
 
/*巡线任务*/ 
unsigned long crossroadTimer = 0; 
/* PI 控制参数 */ 
float KP = 1.0, KI = 0.1; 
float Kp = 0.8, Kd = 0.1; 
int PWM_Restrict = 255; 
float lastError = 0;     
// 上一次的误差值 
const float BASE_SPEED = -2.0; // 基准速度（可根据电机正负方向调整） 
float PWM_L = 0, PWM_R = 0; 
float Last_bias_L = 0, Last_bias_R = 0; 
/* 编码器引脚 */ 
#define ENCODER_R_A 8 
#define ENCODER_R_B 4 
#define ENCODER_L_A 2 
#define ENCODER_L_B 7 
/* 测距引脚 */ 
const int trigPin = 12; 
const int echoPin = 13; 
const int MEASURE_COUNT = 5;       
// 测量次数，须为奇数 
const float MAX_DISTANCE = 400.0;  // 最大有效测距（cm） 
/* 函数声明 */ 
void READ_ENCODER_R(); 
void READ_ENCODER_L(); 
void control(); 
int Incremental_PI(int encoderCount, float target, float &PWM_var, 
float &Last_bias_var); 
void Set_PWM(int motorR, int motorL); 
void setup() { 
Serial.begin(9600); 
for (int i = 0; i < 4; i++) { 
pinMode(sensorPins[i], INPUT_PULLUP); 
} 
pinMode(ENCODER_R_A, INPUT); 
pinMode(ENCODER_R_B, INPUT); 
pinMode(ENCODER_L_A, INPUT); 
pinMode(ENCODER_L_B, INPUT); 
pinMode(LIN1, OUTPUT); 
  pinMode(LIN2, OUTPUT); 
  pinMode(RIN1, OUTPUT); 
  pinMode(RIN2, OUTPUT); 
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
 
  MsTimer2::set(10, control); 
  MsTimer2::start(); 
 
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), READ_ENCODER_L, 
CHANGE); 
  attachPinChangeInterrupt(digitalPinToPCINT(ENCODER_R_A), 
READ_ENCODER_R, CHANGE); 
} 
 
void loop() { 
 
  lineTrackingPID(); 
  delay(100); 
 /*Serial.print("Vel_L:"); 
  Serial.print(Target_L); 
  Serial.print("  Vel_R:"); 
  Serial.print(Target_R); 
  Serial.print("  PWM_L:"); 
  Serial.print((int)PWM_L); 
  Serial.print("  PWM_R:"); 
  Serial.println((int)PWM_R); 
*/ 
 
} 
 
/*—— 右编码器中断 ——*/ 
void READ_ENCODER_R() { 
  if (digitalRead(ENCODER_R_A) == LOW) { 
    if (digitalRead(ENCODER_R_B) == LOW) count_R++; 
    else count_R--; 
  } else { 
    if (digitalRead(ENCODER_R_B) == LOW) count_R--; 
    else count_R++; 
  } 
} 
 
/*—— 左编码器中断 ——*/ 
void READ_ENCODER_L() { 
  if (digitalRead(ENCODER_L_A) == LOW) { 
    if (digitalRead(ENCODER_L_B) == LOW) count_L++; 
    else count_L--; 
  } else { 
    if (digitalRead(ENCODER_L_B) == LOW) count_L--; 
    else count_L++; 
  } 
} 
 
/*—— 定时中断：测速 + PI 控制 + 驱动 ——*/ 
void control() { 
  int MotorR, MotorL; 
  Velocity_Left = count_L; 
  count_L = 0; 
  Velocity_Right = count_R; 
  count_R = 0; 
 
  MotorR = Incremental_PI(Velocity_Right, Target_R, PWM_R, Last_bias_R); 
  MotorL = Incremental_PI(Velocity_Left, Target_L, PWM_L, Last_bias_L); 
 
  Set_PWM(MotorR, MotorL); 
} 
 
/*—— 增量式 PI 控制器 ——*/ 
int Incremental_PI(int encoderCount, float target, float &PWM_var, 
float &Last_bias_var) { 
  float Bias = target - encoderCount; 
  PWM_var += KP * (Bias - Last_bias_var) + KI * Bias; 
 
  if (PWM_var > PWM_Restrict) PWM_var = PWM_Restrict; 
  if (PWM_var < -PWM_Restrict) PWM_var = -PWM_Restrict; 
 
  Last_bias_var = Bias; 
  return (int)PWM_var; 
} 
 
/*—— PWM 输出函数 ——*/ 
void Set_PWM(int motorR, int motorL) { 
  // 左电机 
  switch ((motorL > 0) - (motorL < 0)) { 
    case 1: 
      analogWrite(LIN1, motorL); 
      digitalWrite(LIN2, LOW); 
      break; 
    case 0: 
      digitalWrite(LIN1, LOW); 
      digitalWrite(LIN2, LOW); 
      break; 
    case -1: 
      analogWrite(LIN1, motorL + 255); 
      digitalWrite(LIN2, HIGH); 
      break; 
  } 
  // 右电机 
  switch ((motorR > 0) - (motorR < 0)) { 
    case 1: 
      analogWrite(RIN1, motorR); 
      digitalWrite(RIN2, LOW); 
      break; 
    case 0: 
      digitalWrite(RIN1, LOW); 
      digitalWrite(RIN2, LOW); 
      break; 
    case -1: 
      analogWrite(RIN1, motorR + 255); 
      digitalWrite(RIN2, HIGH); 
      break; 
  } 
} 
bool scanned = false; 
void lineTrackingPID() { 
  // 读取每个传感器状态 
  bool s4 = (digitalRead(A5) == LOW);   
  bool s3 = (digitalRead(A1) == LOW);   
  bool s1 = (digitalRead(A2) == LOW);   
  bool s2 = (digitalRead(A3) == LOW);   
 
  // 终点停下 
  if (s4 && s3 && s2 && s1) { 
    Target_L = 0; 
    Target_R = 0; 
    distanceMeasure(); 
    delay(5000); 
    return; 
  } 
  // 分叉点 
  if (!s4 && s3 && s2 && s1) { 
    Target_L = BASE_SPEED - pid_output; 
    Target_R = BASE_SPEED + pid_output; 
    delay(1000); 
  } 
  if (s4 && s3 && s2 && !s1) { 
    Target_L = BASE_SPEED - pid_output; 
    Target_R = BASE_SPEED + pid_output; 
    delay(1000); 
  } 
  // 给每个传感器一个权重：s4=-3, s3=-1, s2=+1, s1=+3 
  float weights[4] = { -3.0, -1.0, +1.0, +3.0 }; 
  bool states[4]   = { s4,        s3,       s2,       s1     }; 
 
  float sumWeight = 0.0;   // 加权和 
  int   countOn   = 0;     // 检测到黑线的传感器个数 
 
  for (int i = 0; i < 4; i++) { 
    if (states[i]) { 
      sumWeight += weights[i]; 
      countOn++; 
    } 
  } 
 
  float position;  
  if (countOn > 0) { 
    position = sumWeight / countOn; // 归一化“偏移值” 
  } else { 
    // 如果没有任何传感器检测到黑线，保持上次的误差 
    position = lastError; 
  } 
 
  float error = position; 
  float derivative = error - lastError;    // 误差变化率 
  float P = Kp * error; 
  float D = Kd * derivative; 
  lastError = error; 
  float pid_output = P + D; 
 
  // 根据 PID 输出调整左右电机速度 
 
  float raw_L = BASE_SPEED - pid_output; 
  float raw_R = BASE_SPEED + pid_output; 
 
  const float T_MAX = 2.5; 
  if (raw_L >  T_MAX) raw_L =  T_MAX; 
  if (raw_L < -T_MAX) raw_L = -T_MAX; 
  if (raw_R >  T_MAX) raw_R =  T_MAX; 
  if (raw_R < -T_MAX) raw_R = -T_MAX; 
 
  Target_L = raw_L; 
  Target_R = raw_R; 
 
  // （可视化或调试信息输出，可选） 
  // Serial.print("output="); Serial.print(pid_output); 
  // Serial.print(" err="); Serial.print(error); 
  // Serial.print(" P="); Serial.print(P); 
  // Serial.print(" I="); Serial.print(I); 
  // Serial.print(" D="); Serial.print(D); 
  // Serial.print(" out="); Serial.print(pid_output); 
  // Serial.print(" TL="); Serial.print(Target_L); 
  // Serial.print(" TR="); Serial.println(Target_R); 
} 
 
void distanceMeasure() { 
    /* 测距 */ 
  float readings[MEASURE_COUNT]; 
  int count = 0; 
  const int MAX_TRIES = MEASURE_COUNT * 5;  // 最多尝试次数，避免死循环 
  int tries = 0; 
 
  // 采集有效读数，且不超过 MAX_TRIES 次尝试 
  while (count < MEASURE_COUNT && tries < MAX_TRIES) { 
    tries++; 
 
    digitalWrite(trigPin, LOW); 
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH); 
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW); 
 
    long duration = pulseIn(echoPin, HIGH, 25000);  // 超时 25ms 
    delay(60);                                      // 给模块复位时间 
 
    if (duration == 0) continue;                             // 无回波，
跳过 
    float distance = duration * 0.0343 / 2.0;                // 计算距离 
    if (distance <= 0 || distance > MAX_DISTANCE) continue;  // 异常值，
跳过 
 
    readings[count++] = distance;  // 有效值计入 
  } 
 
  // 排序并取中值 
  for (int i = 0; i < MEASURE_COUNT - 1; i++) { 
    for (int j = i + 1; j < MEASURE_COUNT; j++) { 
      if (readings[j] < readings[i]) { 
        float tmp = readings[i]; 
        readings[i] = readings[j]; 
        readings[j] = tmp; 
      } 
    } 
  } 
  float medianDist = readings[MEASURE_COUNT / 2]; 
  // 串口输出中值 
  Serial.println("Median distance: "); 
  Serial.print(medianDist, 1); 
  Serial.println(" cm"); 
 
} 
