/*AB相编码器直流电机=uno 引脚*/
#define MORTOR_C1 2                //中断口0是2 INT0
#define MORTOR_C2 12               // Right motor

// #define MORTOR_C1 3                //中断口1是3 INT1
// #define MORTOR_C2 13               //Left motor

#define MORTOR_REDUCTION 90        //减速比
#define MORTOR_PULSE_PER_ROUND 11  //编码器每周输出脉冲个数
/*
L298N模块连接 =uno 引脚
*/
#define L298N_EN_MORTOR 10  //B-Right motor。PWM 默认频率 490hz
#define L298N_MORTOR_IN_X 6
#define L298N_MORTOR_IN_Y 5

// #define L298N_EN_MORTOR 9  ///A-Left motor。PWM 默认频率 490hz
// #define L298N_MORTOR_IN_X 8
// #define L298N_MORTOR_IN_Y 7
volatile double pulse = 0;  //如果是正转，那么每计数一次自增1，如果是反转，那么每计数一次自减1

void pulse_C1() {
  //2倍频计数实现
  //手动旋转电机一圈，输出结果为 一圈脉冲数 * 减速比 * 2
  if (digitalRead(MORTOR_C1) == HIGH) {

    if (digitalRead(MORTOR_C2) == HIGH) {  //A 高 B 高
      pulse++;
    } else {  //A 高 B 低
      pulse--;
    }


  } else {

    if (digitalRead(MORTOR_C2) == LOW) {  //A 低 B 低
      pulse++;
    } else {  //A 低 B 高
      pulse--;
    }
  }
}
long interval_time = 100;  //一个计算周期100ms
long start_time = millis();

#include <PID_v1.h>
//-------------------------------------PID-------------------------------------------
//创建 PID 对象
//1.当前转速 2.计算输出的pwm 3.目标转速 4.kp 5.ki 6.kd 7.当输入与目标值出现偏差时，向哪个方向控制
double pwm=0;              //电机驱动的PWM值
double current_vel = 0;  //车轮每分钟转数
double target = 40;      //预期车轮每分钟转数
double kp = 1.5, ki = 3.0, kd = 0.1;
// PID pid(&current_vel, &pwm, &target, kp, ki, kd, DIRECT);//Porprotion On Error as usual
PID pid(&current_vel, &pwm, &target, kp, ki, kd, 0,DIRECT);//Porprotion On Measurement //从测试驱动直流电机来看，好像没什么太大改进，可能我的测试还不太严谨
//速度更新函数
void update_vel() {
  //获取当前速度
  long right_now = millis();
  long past_time = right_now - start_time;                                //计算逝去的时间
  if (past_time >= interval_time) {                                       //如果逝去时间大于等于一个计算周期
    noInterrupts();                                                       // 关闭所有中断
    current_vel = (double)pulse / (2 * 11 * 90) / past_time * 1000 * 60;  //车轮每分钟转数
    // Serial.print("current_vel:");
    // Serial.println(current_vel);

    //4.重置开始时间和
    start_time = right_now;
    pulse = 0;
    interrupts();  // 启动中断允许
                   /*匹配串口绘图器的多参数格式*/
    Serial.print("current_vel:");
    Serial.print(current_vel);
    Serial.print(",target:");
    Serial.println(target);
    /*匹配串口绘图器的多参数格式*/
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);  //设置波特率
  /*设置编码输入*/
  pinMode(MORTOR_C1, INPUT);
  pinMode(MORTOR_C2, INPUT);
  /*设置L298N的输出引脚*/
  pinMode(L298N_EN_MORTOR, OUTPUT);
  pinMode(L298N_MORTOR_IN_X, OUTPUT);
  pinMode(L298N_MORTOR_IN_Y, OUTPUT);
  /*0号中断*/
  attachInterrupt(0, pulse_C1, CHANGE);  //当电平发生改变时触发中断0函数
  pid.SetMode(AUTOMATIC);
  digitalWrite(L298N_MORTOR_IN_Y, HIGH);  //给高电平
  digitalWrite(L298N_MORTOR_IN_X, LOW);   //给低电平
}

void loop() {
  // put your main code here, to run repeatedly:
  // noInterrupts();  // 关闭所有中断
  // Serial.println(pulse);//手动转一周应当输出   MORTOR_REDUCTION*MORTOR_PULSE_PER_ROUND*2    【此处2倍频统计】
  // interrupts();  // 启动中断允许
  delay(10);
  update_vel();
  pid.Compute();  //计算需要输出的PWM
  analogWrite(L298N_EN_MORTOR, pwm);
}
