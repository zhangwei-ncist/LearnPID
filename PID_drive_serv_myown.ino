/*AB相编码器直流电机=uno 引脚*/
#define MORTOR_C1 2   //中断口0是2 INT0
#define MORTOR_C2 12  // Right motor

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
/*PID 相关参数*/
unsigned int sample_time = 100;  //100ms为默认采样时间
unsigned long start_time = millis();
volatile double pulse = 0;  //如果是正转，那么每计数一次自增1，如果是反转，那么每计数一次自减1
double kp = 1.5, ki = 1, kd =0.1;
double target_wheel_vel = 55;                              //目标转速
double measure_wheel_vel = 0, previous_measure_wheel_vel=0;  //测量转速
double error = 0, previous_error = 0, sum_error = 0;       //转速误差
double out_pwm = 0, previous_out_pwm = 0;                  //pid计算得到的pwm



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
//速度更新函数
void getWheelVel() {
  //获取当前速度
  long right_now = millis();
  long past_time = right_now - start_time;                                                                        //计算逝去的时间
  if (past_time >= sample_time) {                                                                                 //如果逝去时间大于等于一个计算周期
    noInterrupts();                                                                                               // 关闭所有中断
    previous_measure_wheel_vel = measure_wheel_vel;                                                               //保留轮速，在计算新的
    measure_wheel_vel = (double)pulse / (2 * MORTOR_PULSE_PER_ROUND * MORTOR_REDUCTION) / past_time * 1000 * 60;  //车轮每分钟转数

    //4.重置开始时间和脉冲
    start_time = right_now;
    pulse = 0;
    interrupts();  // 启动中断允许
                   /*匹配串口绘图器的多参数格式*/
    Serial.print("sum_error:");
    Serial.print(sum_error);
    Serial.print(",measure_wheel_vel:");
    Serial.print(measure_wheel_vel);
    Serial.print(",target_wheel_vel:");
    Serial.println(target_wheel_vel);
    /*匹配串口绘图器的多参数格式*/

    //开始pid
    previous_error = error;  //保留上次的误差
    error = target_wheel_vel - measure_wheel_vel;
    sum_error += error;  //求和
    out_pwm = kp * error + ki * sum_error + kd * (error - previous_error);//计算pid
    if (out_pwm > 255) {
      out_pwm = 255;
      // sum_error=0; 何时清零合适呢？还是一直带着
    }
    if (out_pwm < 0) {
      out_pwm = 0;
      // sum_error=0;
    }
  }
}
void setup() {
  // put your setup code here, to run once:
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
  attachInterrupt(0, pulse_C1, CHANGE);   //当电平发生改变时触发中断0函数
  digitalWrite(L298N_MORTOR_IN_Y, HIGH);  //给高电平
  digitalWrite(L298N_MORTOR_IN_X, LOW);   //给低电平
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
  getWheelVel();
  analogWrite(L298N_EN_MORTOR, out_pwm);
}
