/**
*通过串口调节并找到PID参数实例
*
*使用串口终端用户可以发送电机命令，实时配置电机和FOC，你可以在这个例程中实现如下功能:
* -配置PID控制器常数
* -改变运动控制回路
* -监控电机变量
* -设置目标值
* -检查所有配置值
*
*输入命令字母即可查看命令字符对应的电机参数值
*例如:当你需要读取速度环P环值时，串口中输入: P
*再例如：设置速度PI控制器P增益为1.2，时，串口中输入：P1.2
*
*你也可以在串口中直接输入对应的弧度数字，命令电机转到该弧度位置
*
*串口命令列表:
* - P:速度PID控制器P增益
* - I:速度PID控制器I增益
* - D:速度PID控制器D增益
* - R:速度PID控制器电压斜坡
* - F:速度低通滤波器时间常数
* - K:角度P控制器P增益
* - N:角度P控制器速度限制
* - L:系统电压限制
* - 0:电压
* - 1:速度
* - 2:角度
* - V:获取运动目标弧度值
* - 0:输出当前设置的电压
* - 1:输出当前速度
* - 2:输出当前角度
* - 3:输出当前目标值
*
 * 
 */
#include <SimpleFOC.h>

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

// Motor instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 18);

BLDCMotor motor1 = BLDCMotor(7);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 12);

void setup() {
  I2Cone.begin(18, 5, 400000); 
  I2Ctwo.begin(19, 23, 400000);
  sensor.init(&I2Cone);
  sensor1.init(&I2Ctwo);
  //把电机和传感器连接起来
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //驱动器设置
  // 供电电压设置 [V]
  driver.voltage_power_supply = 16.8;
  driver.init();

  driver1.voltage_power_supply = 16.8;
  driver1.init();
  //把电机和驱动连接起来
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);
  
  //选择FOC调制方式
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //设置FOC控制模式为角度控制模式
  motor.controller = ControlType::angle;
  motor1.controller = ControlType::angle;


  //速度环P环参数
  motor.PID_velocity.P = 0.2;
  motor1.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor1.PID_velocity.I = 20;
  //设置电机的最大电压
  motor.voltage_limit = 10;
  motor1.voltage_limit = 10;
  
  //磁传感器低通滤波时间常数（越大磁传感器数据滤波效果越好，但是越滞后）
  motor.LPF_velocity.Tf = 0.01;
  motor1.LPF_velocity.Tf = 0.01;

  //角度P环控制器设置
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //运行角度控制时的电机最大运动速度
  motor.velocity_limit = 40;
  motor1.velocity_limit = 40;

  //设置串口
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //初始化电机
  motor.init();
  motor1.init();
  motor.initFOC();
  motor1.initFOC();


  Serial.println("Motor ready");
  Serial.println("Input your Command:");
  _delay(1000);
}

//设置转动角度初始变量
float target_angle = 0;

void loop() {
  motor.loopFOC();
  motor1.loopFOC();

  motor.move();
  motor.command(serialReceiveUserCommand());

  motor1.move();
  motor1.command(serialReceiveUserCommand());


  serialReceiveUserCommand();
}

//串口收发函数
String serialReceiveUserCommand() {
  

  static String received_chars;
  
  String command = "";

  while (Serial.available()) {

    char inChar = (char)Serial.read();

    received_chars += inChar;


    if (inChar == '\n') {

      command = received_chars;

      received_chars = "";
    }
  }
  return command;
}
