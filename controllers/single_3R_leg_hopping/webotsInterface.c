#include <stdio.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/touch_sensor.h>
#include <webots/keyboard.h>
#include <webots/inertial_unit.h>

#include "webotsInterface.h"

//-----------------------------------------------------------device
WbDeviceTag touch_sensor;
WbDeviceTag IMU;
WbDeviceTag hipX_motor;
WbDeviceTag hipZ_motor;
WbDeviceTag knee_motor;
WbDeviceTag hipX_motor_pos_sensor;
WbDeviceTag hipZ_motor_pos_sensor;
WbDeviceTag knee_motor_pos_sensor;
/*
函数功能：初始化devices
*/
void webots_device_init()
{
  //get device
  touch_sensor          = wb_robot_get_device("touch sensor");
  IMU                   = wb_robot_get_device("inertial unit");
  hipX_motor            = wb_robot_get_device("hipX rotational motor");
  hipZ_motor            = wb_robot_get_device("hipZ rotational motor");
  knee_motor            = wb_robot_get_device("knee rotational motor");
  hipX_motor_pos_sensor = wb_robot_get_device("hipX position sensor");
  hipZ_motor_pos_sensor = wb_robot_get_device("hipZ position sensor");
  knee_motor_pos_sensor = wb_robot_get_device("knee position sensor");
  //enable
  wb_position_sensor_enable(hipX_motor_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(hipZ_motor_pos_sensor, TIME_STEP);
  wb_position_sensor_enable(knee_motor_pos_sensor, TIME_STEP);
  wb_touch_sensor_enable(touch_sensor,TIME_STEP);
  wb_keyboard_enable(TIME_STEP);
  wb_inertial_unit_enable(IMU,TIME_STEP);
}
//-----------------------------------------------------------motor
/*
函数功能：设置臀部 X 轴电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 X 轴旋转的力
*/
void set_hipX_torque(double torque)
{
  if (torque > 1000) torque = 1000;
  if (torque < -1000) torque = -1000;
  wb_motor_set_torque(hipX_motor, torque);
}
/*
函数功能：设置臀部 Z 轴电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 Z 轴旋转的力
*/
void set_hipZ_torque(double torque)
{
  if (torque > 1000) torque = 1000;
  if (torque < -1000) torque = -1000;
  wb_motor_set_torque(hipZ_motor, torque);
}
/*
函数功能：设置膝关节电机的扭矩
注    意：扭矩正方向定义为：机身不动，使腿绕 X 轴旋转的力
*/
void set_knee_torque(double torque)
{
  if (torque > 2000) torque = 2000;
  if (torque < -2000) torque = -2000;
  wb_motor_set_torque(knee_motor, torque);
}
//-----------------------------------------------------------sensor
/*
函数功能：获取臀关节 X 轴电机角度,角度制
*/
double get_hipX_motor_angle()
{
  double angle = wb_position_sensor_get_value(hipX_motor_pos_sensor);
  return angle*180.0f/PI;
}
/*
函数功能：获取臀关节 Z 轴电机角度,角度制
*/
double get_hipZ_motor_angle()
{
  double angle = wb_position_sensor_get_value(hipZ_motor_pos_sensor);
  return angle*180.0f/PI;
}
/*
函数功能：获取膝关节 X 轴电机角度,角度制
*/
double get_knee_motor_angle()
{
  double angle = wb_position_sensor_get_value(knee_motor_pos_sensor);
  return angle*180.0f/PI;
}
/*
函数功能：检测足底是否接触地面
*/
bool is_foot_touching()
{
  return wb_touch_sensor_get_value(touch_sensor);
}

/*
函数功能：读取IMU数据
*/
eulerAngleTypeDef get_IMU_Angle()
{
  const double* data = wb_inertial_unit_get_roll_pitch_yaw(IMU);
  
  eulerAngleTypeDef eulerAngle;
  eulerAngle.roll  = data[0]*180.0f/PI;
  eulerAngle.pitch = data[1]*180.0f/PI;
  eulerAngle.yaw   = data[2]*180.0f/PI;
  
  return eulerAngle;
}

//-----------------------------------------------------------keyboard
/*
函数功能：读取键盘键值
*/
int get_keyboard()
{
  return wb_keyboard_get_key();
}

