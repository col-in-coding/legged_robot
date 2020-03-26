
#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <stdbool.h>

#include "easyMat.h"
#include "webotsInterface.h"

//-----------------------------------------------------------macro

//-----------------------------------------------------------typedef
/*
机器人 状态机 枚举类型定义
*/
typedef enum
{
  LOADING      = 0x00,    //落地
  COMPRESSION  = 0x01,    //压缩腿
  THRUST       = 0x02,    //伸长腿
  UNLOADING    = 0x03,    //离地
  FLIGHT       = 0x04     //飞行
}stateMachineTypeDef;
/*
机器人 关节空间 类型定义
*/
typedef struct
{
  double hipX_motor_angle;    //臀关节x轴扭转角，角度制
  double hipZ_motor_angle;    //臀关节z轴扭转角，角度制
  double knee_motor_angle;    //膝关节x轴扭转角，角度制
}jointSpaceTypeDef;
/*
机器人定义,包括机器人状态和机器人参数
注 意：备注“ // ”中加“ * ”符号的，需要同时修改场景树中相关内容才能生效
*/
typedef struct
{
  //------------------------------------------------------------------------参数区
  double              virtual_leg_norm_len;   //*虚拟弹簧腿原长
  double              thigh_len;              //*大腿长度
  double              calf_len;               //*小腿长度
  double              k_spring_p;             //弹簧刚度
  double              k_spring_v;             //弹簧阻尼
  double              F_thrust;               //THRUST推力
  double              k_xz_dot;               //净加速度系数
  double              r_threshold;            //状态机在脱离LOADING和进入UNLOADING状态时，腿长阈值判断
  double              v;                      //机器人水平运动速度
  double              k_pose_p;               //姿态控制时的kp
  double              k_pose_v;               //姿态控制时的kv
  double              k_leg_p;                //腿部控制时的kp
  double              k_leg_v;                //腿部控制时的kv
  //------------------------------------------------------------------------状态区
  eulerAngleTypeDef   eulerAngle;             //欧拉角，Roll,Pitch,Yaw
  eulerAngleTypeDef   eulerAngle_dot;         //欧拉角的导数，Roll,Pitch,Yaw
  matTypeDef          R_H_B;                  //从{B}坐标系到{H}坐标系的转换
  matTypeDef          R_B_H;                  //从{H}坐标系到{B}坐标系的转换
  
  jointSpaceTypeDef   jointPoint;             //关节空间
  jointSpaceTypeDef   jointPoint_dot;         //关节空间的导数
  matTypeDef          workPoint_B;            //{B}坐标系工作空间
  matTypeDef          workPoint_H;            //{H}坐标系工作空间
  matTypeDef          workPoint_B_desire;     //{B}坐标系工作空间期望值
  matTypeDef          workPoint_H_desire;     //{H}坐标系工作空间期望值
  
  bool                is_foot_touching;       //足底是否触地
  double              Ts;                     //上一个支撑相持续时间
  double              l;                      //虚拟腿长 
  double              l_dot;                  //虚拟腿长变化
  double              l_alpha;                //虚拟退与大腿的夹角
  double              x_dot;                  //机身x方向水平速度
  double              z_dot;                  //机身z方向水平速度
  double              x_dot_desire;           //机身x方向期望水平速度
  double              z_dot_desire;           //机身z方向期望水平速度
  int                 system_ms;              //从仿真启动开始的计时器
  stateMachineTypeDef stateMachine;           //状态机
}robotTypeDef;
//-----------------------------------------------------------extern
extern robotTypeDef robot;

extern void robot_init           ();
extern void updateRobotState     ();
extern void robot_control        ();
extern void robot_free           ();

#endif

