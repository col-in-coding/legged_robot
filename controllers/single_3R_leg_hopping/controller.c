
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include <webots/keyboard.h>

#include "easyMat.h"
#include "controller.h"

robotTypeDef robot;
//----------------------------------------------------------declaration
void updateRobotStateMachine();
void forwardKinematics(matTypeDef* workPoint, jointSpaceTypeDef* jointPoint);
void update_xz_dot();
void update_last_Ts();
void update_xz_dot_desire();
double get_cur_vleg_len();
double get_pre_vleg_len();

/*
机器人参数初始化，在while（1）之前调用
设初始状态来弹簧压缩状态
初始角度：
hipZ = -30
knee = 60
*/
void robot_init()
{
  printf("Robot Init \n");
  //------------------------------------------------------------------------参数区
  robot.virtual_leg_norm_len    = 0.95   ;     //虚拟弹簧腿原长
  robot.thigh_len               = 0.5   ;     //大腿长度
  robot.calf_len                = 0.5   ;     //小腿长度
  robot.v                       = 0.2  ;     //机器人水平运动速度
  robot.r_threshold             = 0.95  ;     //用于状态机切换的腿长阈值系数
  robot.k_spring_p              = 3000  ;     //弹簧刚度
  robot.k_spring_v              = 0.8     ;     //弹簧阻尼   
  robot.F_thrust                = 80.0 ;     //THRUST推力，用于调节跳跃高度
  robot.k_leg_p                 = 6     ;     //腿部控制时的kp
  robot.k_leg_v                 = 0.8   ;     //腿部控制时的kv 
  robot.k_xz_dot                = 0.072 ;     //净加速度系数
  robot.k_pose_p                = 0.8   ;     //姿态控制时的kp
  robot.k_pose_v                = 0.025 ;     //姿态控制时的kv

  //------------------------------------------------------------------------状态区
  //欧拉角，Roll,Pitch,Yaw
  robot.eulerAngle.roll         = 0;
  robot.eulerAngle.pitch        = 0;
  robot.eulerAngle.yaw          = 0;
  //欧拉角的导数，Roll,Pitch,Yaw
  robot.eulerAngle_dot.roll     = 0;
  robot.eulerAngle_dot.pitch    = 0;
  robot.eulerAngle_dot.yaw      = 0;
  //从{B}坐标系到{H}坐标系的转换
  easyMat_create(&robot.R_H_B, 3, 3);
  easyMat_eye(&robot.R_H_B);
  //从{H}坐标系到{B}坐标系的转换
  easyMat_create(&robot.R_B_H, 3, 3);
  easyMat_eye(&robot.R_B_H);             
  //关节空间
  robot.jointPoint.hipX_motor_angle = 0;
  robot.jointPoint.hipZ_motor_angle = -30;
  robot.jointPoint.knee_motor_angle = 60;    
  //关节空间的导数
  robot.jointPoint_dot.hipX_motor_angle = 0;
  robot.jointPoint_dot.hipZ_motor_angle = 0;
  robot.jointPoint_dot.knee_motor_angle = 0;
  //{B}坐标系工作空间
  easyMat_create(&robot.workPoint_B, 3, 1);
  easyMat_clear(&robot.workPoint_B);
  //{H}坐标系工作空间
  easyMat_create(&robot.workPoint_H, 3, 1);
  easyMat_clear(&robot.workPoint_H);
 //{B}坐标系工作空间期望值    
  easyMat_create(&robot.workPoint_B_desire, 3, 1); 
  easyMat_clear(&robot.workPoint_B_desire); 
 //{H}坐标系工作空间期望值    
  easyMat_create(&robot.workPoint_H_desire, 3, 1);  
  easyMat_clear(&robot.workPoint_H_desire);
  
  robot.is_foot_touching    = true;       //足底是否触地
  robot.Ts                  = 0;          //上一个支撑相持续时间
  robot.x_dot               = 0;          //机身x方向水平速度
  robot.z_dot               = 0;          //机身z方向水平速度
  robot.x_dot_desire        = 0;          //机身x方向期望水平速度
  robot.z_dot_desire        = 0;          //机身z方向期望水平速度
  robot.system_ms           = 0;          //从仿真启动开始的计时器
  robot.stateMachine        = THRUST;     //状态机
}
/*
机器人内存空间释放
*/
void robot_free()
{
  easyMat_free(&robot.R_H_B);
  easyMat_free(&robot.R_B_H);
  easyMat_free(&robot.workPoint_B);
  easyMat_free(&robot.workPoint_H);
  easyMat_free(&robot.workPoint_B_desire);  
  easyMat_free(&robot.workPoint_H_desire);  
}
/*
机器人状态更新，包括读取传感器的数据以及一些状态估计
*/
void updateRobotState()
{
  /*时钟更新*/
  robot.system_ms += TIME_STEP;
  /*通过键盘修改期望速度*/
  update_xz_dot_desire();
  /*足底传感器更新*/
  robot.is_foot_touching = is_foot_touching();
  /*IMU，IMU导数，以及旋转矩阵更新*/
  eulerAngleTypeDef now_IMU  = get_IMU_Angle();
  eulerAngleTypeDef now_IMU_dot;
  
  now_IMU_dot.roll           = (now_IMU.roll  - robot.eulerAngle.roll )/(0.001*TIME_STEP);
  now_IMU_dot.pitch          = (now_IMU.pitch - robot.eulerAngle.pitch)/(0.001*TIME_STEP);
  now_IMU_dot.yaw            = (now_IMU.yaw   - robot.eulerAngle.yaw  )/(0.001*TIME_STEP);
  robot.eulerAngle = now_IMU;

  robot.eulerAngle_dot.roll  = robot.eulerAngle_dot.roll *0.5 + now_IMU_dot.roll *0.5;
  robot.eulerAngle_dot.pitch = robot.eulerAngle_dot.pitch*0.5 + now_IMU_dot.pitch*0.5;
  robot.eulerAngle_dot.yaw   = robot.eulerAngle_dot.yaw  *0.5 + now_IMU_dot.yaw  *0.5;
  
  easyMat_RPY(&robot.R_H_B, robot.eulerAngle.roll, robot.eulerAngle.pitch, robot.eulerAngle.yaw);
  easyMat_trans(&robot.R_B_H, &robot.R_H_B);

  /*虚拟弹簧腿长度更新*/
  double r = get_cur_vleg_len();
  double pre_r = get_pre_vleg_len();
  double vleg_len_dot = (r - pre_r)/(0.001*TIME_STEP);
  double l1 = robot.thigh_len;
  double l2 = robot.calf_len;
  robot.l = r;
  robot.l_dot = vleg_len_dot;

  /*关节角度更新*/
  double cur_hipX_ang     = get_hipX_motor_angle();
  double cur_hipX_ang_dot = (cur_hipX_ang - robot.jointPoint.hipX_motor_angle)/(0.001*TIME_STEP);
  robot.jointPoint.hipX_motor_angle     = cur_hipX_ang;
  robot.jointPoint_dot.hipX_motor_angle = robot.jointPoint_dot.hipX_motor_angle * 0.5 + cur_hipX_ang_dot * 0.5;    //一阶低通滤波器
  
  double cur_hipZ_ang     = get_hipZ_motor_angle();
  double cur_hipZ_ang_dot = (cur_hipZ_ang - robot.jointPoint.hipZ_motor_angle)/(0.001*TIME_STEP);
  robot.jointPoint.hipZ_motor_angle     = cur_hipZ_ang;
  robot.jointPoint_dot.hipZ_motor_angle = robot.jointPoint_dot.hipZ_motor_angle * 0.5 + cur_hipZ_ang_dot*0.5;    //一阶低通滤波器
  
  double cur_knee_ang     = get_knee_motor_angle();
  double cur_knee_ang_dot = (cur_knee_ang - robot.jointPoint.knee_motor_angle)/(0.001*TIME_STEP);
  robot.jointPoint.knee_motor_angle     = cur_knee_ang;
  robot.jointPoint_dot.knee_motor_angle = robot.jointPoint_dot.knee_motor_angle * 0.5 + cur_knee_ang_dot*0.5;

  // 虚拟腿与大腿的夹角
  robot.l_alpha = acos((r*r + l1*l1 - l2*l2)/(2*r*l1));

  /*机器人在世界坐标系下水平速度估计更新*/
  update_xz_dot();
  /*上次支撑相时间Ts更新*/
  update_last_Ts();
  /*更新状态机*/
  updateRobotStateMachine();
}
/*
机器人控制
膝关节控制弹力，臀关节控制姿态
*/
void robot_control()
{
  double T_knee, T_hipX, T_hipZ;
  /*控制弹簧弹性力*/
  double dx = robot.virtual_leg_norm_len - robot.l;  //求压缩量
  double F_spring = robot.k_spring_p * dx + robot.k_spring_v * robot.l_dot;
  if(robot.stateMachine == THRUST)
  {
    F_spring += robot.F_thrust;
  }
  T_knee = - F_spring * robot.thigh_len * sin(robot.l_alpha);
  T_hipX = 0;
  T_hipZ = 0;

  /*控制臀部扭矩力*/
  if((robot.stateMachine == LOADING)||(robot.stateMachine == UNLOADING))
  {
    //LOADING和UNLOADING时候，扭矩为0
    
  }else if((robot.stateMachine == COMPRESSION)||(robot.stateMachine == THRUST))
  {
    //COMPRESSION和THRUST时候，臀部电机控制身体姿态
    T_hipX = -(-robot.k_pose_p*robot.eulerAngle.roll  - robot.k_pose_v*robot.eulerAngle_dot.roll) ;
    T_hipZ = -(-robot.k_pose_p*robot.eulerAngle.pitch - robot.k_pose_v*robot.eulerAngle_dot.pitch);
  }else if(robot.stateMachine == FLIGHT)
  {
    //FLIGHT的时候，控制足底移动到落足点
    double r  = robot.l;

    double x_f = robot.x_dot*robot.Ts/2.0 + robot.k_xz_dot*(robot.x_dot - robot.x_dot_desire);
    double z_f = robot.z_dot*robot.Ts/2.0 + robot.k_xz_dot*(robot.z_dot - robot.z_dot_desire);
    double y_f = -sqrt(r*r -x_f*x_f - z_f*z_f);

    printf("******* Landing point: ************\n");
    printf(" ( %f, %f, %f ) \n", x_f, y_f, z_f);

    robot.workPoint_H_desire.data[0][0] = x_f;
    robot.workPoint_H_desire.data[1][0] = y_f;
    robot.workPoint_H_desire.data[2][0] = z_f;

    //转到{B}坐标系下
    easyMat_mult(&robot.workPoint_B_desire, &robot.R_B_H, &robot.workPoint_H_desire);
    //计算期望关节角
    double x_f_B = robot.workPoint_H_desire.data[0][0];
    double y_f_B = robot.workPoint_H_desire.data[1][0];
    double z_f_B = robot.workPoint_H_desire.data[2][0];

    double x_angle_desire = atan(z_f_B/y_f_B)*180.0/PI;
    double z_angle_desire = asin(x_f_B/r)    *180.0/PI - robot.l_alpha*180/PI;

    //控制关节角
    double x_angle     = robot.jointPoint.hipX_motor_angle;
    double z_angle     = robot.jointPoint.hipZ_motor_angle;
    double x_angle_dot = robot.jointPoint_dot.hipX_motor_angle;
    double z_angle_dot = robot.jointPoint_dot.hipZ_motor_angle;

    T_hipX = -robot.k_leg_p*(x_angle - x_angle_desire)  - robot.k_leg_v*x_angle_dot;
    T_hipZ = -robot.k_leg_p*(z_angle - z_angle_desire)  - robot.k_leg_v*z_angle_dot;
  }

  set_hipX_torque(T_hipX);
  set_hipZ_torque(T_hipZ);
  set_knee_torque(T_knee);
}
/*
通过键盘修改期望速度
*/
void update_xz_dot_desire()
{
   /*读取键盘，获取速度*/
  switch(get_keyboard())
  {
    case WB_KEYBOARD_UP:
    {
      robot.x_dot_desire = robot.v;
      break;
    }
    case WB_KEYBOARD_DOWN:
    {
      robot.x_dot_desire = -robot.v;
      break;
    }
    case WB_KEYBOARD_RIGHT:
    {
      robot.z_dot_desire = robot.v;
      break;
    }
    case WB_KEYBOARD_LEFT:
    {
      robot.z_dot_desire = -robot.v;
      break;
    }
    default:
    {
      robot.z_dot_desire = 0;
      robot.x_dot_desire = 0;
      break;
    }
  }
}
/*
计算目前虚拟弹簧腿长
*/
double get_cur_vleg_len()
{
  double l1, l2, theta2, res; 
  l1 = robot.thigh_len;
  l2 = robot.calf_len;
  theta2 = get_knee_motor_angle();
  res = sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(theta2 * PI / 180));
  return res;
}
/*
计算上一次虚拟弹簧腿长
*/
double get_pre_vleg_len()
{
  double l1 = robot.thigh_len;
  double l2 = robot.calf_len;
  double theta2 = robot.jointPoint.knee_motor_angle;
  return sqrt(l1 * l1 + l2 * l2 + 2 * l1 * l2 * cos(theta2 * PI / 180));
}
/*
上次支撑相时间Ts更新
*/
void update_last_Ts()
{
   static bool pre_is_foot_touching = false;
   static int stance_start_ms = 0;
   if((pre_is_foot_touching == false)&&(robot.is_foot_touching == true)) //此时进入支撑相
   {
     stance_start_ms = robot.system_ms;
   }
   if((pre_is_foot_touching == true)&&(robot.is_foot_touching == false)) //此时进入摆动相
   {
     int stance_end_ms = robot.system_ms;
     robot.Ts = 0.001*(double)(stance_end_ms - stance_start_ms);
   }
   pre_is_foot_touching = robot.is_foot_touching;
}
/*
机器人在世界坐标系下水平速度估计更新
*/
void update_xz_dot()
{
    //正运动学
    forwardKinematics(&robot.workPoint_B, &robot.jointPoint);
      
    //转换到{H}坐标系下
    double pre_x = robot.workPoint_H.data[0][0];
    double pre_z = robot.workPoint_H.data[2][0];
    easyMat_mult(&robot.workPoint_H, &robot.R_H_B, &robot.workPoint_B);
    double now_x = robot.workPoint_H.data[0][0];
    // double now_y = robot.workPoint_H.data[1][0];
    double now_z = robot.workPoint_H.data[2][0];

    //求导
    double now_x_dot = -(now_x - pre_x)/(0.001*TIME_STEP);
    double now_z_dot = -(now_z - pre_z)/(0.001*TIME_STEP);

    //滤波
    static double pre_x_dot = 0;
    static double pre_z_dot = 0;
    now_x_dot = pre_x_dot*0.5 + now_x_dot*0.5;
    now_z_dot = pre_z_dot*0.5 + now_z_dot*0.5;
    pre_x_dot = now_x_dot;
    pre_z_dot = now_z_dot;
    
    if((robot.stateMachine == COMPRESSION)||(robot.stateMachine == THRUST))
    {
      robot.x_dot = now_x_dot;
      robot.z_dot = now_z_dot;

      // printf("##################################################\n");
      // printf("----------- Velocity Updating ------------\n");
      // printf("Vx: %f, Vz: %f \n", robot.x_dot, robot.z_dot);

      // printf("---------- End Effact Position -----------\n");
      // printf("B frame: ( %f, %f, %f ) \n",
      //   robot.workPoint_B.data[0][0],
      //   robot.workPoint_B.data[1][0],
      //   robot.workPoint_B.data[2][0]
      // );
      // printf("H frame: ( %f, %f, %f ) \n", now_x, now_y, now_z);


    }

}

/*
机器人状态机切换
---------------------------------------------------
|  名称     |     代码       |       触发条件      |
---------------------------------------------------
|  落地     |   LOADING      |   足底传感器触地    |
|  压缩腿   |   COMPRESSION  |   腿长小于阈值      |
|  伸长腿   |   THRUST       |   腿长导数为正      |
|  离地     |   UNLOADING    |   腿长大于阈值      |
|  飞行     |   FLIGHT       |   足底传感器离地    |
---------------------------------------------------
*/
void updateRobotStateMachine()
{
  switch(robot.stateMachine)
  {
    case LOADING:
    {
      // printf("-------------robot.stateMachine: LOADING\n");
      if(robot.l < robot.virtual_leg_norm_len * robot.r_threshold)
        robot.stateMachine = COMPRESSION;
      break;
    }
    case COMPRESSION:
    {
      // printf("-------------robot.stateMachine: COMPRESSION\n");
      if(robot.l_dot > 0)
        robot.stateMachine = THRUST;
      break;
    }
    case THRUST:
    {
      // printf("-------------robot.stateMachine: THRUST\n");
      if(robot.l > robot.virtual_leg_norm_len * robot.r_threshold)
        robot.stateMachine = UNLOADING;
      break;
    }
    case UNLOADING:
    {
      // printf("-------------robot.stateMachine: UNLOADING\n");
      if(robot.is_foot_touching == false)
        robot.stateMachine = FLIGHT;
      break;
    }
    case FLIGHT:
    {
      // printf("-------------robot.stateMachine: FLIGHT\n");
      if(robot.is_foot_touching == true)
      robot.stateMachine = LOADING;
      break;
    }
    default: break;
  }
}
/*
运动学正解
{B}坐标系下足底坐标 = forwardKinematics(关节角度)
DH 模型：
T_B0 = Rot(x, theta0)
T_01 = Rot(z, theta1)Trans(0, -thigh_len, 0)
T_12 = Rot(z, theta2)Trans(calf_len, 0, 0)
End effect: (p, 1) = T_B0 * T_01 * T12
*/
void forwardKinematics(matTypeDef* workPoint, jointSpaceTypeDef* jointPoint)
{
  double theta0 = jointPoint->hipX_motor_angle;
  double theta1 = jointPoint->hipZ_motor_angle;
  double theta2 = jointPoint->knee_motor_angle;
  // printf("theta: %f  %f  %f\n", theta0, theta1, theta2);

  // Homogeneious Transformation Matrix
  matTypeDef T_B, T_B0, T_01, T_12, temp;
  easyMat_create(&T_B, 4, 4);
  easyMat_create(&T_B0, 4, 4);
  easyMat_create(&T_01, 4, 4);
  easyMat_create(&T_12, 4, 4);
  easyMat_create(&temp, 4, 4);
  easyMat_eye(&T_B0);
  easyMat_eye(&T_01);
  easyMat_eye(&T_12);

  easyMat_rotX(&T_B0, theta0);
  easyMat_rotZ(&T_01, theta1);
  easyMat_rotZ(&T_12, theta2);

  double p_01[3] = {0, -robot.thigh_len, 0};
  easyMat_translate(&T_01, p_01);
  double p_12[3] = {0, -robot.calf_len, 0};
  easyMat_translate(&T_12, p_12);

  easyMat_mult(&temp, &T_B0, &T_01);
  easyMat_mult(&T_B, &temp, &T_12);

  workPoint->data[0][0] = T_B.data[0][3];
  workPoint->data[1][0] = T_B.data[1][3];
  workPoint->data[2][0] = T_B.data[2][3];
  
  easyMat_free(&T_B);
  easyMat_free(&T_B0);
  easyMat_free(&T_01);
  easyMat_free(&T_12);
  easyMat_free(&temp);
}