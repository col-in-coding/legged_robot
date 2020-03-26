
#ifndef _WEBOTSINTERFACE_H_
#define _WEBOTSINTERFACE_H_

//-----------------------------------------------------------macro
#define PI           (3.141892654)
#define TIME_STEP    (2)
//-----------------------------------------------------------typedef
/*
1，陀螺仪数据定义,为了方便调试采用角度制，注意，采用角度制
2，webots IMU模块采用RPY角度制，定系旋转，矩阵左乘，即：
       Rot=RotY(yaw)*RotZ(pitch)*RotX(roll);
*/
typedef struct
{
  double roll;       //横滚，x轴
  double pitch;      //俯仰，z轴
  double yaw;        //偏航，y轴
}eulerAngleTypeDef;

//-----------------------------------------------------------extern
extern void              webots_device_init               ();
extern void              set_hipX_torque        (double torque);
extern void              set_hipZ_torque        (double torque);
extern void              set_knee_torque        (double torque);
extern double            get_hipX_motor_angle             ();
extern double            get_hipZ_motor_angle             ();
extern double            get_knee_motor_angle             ();
extern bool              is_foot_touching                 ();
extern int               get_keyboard                     ();
extern eulerAngleTypeDef get_IMU_Angle                    ();
#endif

