/*
 * File:          single_3R_leg_hopping.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <stdio.h>
#include <stdlib.h>

#include <webots/robot.h>
#include <webots/keyboard.h>

#include "webotsInterface.h"
#include "controller.h"

int main(int argc, char **argv) {
  
  wb_robot_init();
  webots_device_init();                           //webots设备初始化
  robot_init();                                   //机器人初始化
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
      updateRobotState();                             //机器人状态更新
      robot_control();                                //机器人控制  
  }
  robot_free();                                   //机器人释放内存空间
  wb_robot_cleanup();
  return 0;
}