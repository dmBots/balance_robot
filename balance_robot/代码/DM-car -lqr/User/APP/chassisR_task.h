#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"

#include "INS_task.h"

typedef struct
{

  Wheel_Motor_t wheel_motor[2];
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
  float v;//实际的速度,单位是m/s
	float x;//实际的位移，单位是m
	
	
	float phi_set;
  float d_phi_set;
		
	
	float turn_set;//期望yaw轴弧度

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;

	float total_yaw;
	
	uint8_t start_flag;//启动标志
	uint8_t front_flag;
	uint8_t turn_flag;

} chassis_t;


extern void ChassisR_init(chassis_t *chassis);
extern void ChassisR_task(void);
extern void mySaturate(float *in,float min,float max);
extern void chassisR_feedback_update(chassis_t *chassis,INS_t *ins);


#endif




