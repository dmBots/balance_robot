/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制底盘两个的电机，两个都是DM3510、，这两个电机挂载在can1总线上
	*						 从底盘上往下看，开发板电源输入口朝右，左边电机发送id为0x01、接收id为0x11，
	*            右边电机发送id为0x02、接收id为0x12
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include <math.h>

extern INS_t INS;
																
chassis_t chassis_move;

uint32_t CHASSR_TIME=1;	

//float Turn_Kp=0.4f;//0.8f;
//float Turn_Kd=0.04f;//0.05f;
float Turn_Kp=0.3f;//0.3f;
float Turn_Kd=0.04f;//0.04f;
//转向环
float Turn(float Angle,float Gyro)
{  
	 float tor3;

	 if(chassis_move.turn_flag==1)
	 {
		 tor3=(chassis_move.turn_set-Gyro)*Turn_Kd;
	 }
	 else if(chassis_move.turn_flag==0)
	 {
	   tor3=Turn_Kp*(chassis_move.turn_set-Angle)-Gyro*Turn_Kd; 
	 } 
	 return tor3;
}

float turn_T=0.0f;

//float lqr_k[2][4]={    { -0.0060f  , -0.0628f  , -0.7354f   ,-0.0659f},
//   {-0.0060f ,  -0.0628f , -0.7354f  , -0.0659f}};
float lqr_k[2][4]={    {-0.0060f   , -0.0613f    ,-0.6800f ,   -0.0656f} ,
   {-0.0060f  ,  -0.0613f  ,  -0.6800f  ,  -0.0656f}};
void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}

  ChassisR_init(&chassis_move);
	
	while(1)
	{	
		chassisR_feedback_update(&chassis_move,&INS);//更新数据
							
    chassis_move.wheel_motor[0].wheel_T=lqr_k[0][0]*(chassis_move.x_set-chassis_move.x)
																				+lqr_k[0][1]*(0.5f*chassis_move.v_set-chassis_move.v)
																				+lqr_k[0][2]*(0.05f-chassis_move.myPithR)//0.05rad是机械中值
																				+lqr_k[0][3]*(chassis_move.d_phi_set-chassis_move.myPithGyroR);
		
		chassis_move.wheel_motor[1].wheel_T=lqr_k[1][0]*(chassis_move.x_set-chassis_move.x)
																				+lqr_k[1][1]*(0.5f*chassis_move.v_set-chassis_move.v)
																				+lqr_k[1][2]*(0.05f-chassis_move.myPithR)
																				+lqr_k[1][3]*(chassis_move.d_phi_set-chassis_move.myPithGyroR);
		
		chassis_move.wheel_motor[0].wheel_T=0.0f-chassis_move.wheel_motor[0].wheel_T;
		
		turn_T= Turn(chassis_move.total_yaw,INS.Gyro[2]);
		
		chassis_move.wheel_motor[0].wheel_T=chassis_move.wheel_motor[0].wheel_T-turn_T;
		chassis_move.wheel_motor[1].wheel_T=chassis_move.wheel_motor[1].wheel_T-turn_T;

		mySaturate(&chassis_move.wheel_motor[0].wheel_T,-0.18f,0.18f);
		mySaturate(&chassis_move.wheel_motor[1].wheel_T,-0.18f,0.18f);
			
		if(chassis_move.start_flag==1)	
		{
			mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[0].wheel_T);//左边电机
			mit_ctrl2(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,chassis_move.wheel_motor[1].wheel_T);//右边电机
			osDelay(CHASSR_TIME);
		}
		else if(chassis_move.start_flag==0)	
		{
			chassis_move.turn_set=chassis_move.total_yaw;
			mit_ctrl2(&hfdcan1,0x01, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//左边电机	
			mit_ctrl2(&hfdcan1,0x02, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//右边电机	
			osDelay(CHASSR_TIME);
		}
	
	}
}

void ChassisR_init(chassis_t *chassis)
{
	wheel_motor_init(&chassis->wheel_motor[0],0x01,MIT_MODE);//发送id为1

	wheel_motor_init(&chassis->wheel_motor[1],0x02,MIT_MODE);//发送id为2
	
	for(int j=0;j<10;j++)
	{//左边
    enable_motor_mode(&hfdcan1,chassis->wheel_motor[0].para.id,chassis->wheel_motor[0].mode);//右边轮毂电机
	  osDelay(1);
	}
	for(int j=0;j<10;j++)
	{
    enable_motor_mode(&hfdcan1,chassis->wheel_motor[1].para.id,chassis->wheel_motor[1].mode);//右边轮毂电机
	  osDelay(1);
	}	
}

int64_t time=0;
void chassisR_feedback_update(chassis_t *chassis,INS_t *ins)
{
	chassis->myPithR=ins->Pitch;
	chassis->myPithGyroR=ins->Gyro[1];
	
	chassis->total_yaw=ins->YawTotalAngle;
	chassis->v=((chassis->wheel_motor[1].para.vel-chassis->wheel_motor[0].para.vel)/2.0f)*0.03375f;//0.03375m是轮子半径
	chassis->x=chassis->x+chassis->v*0.001f;//控制周期为1ms
	
	if(chassis_move.wheel_motor[0].para.vel>180.0f||chassis_move.wheel_motor[0].para.vel<-180.0f
		||chassis_move.wheel_motor[1].para.vel>180.0f||chassis_move.wheel_motor[1].para.vel<-180.0f)
	{//速度过大，应该是被抬起来了
	  
	  time++;
		if(time>1500)
		{
		  time=0;
			chassis->start_flag=0;
		  chassis->turn_flag=0;
		}		
	}
	else
	{
	  time=0;
	}

}


void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}





