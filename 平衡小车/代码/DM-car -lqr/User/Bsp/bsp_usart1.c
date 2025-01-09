#include "bsp_usart1.h"
#include "chassisR_task.h"

extern UART_HandleTypeDef huart1;

extern chassis_t chassis_move;
send_data_t send_data;

rev_data_t rev_data;


void connect_usart1_init(void)
{ 
	 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rev_data.rx,RECEIVE_DATA_SIZE*2);	
}

float vel_ratio=0.05f;
extern chassis_t chassis_move;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		if(rev_data.rx[0]=='X')
		{
		  chassis_move.start_flag=1;
		}
		else if(rev_data.rx[0]=='Y')
		{
		  chassis_move.start_flag=0;
		}
		
		if(chassis_move.start_flag==1)
		{
			if(rev_data.rx[0]=='Z')
			{//Õ£÷π
				chassis_move.front_flag=0;
				chassis_move.turn_flag=0;
				chassis_move.v_set=0.0f;
				chassis_move.turn_set=chassis_move.total_yaw;
			}
			else if(rev_data.rx[0]=='A')
			{
				chassis_move.front_flag=1;

				//chassis_move.v_set=2.0f;	
				chassis_move.v_set=65.0f*vel_ratio;
				chassis_move.x_set=chassis_move.x_set+chassis_move.v_set*0.02f;				
			}
			else if(rev_data.rx[0]=='E')
			{
				chassis_move.front_flag=1;
				//chassis_move.v_set=-2.0f;
			  chassis_move.v_set=-65.0f*vel_ratio;
				chassis_move.x_set=chassis_move.x_set+chassis_move.v_set*0.02f;				
			}
			else if(rev_data.rx[0]=='G')
			{
				chassis_move.turn_flag=1;
				chassis_move.turn_set=4.5f;
			}
			else if(rev_data.rx[0]=='C')
			{
				chassis_move.turn_flag=1;
				chassis_move.turn_set=-4.5f;
			}
		//	else if(rev_data.rx[0]=='F')
			//{
		//		chassis_move.turn_flag=1;
				//chassis_move.turn_set=-20.0f;
		//	}
		//	else if(rev_data.rx[0]=='D')
		//	{
		//	chassis_move.turn_flag=1;
		//	}
		}	
		HAL_UART_Receive_IT(&huart1,rev_data.rx, 1);//Ω” ’
	}
	
}

