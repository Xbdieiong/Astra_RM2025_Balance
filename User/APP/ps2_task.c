/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     �������Ƕ�ȡ������ps2�ֱ�������ң�����ݣ�
	*            ��ң������ת��Ϊ�������ٶȡ�������ת�ǡ��������ȳ���
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "ps2_task.h"
#include "cmsis_os.h"
#include "remote_control.h"

//��������������10ms
#define PS2_TIME 10.0f  //ע�ⵥ���ȸ������ľ�������

//��ʼ�ȳ�
#define BEGIN_LEG_LENGTH 0.14f    //0.14  
#define INITIAL_LEG_LENGTH 0.18f   

//�ȳ����� �Ȳ�����������
#define RC_TO_ADD_LEG 0.00005f   
//ת������ ת��������
#define RC_TO_TURN_RATIO  0.00006f

//����ٶ�
#define VX_MAX  0.6f
#define RC_TO_VX   VX_MAX/660.0f
//�ٶ�б�£��ٶȿ���������
#define SPEED_STEP 0.2f

extern chassis_t chassis_move;
extern INS_t INS;
extern RC_ctrl_t rc_ctrl;
extern vmc_leg_t right;			
extern vmc_leg_t left;	
float leg_add = 0;
int leg_set_flag = 0;
int last_start_flag = 0;
int Left_Remote_rs_flag = 0;

//������ʼ������
void PS2_init(void){
	 chassis_move.start_flag = 0;
	 chassis_move.x_set = chassis_move.x_filter;
	 chassis_move.v_set = 0.0f;
	 chassis_move.turn_set = chassis_move.total_yaw;
	 chassis_move.leg_set = BEGIN_LEG_LENGTH;
}	
	
void pstwo_task(void)
{	
	//��ʼ��	
	PS2_init();

   while(1)
	 {	 
		   PS2_data_process(&rc_ctrl,&chassis_move,(PS2_TIME/1000.0f));//�������ݣ�������������
	     osDelay(PS2_TIME);
	 }
}

void PS2_data_process(RC_ctrl_t *rc_ctrl,chassis_t *chassis,float dt)
{   
//	if(rc_ctrl.rc.s[0] == 2 &&chassis->start_flag==0) 
//	{
//		//�ֱ��ϵ�Start����������
//		chassis->start_flag=1;
//		if(chassis->recover_flag==0
//			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
//		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
//		{
//		  chassis->recover_flag=1;//��Ҫ����
//		}
//	}
//	else if(rc_ctrl.rc.s[0] == 2&&data->key==4&&chassis->start_flag==1) 
//	{
//		//�ֱ��ϵ�Start����������
//		chassis->start_flag=0;
//		chassis->recover_flag=0;
//	}
//	
//	data->last_key=data->key;
//  
//	if(chassis->start_flag==1)
//	{//����
////		chassis->v_set=((float)(data->ry-128))*(-0.004f);//��ǰ����0
////		chassis->x_set=chassis->x_set+chassis->v_set*dt;
////		chassis->turn_set=chassis->turn_set+(data->rx-127)*(-0.00025f);//���Ҵ���0
//	  			
//		//�ȳ��仯
//		chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000015f); 
//		
//		mySaturate(&chassis->leg_set,0.15f,0.28f);//�ȳ��޷���0.065m��0.18m֮��
//				
//		if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.0001f)
//		{//ң���������ȳ��ڱ仯
//			right.leg_flag=1;	//Ϊ1��־���ȳ�����������(����������Ӧ����)�����������־���Բ�������ؼ�⣬��Ϊ���ȳ�����������ʱ����ؼ������ж�Ϊ�����
//      left.leg_flag=1;	 			
//		}
//		chassis->last_leg_set=chassis->leg_set;
//	} 
//	else if(chassis->start_flag==0)
//	{//�ر�
   leg_add += rc_ctrl->rc.ch[4] * RC_TO_ADD_LEG;
   
//�����ڲ����ȳ�����
		if(switch_is_mid(rc_ctrl->rc.s[0]))
		{
			 chassis->start_flag = 1;
		}
		else if(switch_is_down(rc_ctrl->rc.s[0]))
		{
			 chassis->start_flag = 0;
			 chassis->turn_set = chassis->total_yaw;
			 chassis->x_set = chassis->x_filter;
		}
		 if(last_start_flag == 0 && chassis->start_flag == 1){
             Left_Remote_rs_flag = 1;
		 }
  
		
    if(chassis->start_flag == 1)
    {
			 chassis_move.leg_set =  BEGIN_LEG_LENGTH;
			 if(switch_is_mid(rc_ctrl->rc.s[1])){
				if(Left_Remote_rs_flag == 1){  //��ֹδ����״̬�£���ң�����ϵ�RS����������λ����Ȳ����ù���
				   chassis_move.leg_set = BEGIN_LEG_LENGTH;
				}
				 else{
				 chassis_move.leg_set = INITIAL_LEG_LENGTH + leg_add;
				 leg_add = 0.0f;
				 }
			 }
			 else if(switch_is_down(rc_ctrl->rc.s[1])){
				Left_Remote_rs_flag = 0;
			 }

			chassis->turn_set = chassis->turn_set - rc_ctrl->rc.ch[0] * RC_TO_TURN_RATIO;
			 //�ٶ�б�¿���
	        float vx_speed_cmd = rc_ctrl->rc.ch[1] * RC_TO_VX;
            if(fabs(vx_speed_cmd-chassis->v_set) < 0.01f){
                chassis->v_set = vx_speed_cmd;
			}
			else{	
                chassis->v_set = (vx_speed_cmd > chassis->v_set) ? (chassis->v_set + 0.01f) : (chassis->v_set - 0.01f);
			}
			if(fabs(vx_speed_cmd) > 0.1f ){
				chassis->x_set = chassis->x_filter;
			}  
  	}

    //�ٶȲ�������
	// float vx_cmd = chassis->v_set - chassis->v_filter;
	float legLength = (left.L0+right.L0)/2.0f;
	// float speed_step = -(legLength - 0.15f) * 0.36f + SPEED_STEP;
	// if(fabs(vx_cmd) > speed_step){
	// 	chassis->v_set = (vx_cmd > 0) ? (chassis->v_filter + speed_step) :(chassis->v_filter - speed_step);
	// }
	// chassis->x_set = chassis->x_set + chassis->v_set * dt; 
	// if(rc_ctrl->rc.ch[1] > 1 || rc_ctrl->rc.ch[1] < -1){
	// 	chassis->x_set = chassis->x_filter;
	// }
	
	 //�ȳ������޷�
	float leg_cmd = chassis_move.leg_set - legLength;
	if(fabs(leg_cmd) > 0.1f){
	    chassis_move.leg_set = (leg_cmd > 0) ? (legLength + 0.1f) : (legLength - 0.1f); 
	}
	//λ���޷�
	float x_cmd = chassis->x_set - chassis->x_filter ;
	if(fabs(x_cmd) > 0.05f){  //0.04
		chassis->x_set = (x_cmd > 0) ? (chassis->x_filter + 0.05f) : (chassis->x_filter - 0.05f);
	}

	//ת���޷�
	if(fabsf(chassis->turn_set - chassis->total_yaw) > 0.3f){
		chassis->turn_set = ((chassis->turn_set - chassis->total_yaw) > 0) ? (chassis->total_yaw + 0.3f) : (chassis->total_yaw - 0.3f);
	}

	mySaturate(&chassis_move.leg_set,0.14,0.35);//�����ȳ���Χ
	mySaturate(&chassis->v_set,-VX_MAX,VX_MAX);
//		if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.03f)//Ϊ�ﵽ�Ϻõ�����Ч�����ȳ���̬���Ƚϴ󣬻���ά����0.03f֮��
//		{//ң���������ȳ��ڱ仯
//			right.leg_flag=1;	//Ϊ1��־���ȳ�����������(����������Ӧ����)�����������־���Բ�������ؼ�⣬��Ϊ���ȳ�����������ʱ����ؼ������ж�Ϊ�����
//      left.leg_flag=1;	 			
//		}
//		 chassis->last_leg_set=chassis->leg_set; 
   last_start_flag = chassis->start_flag ;
}






