/**--------- @tips: using UTF8---------------------------------------**/

/***********************************************************************

代码编写参考：大疆电机测试 
使用场景：    快速测试老电机能否使用、所有电机是否都正确接入CAN总线、测试主控能否正常使用CAN通信。通常在布线阶段编写，测试完之后会把代码删掉。
涉及到的文件：RCS_VESC_MOTOR.c
思路：        完成CAN1和CAN2的初始化后，调用函数控制电机开环转动，同时获取电机的转速
常见结果：    ①电机能转+能看到转速--->电机与主控均正常 
              ②电机能转+不能看到转速/电机不能转+能看到转速---->主控的CAN收发器或者电机有一个损坏
							③电机不能转+不能看到转速---->线可能接触不良，分电板可能接触不良，如果不是接触不良问题，那就是主控的CAN收发器或者电机有一个损坏
注意事项:     当CAN总线上只有大疆电机，并且6020的ID在5~7时，可以不开启CAN缓冲区而完成全部电机的控制

***********************************************************************/

/**---------- @addtogroup:Demo代码的文件依赖 ---------------**/
#include "RCS_MOTOR.h"


/**---------- @addtogroup:Demo代码所需的全局变量 ------------**/
float Demo_TestAngle_CAN1[8];
float Demo_TestAngle_CAN2[8];

/**---------- @addtogroup:Demo代码提供的编写示范 ------------**/

/**
 *@name: Demo_MotorTest_Init
 *@brief:将该函数放入MainTask_AllInit函数中的“输出部分初始化”，即可完成CAN总线上大疆电机和本杰明电调的初始化
 *@usage:不建议在新编写的代码中采用老式写法，这里放老写法只是为了方便理解一些老代码，也因为老代码方便用于测试电机
**/
void Demo_MotorTest_Init_Old()
{
	//大疆电机在老代码中被称为Motor
	//Motor_Init的含义是，完成大疆电机在CAN1的初始化，Motor_Init2则是CAN2
	Motor_Init();
	Motor_Init2();//RCS_CAN.c有bug，必须先初始化CAN1再初始化CAN2
}

/**
 *@name: Demo_RMESC_Test_Old
 *@brief:老代码测试大疆电机

 *@usage:请先将UpCtrl_Task中的whil(1)循环中的“下发机器人控制指令”里面的两个MotorList_Excute注释，然后放入本函数。
 *@usage:这么做的原因是， MotorList_Excute函数底层会发送CAN报文，这会干涉到本函数的运行

 *@tips:不建议在新编写的代码中采用老式写法，这里放老写法只是为了方便理解一些老代码，也因为老代码方便用于测试电机
 *@tips:老代码有个优点，能一下子看出往CAN总线上发了几条报文，比如本函数，有三个函数和CAN1有关，一次调用就会往CAN1上发三条报文
**/
void Demo_RMESC_Test_Old()
{
	//测试CAN1上的3508或2006
	Motor_Send(400,400,400,400);
	Motor_Send_ADD(400,400,400,400);//3508和2006使用同样的报文，地位相同，控制方式和反馈方式也是一样的，只有减速比不同
	Demo_TestAngle_CAN1[0]=Get_Motor_Float_Angle(1);
	Demo_TestAngle_CAN1[1]=Get_Motor_Float_Angle(2);
	Demo_TestAngle_CAN1[2]=Get_Motor_Float_Angle(3);
	Demo_TestAngle_CAN1[3]=Get_Motor_Float_Angle(4);
	Demo_TestAngle_CAN1[4]=Get_Motor_Float_Angle(5);
	Demo_TestAngle_CAN1[5]=Get_Motor_Float_Angle(6);
	Demo_TestAngle_CAN1[6]=Get_Motor_Float_Angle(7);
	Demo_TestAngle_CAN1[7]=Get_Motor_Float_Angle(8);
	//测试CAN1上ID为5~7的6020
	GM6020_Send_ADD(700,700,700);   //6020和3508报文不同，如果CAN总线上存在ID小于5的6020，请考虑是否会出现报文干涉，判断方式可以跳转到GM6020_Send_ADD函数中，里面的注释有写怎么看
	
	
	//测试CAN2上的3508或2006
	Motor_Send2(400,400,400,400);
	Motor_Send2_ADD(400,400,400,400);
	Demo_TestAngle_CAN2[0]=Get_Motor_Float_Angle2(1);
	Demo_TestAngle_CAN2[1]=Get_Motor_Float_Angle2(2);
	Demo_TestAngle_CAN2[2]=Get_Motor_Float_Angle2(3);
	Demo_TestAngle_CAN2[3]=Get_Motor_Float_Angle2(4);
	Demo_TestAngle_CAN2[4]=Get_Motor_Float_Angle2(5);
	Demo_TestAngle_CAN2[5]=Get_Motor_Float_Angle2(6);
	Demo_TestAngle_CAN2[6]=Get_Motor_Float_Angle2(7);
	Demo_TestAngle_CAN2[7]=Get_Motor_Float_Angle2(8);
	//测试CAN1上ID为5~7的6020
	GM6020_Send_ADD(700,700,700);
}


/***********************************************************************

代码编写参考：本杰明电调测试 
使用场景：    快速测试能否正常通过CAN总线控制本杰明电调。通常在布线阶段编写，测试完之后会把代码删掉。
涉及到的文件：RCS_VESC_MOTOR.c
思路：        完成CAN1和CAN2的初始化后，调用函数控制电机开环转动，同时获取电机的转速
常见结果：    ①电机能转+能看到转速--->电机与主控均正常 
              ②电机能转+不能看到转速/电机不能转+能看到转速---->主控的CAN收发器或者电机有一个损坏
							③电机不转+不能看到转速---->线可能接触不良，分电板可能接触不良，如果不是接触不良问题，那就是主控的CAN收发器或者电机有一个损坏
              ④电机异响+能看到转速------>本杰明电调参数没调好，或者本杰明电调与电机的连接点焊接质量差
注意事项：    往CAN总线上同时挂本杰明+大疆电调时需谨慎。出于如下两个原因：
              ①上赛季并未完全验证本杰明电调报文是否会与大疆电调干涉
              ②不同于大疆电调，一条CAN报文只能控制一个本杰明电调。也就是有可能会同时发送超过3条CAN报文，这超过了CAN外设的信箱容量。
              但如果从头到尾只用大疆电机的话，机器人处理的场景相当受限。正因如此，如果选择了大疆电机以外的电机，请提前做好充分的测试和调试。

本杰明电调相关：本杰明电调有配套的上位机，如果只是测试电调好坏、电机好坏、编码器好坏等功能的话，只用上位机即可。

***********************************************************************/

/**---------- @addtogroup:Demo代码的文件依赖 ---------------**/
#include "RCS_VESC_MOTOR.h"

/**---------- @addtogroup:Demo代码所需的全局变量 ------------**/
float Demo_TestAngle_VESC[2];

/**---------- @addtogroup:Demo代码提供的编写示范 ------------**/

/**
 *@name: Demo_MotorTest_Init_VESC
 *@brief:将该函数放入MainTask_AllInit函数中的“输出部分初始化”，即可完成CAN总线上本杰明电调的初始化
 *@usage:不建议在新编写的代码中采用老式写法，这里放老写法只是为了方便理解一些老代码，也因为老代码方便用于测试电机
**/
void Demo_MotorTest_Init_VESC(uint8_t id)
{
	//RCS_CAN.c有bug，必须先初始化CAN1再初始化CAN2
	VESC_CAN_Init(CAN1);
	VESC_CAN_Init(CAN2);
}

/**
 *@name: Demo_MotorTest_Vesc
 *@brief:老代码测试本杰明电调

 *@usage:请先将UpCtrl_Task中的whil(1)循环中的“下发机器人控制指令”里面的两个MotorList_Excute注释，然后放入本函数。
 *@usage:这么做的原因是， MotorList_Excute函数底层会发送CAN报文，这会干涉到本函数的运行

 *@tips:不建议在新编写的代码中采用老式写法，这里放老写法只是为了方便理解一些老代码，也因为老代码方便用于测试电机
 *@tips:老代码有个优点，能一下子看出往CAN总线上发了几条报文，比如本函数，有一个函数和CAN1有关，一次调用就会往CAN1上发1条报文
**/
void Demo_MotorTest_Vesc(uint16_t e_rpm,uint8_t id)
{
	//CAN1上的本杰明位置放在[0]，CAN2则放在[1]
	Demo_TestAngle_VESC[0]=Get_VESC_Pos(id);
	Demo_TestAngle_VESC[1]=Get_VESC_Pos2(id);
	//分别往CAN1和CAN2发一条控制本杰明转速的报文
	VESC_Excute_Speed(CAN1,id,e_rpm);//e_rpm是磁极转速，不是我们平常理解上的转速
	VESC_Excute_Speed(CAN2,id,e_rpm);
}


/***********************************************************************

代码编写参考：编写Upctrl函数，完成机器人上层机构的控制，并提供日志接口
使用场景：    调车前期编写，完成编写后几乎就不再改动了，半自动逻辑仅需调用Upctrl函数即可实现机器人机构的控制

注意事项：    ①请基于RCS_Motor_Upctrl完成代码，提高代码的可读性和安全性。
              ②编写时要好好斟酌下函数形参怎么定义。要能实现：半自动逻辑仅需调用Upctrl函数就可以让机器人完成所有动作。

***********************************************************************/

/**---------- @addtogroup:Demo代码的文件依赖 ---------------**/
//HAL依赖
#include "RCS_GPIO.h"
#include "RCS_PIDCtrl.h"
#include "RCS_AcceCtrl.h"
//模块驱动依赖
#include "RCS_MOTOR.h"
#include "RCS_Motor_Upctrl.h"


/**---------- @addtogroup:Demo代码所需的参数管理 ------------**/

#define Motor_BallPicker_ID 1
#define Motor_Fiction_LU_ID 3
#define Motor_Fiction_LD_ID 4
#define Motor_Fiction_RU_ID 3
#define Motor_Fiction_RD_ID 4

#define Cylinder_Loader_GPIO GPIOE
#define Cylinder_Loader_Pin  GPIO_Pin_2

/**---------- @addtogroup:Demo代码所需的全局变量 ------------**/

//使用到的电机
Motor_Ctrl_Node Motor_BallPicker;
Motor_Ctrl_Node Motor_Fiction_LU;
Motor_Ctrl_Node Motor_Fiction_LD;
Motor_Ctrl_Node Motor_Fiction_RU;
Motor_Ctrl_Node Motor_Fiction_RD;

//速度环PID
PID_Struct      SpeedPID_BallPicker;
PID_Struct      SpeedPID_Fiction_LD;
PID_Struct      SpeedPID_Fiction_LU;
PID_Struct      SpeedPID_Fiction_RD;
PID_Struct      SpeedPID_Fiction_RU;

//角度环PID
DacePID_Struct  AnglePID_BallPicker;

/**---------- @addtogroup:Demo代码提供的编写示范 ------------**/

/**
 *@name: Demo_External_Encoder_GetPos
 *@brief:编码器修正电机的回调函数

 *@usage:这个函数将会被重定向到xxxxx.Get_Node_Angle函数指针处，因此必须被定义为 float yyyyyy(uint8_t)
 *@usage:如果没用到编码器修正电机，这个函数可以不要
 
 *@tips:请将本函数放在Up_Ctrl.c中
**/
float Demo_External_Encoder_GetPos(uint8_t motor_id)
{
	switch(motor_id)
	{
		case Motor_BallPicker_ID:
			//如果这个电机有外置编码器修正角度的话，在此处完善获取编码器角度的函数
		break;
		//其他id同理
	}
}

/**
 *@name: Demo_Upctrl_Init
 *@brief:初始化上层机构的代码编写参考
 
 *@usage:上层机构有哪些东西，全丢进这个函数中初始化，并通过宏定义的方式指定某个机构对应的外设，实现参数管理
 *@tips: 请将本函数放在Up_Ctrl.c中，并在MainTask_Allinit中被调用
**/
void Demo_Upctrl_Init(void)
{
	//初始化速度环PID参数
	//Tips:虽然共用同一套PID参数，但是必须使用不同的PID变量，避免串扰
	SpeedPID_BallPicker=Get_RM3508_Speed_Pid();
	SpeedPID_Fiction_LD=Get_RM3508_Speed_Pid();
	SpeedPID_Fiction_LU=Get_RM3508_Speed_Pid();
	SpeedPID_Fiction_RD=Get_RM3508_Speed_Pid();
	SpeedPID_Fiction_RU=Get_RM3508_Speed_Pid();
	
	//初始化角度环PID参数(随便给的，请自行调参)
	//在大多数情况下，加减速死区PID比一般PID更稳定而且平滑
	AnglePID_BallPicker.dead_zone     =20.0f;//如果只需要加减速不需要死区，把dead_zone赋值0即可
	AnglePID_BallPicker.max_acce      =5.0f; //如果只需要死区而不需要加减速，把acce和dcce赋值成和Limit_Output一样即可
	AnglePID_BallPicker.max_dcce      =5.0f;
	AnglePID_BallPicker.pid.P         =5.0f;
	AnglePID_BallPicker.pid.I         =0.0f;
	AnglePID_BallPicker.pid.D         =0.0f;
	AnglePID_BallPicker.pid.Limit_Output=7000.0f;
	AnglePID_BallPicker.pid.Limit_Integral=0.0f;
	
	//初始化电机节点(必须配置)
	MotorNode_Init_C620(CAN_GROUP_2,Motor_BallPicker_ID,&Motor_BallPicker);
	MotorNode_Init_C620(CAN_GROUP_2,Motor_Fiction_LU_ID,&SpeedPID_Fiction_LD);
	MotorNode_Init_C620(CAN_GROUP_2,Motor_Fiction_LD_ID,&SpeedPID_Fiction_LU);
	MotorNode_Init_C620(CAN_GROUP_2,Motor_Fiction_RU_ID,&SpeedPID_Fiction_RD);
	MotorNode_Init_C620(CAN_GROUP_2,Motor_Fiction_RD_ID,&SpeedPID_Fiction_RU);
	
	//配置电机节点的闭环控制参数(根据需要配置)
	//tips:编译器不会检查指针传递出错的问题，所以如果电机不能动的话，有可能是参数传错了
	MotorNode_Add_SpeedPid(&Motor_BallPicker,&SpeedPID_BallPicker);
	MotorNode_Add_DaceAnglePid(&Motor_BallPicker,&AnglePID_BallPicker);
	
	MotorNode_Add_SpeedPid(&Motor_Fiction_LD,&SpeedPID_Fiction_LD);
	MotorNode_Add_SpeedPid(&Motor_Fiction_LU,&SpeedPID_Fiction_LU);
	MotorNode_Add_SpeedPid(&Motor_Fiction_RD,&SpeedPID_Fiction_RD);
	MotorNode_Add_SpeedPid(&Motor_Fiction_RU,&SpeedPID_Fiction_RU);
	
	//重定向电机节点的参考函数(根据需要配置，比如电机需要大角度移动的时候，大多时候不需要。必须晚于电机节点的初始化)
	//Get_Node_Angle重定向后，如果调用控制函数比如MotorNode_Update_Angle，将会自动使用重定向后的函数作为参考目标进行闭环控制
	//Motor_BallPicker.Get_Node_Angle=Demo_External_Encoder_GetPos;
	
	//将节点加入电机链表(必须配置，而且每个电机节点只能被单次加入电机链表)
	MotorNode_Add(CAN_GROUP_2,&Motor_BallPicker);
	MotorNode_Add(CAN_GROUP_2,&SpeedPID_Fiction_LD);
	MotorNode_Add(CAN_GROUP_2,&SpeedPID_Fiction_LU);
	MotorNode_Add(CAN_GROUP_2,&SpeedPID_Fiction_RD);
	MotorNode_Add(CAN_GROUP_2,&SpeedPID_Fiction_RU);
	
	//初始化CAN2
	Motor_Init2();
	
	//初始化CAN之外的外设
	RCS_GPIO_Output_Init(Cylinder_Loader_GPIO,Cylinder_Loader_Pin);
	
}

/**
 *@name: Demo_Upctrl
 *@brief:上层机构完全控制函数
 
 *@usage:提供完整的上层机构控制接口，用于编写状态机，避免在出现电机控制不连续的问题
 *@tips: 请将本函数放在Up_Ctrl.c中，并在UpCtrlTask中的”更新机器人控制指令“中被调用
**/
void Demo_Upctrl(float Picker_Pos,float shoter_lu,float shoter_ld,float shoter_ru,float shoter_rd,uint8_t is_shot)
{
	MotorNode_Update_AngleFull(Picker_Pos,&Motor_BallPicker);//内环普通PID，外环加减速PID
	MotorNode_Update_Spd(shoter_lu,&Motor_Fiction_LU);
	MotorNode_Update_Spd(shoter_ld,&Motor_Fiction_LD);
	MotorNode_Update_Spd(-1.0f*shoter_ru,&Motor_Fiction_LU);//摩擦轮正反方向不应该是自动逻辑需要考虑的，在这里进行了处理
	MotorNode_Update_Spd(-1.0f*shoter_rd,&Motor_Fiction_LD);
	
	if (is_shot) RCS_GPIO_Set(Cylinder_Loader_GPIO,Cylinder_Loader_Pin);
	else         RCS_GPIO_Reset(Cylinder_Loader_GPIO,Cylinder_Loader_Pin);
}

/**
 *@name: Demo_UpCtrl_Reset
 *@brief:上层机构重试复位
 *@tips: 请将本函数放在Up_Ctrl.c中，并在UpCtrlTask中的”更新机器人控制指令“中被调用
**/
uint8_t Demo_UpCtrl_Reset(void)
{
	//在此函数内实现机器人重试时的机构复位，完成复位后返回1，否则返回0
}

/**
 *@name: Demo_Upctrl
 *@brief:上层机构完全控制函数
 
 *@usage:提供完整的上层机构控制接口，用于编写状态机，避免在出现电机控制不连续的问题
 *@tips: 请将本函数放在Up_Ctrl.c中，并在BleTask中的”输出日志“中被调用，用于调试
**/
void Demo_Upctrl_LogPrintf(void)
{
	//可以使用MotorNode_Get_Speed等函数获取电机节点当前的速度、角度等
	//可以使用Motor_BallPicker.Out_Spd访问到这个电机节点当前的目标速度，也即角度环PID的输出，用于角度环PID调参
	//可以用Motor_BallPicker.Motor_Speed_PID->Integral访问到速度环PID的积分值，用于角度环PID调参分析震荡原因
	//可以用Motor_BallPicker.Motor_Speed_PID->Last_Error访问到这个时刻的速度环误差值，用于角度环PID调参分析震荡原因
	//如果角度环使用的是加减速PID，可以用Motor_BallPicker.Motor_DaceAngle_PID->pid.Last_Error访问到这个时刻角度环的误差值
	//如果角度环使用的是加减速PID，可以用Motor_BallPicker.Motor_DaceAngle_PID->pid.Last_Error访问到这个时刻角度环的误差值
	//如果角度环使用的是加减速PID，可以用Motor_BallPicker.Motor_DaceAngle_PID->pid.Integral访问到角度环当前的总积分误差
	//如果角度环使用的是一般PID，把DaceAngle_PID换成Motor_Angle_PID即可
}


/***********************************************************************

代码编写参考：编写Chctrl函数，完成机器人底盘的速度、位置控制、单段跑点、伪曲线跑点，并提供日志接口。这里以异形全向轮为例
使用场景：    调车前期编写，完成编写后几乎就不再改动了，半自动逻辑仅需调用Chctrl函数即可实现机器人底盘的控制

注意事项：    ①不推荐使用老代码。老代码仅供Ctrl+CV。请基于RCS_Motor_Upctrl完成代码，提高代码的可读性和安全性。

***********************************************************************/
/**---------- @addtogroup:Demo代码的文件依赖 ---------------**/
//HAL依赖
#include "RCS_GPIO.h"
#include "RCS_PIDCtrl.h"
#include "RCS_AcceCtrl.h"
//模块驱动依赖
#include "RCS_MOTOR.h"
#include "RCS_Motor_Upctrl.h"


/**---------- @addtogroup:Demo代码所需的参数管理 ------------**/

//底盘-电机ID
#define Motor_Chassis_LU_ID 3
#define Motor_Chassis_LD_ID 4
#define Motor_Chassis_RU_ID 3
#define Motor_Chassis_RD_ID 4

//底盘-机械参数，请根据情况自行define


//赛场-蓝区的x轴和z轴取反后可以得到和红区一样的坐标
#define ROBOCON_ZONE        -1.0f

//赛场-重试区的红区相对坐标
#define ROBOCON_RESET_RED_X  0.0f
#define ROBOCON_RESET_RED_Y  5000.0f
#define ROBOCON_RESET_RED_Z  90.0f

/**---------- @addtogroup:Demo代码所需的全局变量 ------------**/

//使用到的电机
Motor_Ctrl_Node Motor_Chassis_LU;
Motor_Ctrl_Node Motor_Chassis_LD;
Motor_Ctrl_Node Motor_Chassis_RU;
Motor_Ctrl_Node Motor_Chassis_RD;

//速度环PID
PID_Struct      SpeedPID_Chassis_LD;
PID_Struct      SpeedPID_Chassis_LU;
PID_Struct      SpeedPID_Chassis_RD;
PID_Struct      SpeedPID_Chassis_RU;

//底盘坐标环PID
DacePID_Struct  PosPID_Chassis_X;
DacePID_Struct  PosPID_Chassis_Y;
DacePID_Struct  PosPID_Chassis_Z;

//底盘坐标环PID的输出，也即底盘输出速度，用于调试
volatile float demo_chassis_x_output;
volatile float demo_chassis_y_output;
volatile float demo_chassis_z_output;

/**---------- @addtogroup:Demo代码提供的编写示范 ------------**/

void Demo_ChCtrl_Init(void)
{
	//电机节点部分的代码和upctrl一个套路，不再赘述
	//底盘跑点参数请在这赋一个初始值
}

uint8_t Demo_ChCtrl_Reset(void)
{
	//在此函数内实现机器人重试时的坐标重置、机构复位，完成复位后返回1，否则返回0
}

float Demo_ChCtrl_HAL_Get_X(void)
{
	//在此函数内实现红蓝半区的坐标差异控制，同时也可以隔离底盘定位方式的差异，比如用激光雷达定位代替码盘，就只需要改这个函数的定义
}

float Demo_ChCtrl_HAL_Get_Y(void)
{
	//在此函数内实现红蓝半区的坐标差异控制，同时也可以隔离底盘定位方式的差异，比如用激光雷达定位代替码盘，就只需要改这个函数的定义
}

float Demo_ChCtrl_HAL_Get_Z(void)
{
	//在此函数内实现红蓝半区的坐标差异控制，同时也可以隔离底盘定位方式的差异，比如用激光雷达定位代替码盘，就只需要改这个函数的定义
}

void Demo_ChCtrl_SpdCtrl(float spd_x,float spd_y,float spd_z)
{
	//在此处填入底盘逆运动学解算，并通过MotorNode_Update_Spd将解算结果下发给轮电机
}

void Demo_ChCtrl_PosCtrl(float pos_x,float pos_y,float pos_z)
{
	demo_chassis_x_output=DacePID_Normal_Ctrl(pos_x,Demo_ChCtrl_HAL_Get_X(),&PosPID_Chassis_X);
	demo_chassis_y_output=DacePID_Normal_Ctrl(pos_y,Demo_ChCtrl_HAL_Get_Y(),&PosPID_Chassis_Y);
	demo_chassis_z_output=DacePID_Normal_Ctrl(pos_z,Demo_ChCtrl_HAL_Get_Z(),&PosPID_Chassis_Z);
	Demo_ChCtrl_SpdCtrl(demo_chassis_x_output,demo_chassis_y_output,demo_chassis_z_output);
}

int16_t Demo_ChCtrl_Run2Point(uint16_t route)
{
	switch(route)
	{
		case 0:
			//跑点参数重新配置
			PosPID_Chassis_X.dead_zone=50.0f;
			//跑点位置重新配置
			Demo_ChCtrl_PosCtrl(0.0f,100.0f,90.0f);
			//什么时候返回什么，记得从近到远，避免if语句短路
			if((Demo_ChCtrl_HAL_Get_Y()-100.0f)<=1.0f) return 1;
			if((Demo_ChCtrl_HAL_Get_Y()-100.0f)<=5.0f) return 2;
			if((Demo_ChCtrl_HAL_Get_Y()-100.0f)<=50.0f) return 3;//可以假定数字越大越远，看个人习惯了
		  //未抵达，返回0
			return 0;
		break;
		
		case 1:
			//需要跑几段点，就写几段case，格式如上所示，为每一段跑点指定各自不同的角度环PID和目标点
		break;
	}
}

void Demo_Chctrl_LogPrintf(void)
{
	//常见的需要被打印出来的参数：底盘当前xyz坐标、现在在跑哪一段点
	//每年需要打印的参数各异，但总的来说，参数是用来辅助分析问题的，如果参数没有选好，很难去发现问题的关键所在
}

/***********************************************************************

代码编写参考：编写控制状态机，完成全车的功能
使用场景：    为完整功能编写，根据调车的结果不断修改，直到比赛。这里以上面两个样例代码为例，实现一段曲线跑点后射球，但未实现机器人重试和红蓝半区的控制
注意事项：    ①状态机有明确和完整的规范，按照这个八股文思考和编写代码，能实现机器人比赛的部分功能
              ②作为八股文，状态机编写只是基础中的基础。需要思考的问题关键点应该是：逻辑怎么写，车跑更快？怎么写半自动让操作手更方便？为什么老是射不准，怎么设计实验流程找到射不准的原因？视觉数据的抖动和丢失怎么处理比较好？等等等

***********************************************************************/



/***********************************************************************

代码编写提示：使用OLED屏幕实时展示变量值
使用场景：    串口不够，没法通过蓝牙将日志传给电脑时，使用OLED屏幕
编写方法：    sprintf到字符数组中，然后分行显示数组即可

注意事项：    ①OLED屏的驱动使用的是软件IIC，会阻塞代码运行，请放在BleTask中运行！
              ②OLED屏幕需要刷屏，否则残留数字可能会让人误解

***********************************************************************/


