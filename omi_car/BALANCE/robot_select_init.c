#include "robot_select_init.h"

//Initialize the robot parameter structure
//初始化机器人参数结构体
Robot_Parament_InitTypeDef  Robot_Parament; 
/**************************************************************************
Function: According to the potentiometer switch needs to control the car type
Input   : none
Output  : none
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models, CAR_NUMBER=6
  //ADC值分段变量，取决于小车型号数量，目前有6种小车型号，CAR_NUMBER=6
	Divisor_Mode=4096/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(CAR_MODE_ADC,10))/Divisor_Mode); //Collect the pin information of potentiometer //采集电位器引脚信息	
  if(Car_Mode>10)Car_Mode=10;
	#if Mec
	{	
		if (Car_Mode==0)  Robot_Init(SENIOR_MEC_BS_wheelspacing, SENIOR_MEC_BS_axlespacing, MD36N_27, Photoelectric_500, Mecanum_100); //SENIOR_MEC_BS - 高配麦轮摆式悬挂 //BS: Pendulum suspension
		if (Car_Mode==1)  Robot_Init(SENIOR_MEC_BS_wheelspacing, SENIOR_MEC_BS_axlespacing, MD36N_27, Photoelectric_500, Mecanum_100); //SENIOR_MEC_BS - 高配麦轮摆式悬挂  
		
		if (Car_Mode==2)  Robot_Init(SENIOR_MEC_DL_wheelspacing, SENIOR_MEC_DL_axlespacing, MD36N_27, Photoelectric_500, Mecanum_152); //SENIOR_MEC_DL - 高配麦轮独立悬挂 //DL: Independent suspension 
		if (Car_Mode==3)  Robot_Init(TOP_MEC_BS_wheelspacing,    TOP_MEC_BS_axlespacing,    MD60N_18, Photoelectric_500, Mecanum_152); //TOP_MEC_BS_18 - 顶配麦轮摆式悬挂常规型
		
		if (Car_Mode==4)  Robot_Init(TOP_MEC_BS_wheelspacing,    TOP_MEC_BS_axlespacing,    MD60N_47, Photoelectric_500, Mecanum_152); //TOP_MEC_BS_47 - 顶配麦轮摆式悬挂重载型
		
		if (Car_Mode==5)  Robot_Init(TOP_MEC_DL_wheelspacing,    TOP_MEC_DL_axlespacing,    MD60N_18, Photoelectric_500, Mecanum_152); //TOP_MEC_DL_18 - 顶配麦轮独立悬挂常规型
		if (Car_Mode==6)  Robot_Init(TOP_MEC_DL_wheelspacing,    TOP_MEC_DL_axlespacing,    MD60N_47, Photoelectric_500, Mecanum_152); //TOP_MEC_DL_18 - 顶配麦轮独立悬挂重载型
		
		if (Car_Mode==7)  Robot_Init(FLAGSHIP_MEC_BS_wheelspacing,    FLAGSHIP_MEC_BS_axlespacing,    MD60N_18, Photoelectric_500, Mecanum_152); //FlagShip_MEC_DL_18 - 旗舰麦轮摆式悬挂常规型
		if (Car_Mode==8)  Robot_Init(FLAGSHIP_MEC_BS_wheelspacing,    FLAGSHIP_MEC_BS_axlespacing,    MD60N_47, Photoelectric_500, Mecanum_152); //FlagShip_MEC_DL_47 - 旗舰麦轮摆式悬挂重载型
		
		if (Car_Mode==9)   Robot_Init(FLAGSHIP_MEC_DL_wheelspacing,FLAGSHIP_MEC_DL_axlespacing,MD60N_18, Photoelectric_500, Mecanum_152); //flagship_MEC_dl_18 - 旗舰麦轮独立悬挂常规型
		if (Car_Mode==10)  Robot_Init(FLAGSHIP_MEC_DL_wheelspacing,FLAGSHIP_MEC_DL_axlespacing,MD60N_47, Photoelectric_500, Mecanum_152); //flagship_MEC_dl_47 - 旗舰麦轮独立悬挂重载型
	}
	#elif Omni
	{
		if (Car_Mode==0)  Robot_Init(Omni_Turn_Radiaus_164, MD36N_5_18, Photoelectric_500, FullDirecion_75); //SENIOR_OMNI_5_18 - 高配全向轮三角形极速型
		if (Car_Mode==1)  Robot_Init(Omni_Turn_Radiaus_180, MD36N_27,   Photoelectric_500, FullDirecion_127);//SENIOR_OMNI_27   - 高配全向轮三角形常规     
		if (Car_Mode==2)  Robot_Init(Omni_Turn_Radiaus_180, MD36N_27,   Photoelectric_500, FullDirecion_127);//SENIOR_OMNI_27   - 高配全向轮圆形常规       
		if (Car_Mode==3)  Robot_Init(Omni_Turn_Radiaus_180, MD36N_51,   Photoelectric_500, FullDirecion_127);//SENIOR_OMNI_51   - 高配全向轮圆形重载       

		if (Car_Mode==4)  Robot_Init(Omni_Turn_Radiaus_290, MD60N_18,   Photoelectric_500, FullDirecion_127);//TOP_OMNI_18      - 顶配全向轮常规 直径127   
		if (Car_Mode==5)  Robot_Init(Omni_Turn_Radiaus_290, MD60N_18,   Photoelectric_500, FullDirecion_152);//TOP_OMNI_18      - 顶配全向轮常规 直径152
		if (Car_Mode==6)  Robot_Init(Omni_Turn_Radiaus_290, MD60N_18,   Photoelectric_500, FullDirecion_203);//TOP_OMNI_18      - 顶配全向轮常规 直径203
	}
	#endif
}


#if Mec
/**************************************************************************
Function: Initialize cart parameters
Input   : wheelspacing, axlespacing, omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(float wheelspacing,float axlespacing,int gearratio,int Accuracy,float tyre_diameter) 
{
  Robot_Parament.WheelSpacing=wheelspacing;   //Wheelspacing 轮距  
	Robot_Parament.AxleSpacing=axlespacing;     //Axlespacing 轴距
  Robot_Parament.GearRatio=gearratio;         //motor_gear_ratio //电机减速比
  Robot_Parament.EncoderAccuracy=Accuracy;    //Number_of_encoder_lines //编码器精度(编码器线数)
  Robot_Parament.WheelDiameter=tyre_diameter; //Diameter of driving wheel //主动轮轮径
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI; 
	//Wheelspacing 轮距 
  Wheel_spacing=Robot_Parament.WheelSpacing;  
 //Wheel_axlespacing (Wheel_axlespacing is not required for motion analysis of differential trolleys) //轴距(差速小车的运动分析不需要轴距)	
	Wheel_axlespacing=Robot_Parament.AxleSpacing;    
}

#elif Omni
/**************************************************************************
Function: Initialize cart parameters
Input   : omni_rotation_radiaus, motor_gear_ratio, Number_of_encoder_lines, tyre_diameter
Output  : none
函数功能：初始化小车参数
入口参数：自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(float omni_turn_radiaus,int gearratio,int Accuracy,float tyre_diameter)
{
	//Rotation radius of omnidirectional trolley
  //全向轮小车旋转半径	
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus;
	//motor_gear_ratio
	//电机减速比
  Robot_Parament.GearRatio=gearratio;
	//Number_of_encoder_lines
  //编码器精度(编码器线数)
  Robot_Parament.EncoderAccuracy=Accuracy;
	//Diameter of driving wheel
  //主动轮直径
  Robot_Parament.WheelDiameter=tyre_diameter;
	
	//Encoder value corresponding to 1 turn of motor (wheel)
	//电机(车轮)转1圈对应的编码器数值
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;
	//Driving wheel circumference
  //主动轮周长	
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;
	//Rotation radius of omnidirectional trolley
  //全向轮小车旋转半径	
  Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus;
}

#endif

