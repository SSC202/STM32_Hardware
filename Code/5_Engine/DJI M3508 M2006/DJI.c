/*包含DJI的初始化，DJI的Transmit，DJI的解码算法*/
#include "DJI.h"

DJI_t hDJI[8];

/**
  * @brief DJI电机的初始化设置
  * @param 无
  * @retval 使用DJI init前需要指定motorType
  */
void DJI_Init()
{
	for(int i = 0; i < 8; i++)
    {
		hDJI[i].speedPID.KP = 12;                 //（速度伺服）比例增益设置
		hDJI[i].speedPID.KI = 0.2;                //（速度伺服）积分增益设置
		hDJI[i].speedPID.KD = 5;                  //（速度伺服）微分增益设置
		hDJI[i].speedPID.outputMax = 8000;        //（速度伺服）最大输出设置

      	hDJI[i].posPID.KP = 80.0f;                //（位置伺服）比例增益设置
		hDJI[i].posPID.KI = 1.0f;                 //（位置伺服）积分增益设置
		hDJI[i].posPID.KD = 0.0f;                 //（位置伺服）微分增益设置
		hDJI[i].posPID.outputMax = 5000;          //（位置伺服）最大输出设置
//		hDJI[i].posPID.outputMin = 1500;          //（位置伺服）最小输出设置
			
/**
  * @retval 2006减速比为36 3508减速比约为19
  */
		if(hDJI[i].motorType == M3508 )
		{
			hDJI[i].reductionRate = 3591.0f/187.0f;
		}
		else if(hDJI[i].motorType == M2006)
		{
			hDJI[i].reductionRate = 36.0f;
		}
      	hDJI[i].encoder_resolution = 8192.0f;     //编码器分辨率设置
    }
}

static uint32_t TxMailbox;
/**
  *@brief 电调0/1/2/3 CAN数据（标准ID数据帧）输出
  *@param hcanx   Can句柄
  *@param cmlx_iq 电调ID
  */
void CanTransmit_DJI_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq){
	CAN_TxHeaderTypeDef TxMessage;
		
	TxMessage.DLC=0x08;                         //DLC数据段长度设置
	TxMessage.StdId=0x200;                      //标准CAN ID设置
	TxMessage.IDE=CAN_ID_STD;                   //标准标识符设置
	TxMessage.RTR=CAN_RTR_DATA;                 //数据帧设置

//发送报文设置
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(cm1_iq >> 8);
	TxData[1] = (uint8_t)cm1_iq;
	TxData[2] = (uint8_t)(cm2_iq >> 8);
	TxData[3] = (uint8_t)cm2_iq;
	TxData[4] = (uint8_t)(cm3_iq >> 8);
	TxData[5] = (uint8_t)cm3_iq;
	TxData[6] = (uint8_t)(cm4_iq >> 8);
	TxData[7] = (uint8_t)cm4_iq; 
//寻找空的邮箱并发送
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0) ;
	
//如果CAN信息发送失败则进入死循环
	if(HAL_CAN_AddTxMessage(hcanx,&TxMessage,TxData,&TxMailbox)!=HAL_OK)
	{
		 Error_Handler();                                                    
	}
}
/**
  *@brief 电调4/5/6/7 CAN数据（标准ID数据帧）输出
  *@param hcanx：can句柄
  *@param cmlx_iq 电调ID
  */
void CanTransmit_DJI_5678(CAN_HandleTypeDef *hcanx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq)
{
	CAN_TxHeaderTypeDef TxMessage;
		
	TxMessage.DLC=0x08;                         //DLC数据段长度设置
	TxMessage.StdId=0x1FF;                      //标准CAN ID设置
	TxMessage.IDE=CAN_ID_STD;                   //标准标识符设置
	TxMessage.RTR=CAN_RTR_DATA;                 //数据帧设置

//发送报文设置
	uint8_t TxData[8];
	TxData[0] = (uint8_t)(cm5_iq >> 8);
	TxData[1] = (uint8_t)cm5_iq;
	TxData[2] = (uint8_t)(cm6_iq >> 8);
	TxData[3] = (uint8_t)cm6_iq;
	TxData[4] = (uint8_t)(cm7_iq >> 8);
	TxData[5] = (uint8_t)cm7_iq;
	TxData[6] = (uint8_t)(cm8_iq >> 8);
	TxData[7] = (uint8_t)cm8_iq; 

//寻找空的邮箱并发送
	while(HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0) ;
//如果CAN信息发送失败则进入死循环
	if(HAL_CAN_AddTxMessage(hcanx,&TxMessage,TxData,&TxMailbox)!=HAL_OK)
	{
		 Error_Handler();       
	}
}

/**
  *@brief DJI电机数据计算
  *@param motor
  *@param cmlx_iq 电调ID
  */
void DJI_Update(DJI_t *motor, uint8_t* fdbData)
{
/************  反馈信息计算  ***********/
	motor->FdbData.RotorAngle_0_360              =   (fdbData[0]<<8|fdbData[1])*360.0f/motor->encoder_resolution ;     /* unit:degree*/
	motor->FdbData.rpm                           =   (int16_t)(fdbData[2]<<8|fdbData[3]);                                   /* unit:rom   */
	motor->FdbData.current                       =   (int16_t)(fdbData[4]<<8|fdbData[5]);   
/************  计算数据处理  ***********/
	
/************  更新反馈速度/位置  ******/
	motor->Calculate.RotorAngle_0_360_Log[LAST]  =   motor->Calculate.RotorAngle_0_360_Log[NOW];
	motor->Calculate.RotorAngle_0_360_Log[NOW]   =   motor->FdbData.RotorAngle_0_360;
	
/************ 电机圈数更新 *************/
	if(motor->Calculate.RotorAngle_0_360_Log[NOW] -  motor->Calculate.RotorAngle_0_360_Log[LAST] > (180.0f))
		motor->Calculate.RotorRound--;
	else if(motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_Log[LAST] < -(180.0f))
		motor->Calculate.RotorRound++;
	
/************ 电机输出轴角度 ***********/
	motor->AxisData.AxisAngle_inDegree  =  motor->Calculate.RotorRound * 360.0f ;
	motor->AxisData.AxisAngle_inDegree  += motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_OffSet;
	motor->AxisData.AxisAngle_inDegree  /= motor->reductionRate; 

	motor->AxisData.AxisVelocity        =  motor->FdbData.rpm / motor->reductionRate;
	motor->Calculate.RotorAngle_all		  =  motor->Calculate.RotorRound * 360 + motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_OffSet;
}
/**
  *@brief DJI
  */
void get_dji_offset(DJI_t *motor, uint8_t* fdbData)
{
	motor->FdbData.RotorAngle_0_360             = (fdbData[0]<<8|fdbData[1])*360.0f/motor->encoder_resolution;
	motor->Calculate.RotorAngle_0_360_Log[LAST] = motor->FdbData.RotorAngle_0_360; 
	motor->Calculate.RotorAngle_0_360_Log[NOW]  = motor->Calculate.RotorAngle_0_360_Log[LAST];
	motor->Calculate.RotorAngle_0_360_OffSet    = motor->FdbData.RotorAngle_0_360;
}


/**
  *@brief DJI返回报文解码
  *@param Stdid:数据帧
  */
HAL_StatusTypeDef DJI_CanMsgDecode(uint32_t Stdid, uint8_t* fdbData){
	int i=Stdid - 0x201;
	if(i>=0 &&i<8){
		if(hDJI[i].FdbData.msg_cnt<50)
		{
			get_dji_offset(&hDJI[i], fdbData);
			hDJI[i].FdbData.msg_cnt++;
		}
		else
		{
			DJI_Update(&hDJI[i], fdbData);
		}
		return HAL_OK;
	} 
	return HAL_ERROR;
}
