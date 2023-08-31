/*过滤器配置，can的fifo0接收*/

#include "wtr_can.h"

uint8_t CanReceiveData[8];                                          //定义8位CAN接受数据

/**
  * @brief  CAN过滤器结构体定义
  * @retval 无
  */
HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef* hcan){           
  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterBank = 0;                                     //使用过滤器编号
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;                 //模式设置为掩码模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;                //位宽设置为32位
  sFilterConfig.FilterIdHigh = 0x0000;                              //标准ID高位
  sFilterConfig.FilterIdLow = 0x0000;                               //标准ID低位
  sFilterConfig.FilterMaskIdHigh = 0x0000;                          //扩展ID高位
  sFilterConfig.FilterMaskIdLow = 0x0000;                           //扩展ID低位
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;                //将报文放置在FIFO0中
  sFilterConfig.FilterActivation = ENABLE;                          //过滤器使能
  sFilterConfig.SlaveStartFilterBank = 14;                //选择从属CAN的启动过滤器组。对于单个CAN通道，此参数没有意义。
	                                                        //对于双CAN通道，所有编号较低的滤波器组都分配给主控制器，而索引较大的所有滤波器组都分配给从属CAN。
  
	
  if(HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)          //设置接收过滤器失败返回错误
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(hcan) != HAL_OK)                                //CAN启动失败返回错误
  {
    Error_Handler();
  }
	
  if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //使能中断启动失败返回错误
  {
    Error_Handler();                                                       
  }

	return HAL_OK;
}

void CanDataDecode(CAN_RxHeaderTypeDef RxHeader){
  /* Can message Decode */
  if( RxHeader.IDE == CAN_ID_STD ){
    DJI_CanMsgDecode (RxHeader.StdId, CanReceiveData);
  }
  if( RxHeader.IDE == CAN_ID_EXT ){
// vesc反馈关掉这里就不会有消息
    ;
  }
  
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){      //FIFO0接收中断回调函数
    CAN_RxHeaderTypeDef   RxHeader;
    if( hcan->Instance == hcan1.Instance ){
        if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanReceiveData) != HAL_OK)
        {

            Error_Handler();
        }
        CanDataDecode(RxHeader);
    }
}
