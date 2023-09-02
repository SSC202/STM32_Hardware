/* ------  Include Document ------- */
#include "DBUS_TOY.h"
/* ------ Definition of data ------ */
RC_Ctl_t RC_CtrlData;                      //Definition of TOY;

/* ------ Decode the toy ---------- */
void RemoteDataProcess(uint8_t *pData)
{
    if(pData == NULL)
    {
       return;
    }
    
    /* Decode the rocker */
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch0 -= 1024;
    
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch1 -= 1024;
    
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch2 -= 1024;
    
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    RC_CtrlData.rc.ch3 -= 1024;
    
    /* Decode the button */
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);
    /* OFF CL : 0x01 -- HL :0x00 */
    /* GPS : 0x01 -- ATTI(MIDDLE) : 0x03 -- ATTI(DOWN) : 0x02 */    
    /* USER CODE BEGIN 0 */
    switch(RC_CtrlData.rc.s2)
    {
      case 1:
        RC_CtrlData.mode.LED                   = 0;
        RC_CtrlData.mode.Friction_motor        = 1;
        RC_CtrlData.mode.Flag                  = 0;
        break;
      case 3:
        RC_CtrlData.mode.LED                   = 0;
        RC_CtrlData.mode.Friction_motor        = 0;
        RC_CtrlData.mode.Flag                  = 1;
        break;
      case 2:
        RC_CtrlData.mode.LED                   = 1;
        RC_CtrlData.mode.Friction_motor        = 0;
        RC_CtrlData.mode.Flag                  = 0;
        break;
      default:
        break;
    }
    switch(RC_CtrlData.rc.s1)
    {
      case 1:
        if(RC_CtrlData.mode.LED == 0 && RC_CtrlData.mode.Friction_motor == 1 && RC_CtrlData.mode.Flag == 0)
        {
           HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_SET);
          
           __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 199); 
//           HAL_Delay(1000);
          
           speedServo(50*(1),&hDJI[0]);
           CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,0,0,0);
			     HAL_Delay(2);
        }
        else if(RC_CtrlData.mode.LED == 0 && RC_CtrlData.mode.Friction_motor == 0 && RC_CtrlData.mode.Flag == 1)
        {
          HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_SET);
        }
        else if(RC_CtrlData.mode.LED == 1 && RC_CtrlData.mode.Friction_motor == 0 && RC_CtrlData.mode.Flag == 0)
        {
          HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_SET);
        }
        break;
      case 0:
        if(RC_CtrlData.mode.LED == 0 && RC_CtrlData.mode.Friction_motor == 1 && RC_CtrlData.mode.Flag == 0)
        {
           HAL_GPIO_WritePin(GPIOG,GPIO_PIN_1,GPIO_PIN_RESET);
          
           __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 99); 
//           HAL_Delay(1000);
          
           speedServo(50*(1),&hDJI[0]);
           CanTransmit_DJI_1234(&hcan1,hDJI[0].speedPID.output,0,0,0);
			     HAL_Delay(2);
        }
        else if(RC_CtrlData.mode.LED == 0 && RC_CtrlData.mode.Friction_motor == 0 && RC_CtrlData.mode.Flag == 1)
        {
          HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,GPIO_PIN_RESET);
        }
        else if(RC_CtrlData.mode.LED == 1 && RC_CtrlData.mode.Friction_motor == 0 && RC_CtrlData.mode.Flag == 0)
        {
          HAL_GPIO_WritePin(GPIOG,GPIO_PIN_3,GPIO_PIN_RESET);
        }
        break;
    }
 }
	/* USER CODE END 0 */
