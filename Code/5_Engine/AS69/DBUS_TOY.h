/* -------------------------Include Document-------------------------------- */
#ifndef __BSP_DMA_H_
#define __BSP_DMA_H_
#include "stm32f4xx.h"
#include <stdio.h>
#include <math.h>
#include "DJI.h"
#include "Caculate.h"
#include "wtr_can.h"
#include "tim.h"

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
/* -------------- Remote control data structure definition-----------------  */ 
typedef struct    
{
    struct 
    {
        uint16_t ch0;      // Channel 0 : right_right
        uint16_t ch1;      // Channel 1 : right_down
        uint16_t ch2;      // Channel 2 : left_right
        uint16_t ch3;      // Channel 3 : left_down
        uint8_t s1;        // button 1 : left
        uint8_t s2;        // button 2 £»right 
    }rc;
    struct 
    {
        uint8_t LED;                 //mode : LED
		    uint8_t Flag;                //mode : Flag
		    uint8_t Friction_motor;      //mode : Friction motor	
	  }mode;
}RC_Ctl_t;
extern RC_Ctl_t RC_CtrlData;
void RemoteDataProcess(uint8_t *pData);
#endif
