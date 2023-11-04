#include "OPS.h"
#include <stdio.h>

OPS_t OPS_Data = {0};
uint8_t ops_buffer[28]; // 接收缓冲区

#if (USE_POLL == 1)

#define OPS_FRAME_LEN 28
/**
 * @brief   OPS数据接收函数
 * @return  0: 成功；1: 未接收到足够字节；2: 接收到错误格式的数据
 */
uint8_t OPS_Get_Data(void)
{
    uint16_t size = 0;
    static union {
        uint8_t data[24];
        float ActVal[6];
    } posture;
    HAL_UARTEx_ReceiveToIdle(&OPS_UART_HANDLE, ops_buffer, OPS_FRAME_LEN, &size, 0xFF);
    if (size != 28) {
        for (int i = 0; i < 28; i++) {
            ops_buffer[i] = 0;
        }
        return 1;
    }
    if (ops_buffer[0] != 0x0D || ops_buffer[1] != 0x0A || ops_buffer[26] != 0x0A || ops_buffer[27] != 0x0D) {
        for (int i = 0; i < 28; i++) {
            ops_buffer[i] = 0;
        }
        return 2;
    }
    for (int i = 0; i < 24; i++) {
        posture.data[i] = ops_buffer[i + 2];
    }
    OPS_Data.z_angle = posture.ActVal[0];
    OPS_Data.x_angle = posture.ActVal[1];
    OPS_Data.y_angle = posture.ActVal[2];
    OPS_Data.pos_x   = posture.ActVal[3];
    OPS_Data.pos_y   = posture.ActVal[4];
    OPS_Data.w_z     = posture.ActVal[5];
    return 0;
}

#endif
#if (USE_IT == 1)
uint8_t _ops_buffer[28] = {0};
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == OPS_UART) {
        if (ops_buffer[0] == 0x0D && ops_buffer[1] == 0x0A && ops_buffer[26] == 0x0A && ops_buffer[27] == 0x0D) {
            memcpy(_ops_buffer, ops_buffer, 28);
            _Data_Get();
        }
        HAL_UART_Receive_IT(huart, ops_buffer, 28);
    }
}

/**
 * @brief		OPS数据处理函数
 */
void _Data_Get()
{
    static union {
        uint8_t data[24];
        float ActVal[6];
    } posture;
    static uint8_t i = 0;
    for (i = 0; i < 24; i++) {
        posture.data[i] = _ops_buffer[i + 2];
    }
    OPS_Data.z_angle = posture.ActVal[0];
    OPS_Data.x_angle = posture.ActVal[1];
    OPS_Data.y_angle = posture.ActVal[2];
    OPS_Data.pos_x   = posture.ActVal[3];
    OPS_Data.pos_y   = posture.ActVal[4];
    OPS_Data.w_z     = posture.ActVal[5];
}
#endif