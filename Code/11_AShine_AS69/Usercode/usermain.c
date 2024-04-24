#include "usermain.h"

void StartDefaultTask(void *argument)
{
    RetargetInit(&huart6);
    AS69_RemoteControl_Init();
    for (;;) {
        printf("%d\r\n", ReadJoystickButtons(&msg_joystick_air, Btn_Btn0));
        osDelay(10);
    }
}