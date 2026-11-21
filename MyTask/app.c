#include "app.h"
TaskHandle_t Block_Manipulator_Handle;
Motor2006Ex_t manipulator[8];//0作为
float init_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float climb_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float turn_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float turn_init_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t motorCurrentBuf_1[4] = {0};
int16_t motorCurrentBuf_2[4] = {0};

static void Manipulator_Config(void)
{
    static const uint32_t id_tab[8] = { 0x201, 0x202, 0x203, 0x204,
                                        0x205, 0x206, 0x207, 0x208 };

    for (uint8_t i = 0; i < 8; i++)
    {
        Motor2006Ex_t *m = &manipulator[i];

        m->hcan = (i < 4) ? &hcan1 : &hcan2;  
        m->ID   = id_tab[i];

        m->pos_pid.Kp = 1.0f;
        m->pos_pid.Ki = 0.0f;
        m->pos_pid.Kd = 0.0f;
        m->pos_pid.limit = 6000.0f;
        m->pos_pid.output_limit = 6000.0f;

        m->vel_pid.Kp = 5.0f;
        m->vel_pid.Ki = 0.0f;
        m->vel_pid.Kd = 0.0f;
        m->vel_pid.limit = 16384.0f;
        m->vel_pid.output_limit = 10000.0f;
    }
}

void climb_down(void)
{
motorCurrentBuf_1[0] = climb_rad[0];
motorCurrentBuf_1[1] = climb_rad[1];
motorCurrentBuf_1[2] = climb_rad[2];
motorCurrentBuf_1[3] = climb_rad[3];
motorCurrentBuf_2[0] = climb_rad[4];
motorCurrentBuf_2[1] = climb_rad[5];
motorCurrentBuf_2[2] = climb_rad[6];
motorCurrentBuf_2[3] = climb_rad[7];
}
void climb_up(void)
{
motorCurrentBuf_1[0] = init_rad[0];
motorCurrentBuf_1[1] = init_rad[1];
motorCurrentBuf_1[2] = init_rad[2];
motorCurrentBuf_1[3] = init_rad[3];
motorCurrentBuf_2[0] = init_rad[4];
motorCurrentBuf_2[1] = init_rad[5];
motorCurrentBuf_2[2] = init_rad[6];
motorCurrentBuf_2[3] = init_rad[7];
}
void turn()
{
motorCurrentBuf_1[0] = turn_rad[0];
motorCurrentBuf_1[1] = turn_rad[1];
motorCurrentBuf_1[2] = turn_rad[2];
motorCurrentBuf_1[3] = turn_rad[3];
motorCurrentBuf_2[0] = turn_rad[4];
motorCurrentBuf_2[1] = turn_rad[5];
motorCurrentBuf_2[2] = turn_rad[6];
motorCurrentBuf_2[3] = turn_rad[7];
}
void turn_back()
{
motorCurrentBuf_1[0] = turn_init_rad[0];
motorCurrentBuf_1[1] = turn_init_rad[1];
motorCurrentBuf_1[2] = turn_init_rad[2];
motorCurrentBuf_1[3] = turn_init_rad[3];
motorCurrentBuf_2[0] = turn_init_rad[4];
motorCurrentBuf_2[1] = turn_init_rad[5];
motorCurrentBuf_2[2] = turn_init_rad[6];
motorCurrentBuf_2[3] = turn_init_rad[7];
}
void Block_Manipulator(void *pvParameters)
{ 
    Manipulator_Config();
    portTickType xLastWakeTime = xTaskGetTickCount();

    for(;;)
    {
        MotorSend(&hcan1, 0x200, motorCurrentBuf_1);
        MotorSend(&hcan1, 0x1FF, motorCurrentBuf_2);
        vTaskDelayUntil(&xLastWakeTime, 2);
    }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	 uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);

	Motor2006Recv(&manipulator[0], hcan, ID, Recv);
	Motor2006Recv(&manipulator[1], hcan, ID, Recv);
	Motor2006Recv(&manipulator[2], hcan, ID, Recv);
	Motor2006Recv(&manipulator[3], hcan, ID, Recv);
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID_1 = CAN_Receive_DataFrame(hcan, Recv);
	Motor2006Recv(&manipulator[4], hcan, ID_1, Recv);
	Motor2006Recv(&manipulator[5], hcan, ID_1, Recv);
	Motor2006Recv(&manipulator[6], hcan, ID_1, Recv);
	Motor2006Recv(&manipulator[7], hcan, ID_1, Recv);
}
