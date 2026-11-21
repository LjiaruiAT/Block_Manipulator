#include "app.h"
TaskHandle_t Block_Manipulator_Handle;
Motor2006Ex_t manipulator[8];//0作为
float init_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float climb_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float turn_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float turn_init_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
float send_rad[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int16_t motorCurrentBuf_1[4] = {0};
int16_t motorCurrentBuf_2[4] = {0};
typedef enum {
    BR_GET_SCORE_IDLE = 0,
    UP_67,
	DOWN_67,
	UP_45,
	DOWN_45,
	PULL_APART_3,
	RETRACT_3,
	TURN_12,
	TURN_BACK_12,
} BR_get_score_state_t;
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
static float Manipulator_Rad(uint8_t i)
{
    return (float)(manipulator[i].motor.Angle - manipulator[i].offset)
           * (360.0f / 8192.0f) / gear_Ratio;
}

static uint8_t Manipulator_Ready(void)
{
    for (uint8_t i = 0; i < 8; i++)
        if (!manipulator[i].ready) return 0;
    return 1;
}
void climb_down_67(void)
{
send_rad[5] = climb_rad[5];
send_rad[6] = climb_rad[6];
}
void climb_up_67(void)
{
send_rad[5] = init_rad[5];
send_rad[6] = init_rad[6];
}
void climb_down_45(void)
{
send_rad[3] = climb_rad[3];
send_rad[4] = climb_rad[4];
}
void climb_up_45(void)
{
send_rad[3] = init_rad[3];
send_rad[4] = init_rad[4];
}
void Pull_apart_3(void)
{
send_rad[2] = climb_rad[2];
}
void Retract_3(void)
{
send_rad[2] = init_rad[2];
}
void turn_12()
{
send_rad[0] = turn_rad[0] + 90.0f;
send_rad[1] = turn_rad[1] - 90.0f;
}
void turn_back_12()
{
send_rad[0] = turn_init_rad[0];
send_rad[1] = turn_init_rad[1];
}
static BR_get_score_state_t BR_get_score_state = BR_GET_SCORE_IDLE;
void Block_Manipulator(void *pvParameters)
{
    Manipulator_Config();

    for (uint8_t n = 0; n < 10 && !Manipulator_Ready(); n++)
    {
        MotorSend(&hcan1, 0x200, motorCurrentBuf_1);
        MotorSend(&hcan2, 0x1FF, motorCurrentBuf_2);
		        vTaskDelay(pdMS_TO_TICKS(2));
	}
for (uint8_t i = 0; i < 8; i++)
{
    init_rad[i] = turn_init_rad[i]  = Manipulator_Rad(i);
	turn_rad[0] = init_rad[0];
	turn_rad[1] = init_rad[1];
	turn_rad[2] = init_rad[2];
	turn_rad[3] = init_rad[3];
	turn_rad[4] = init_rad[4];
	turn_rad[5] = init_rad[5];
	turn_rad[6] = init_rad[6];
	turn_rad[7] = init_rad[7];

	climb_rad[0] = init_rad[0];
    climb_rad[1] = init_rad[1];
	climb_rad[2] = init_rad[2];
	climb_rad[3] = init_rad[3];
	climb_rad[4] = init_rad[4];
	climb_rad[5] = init_rad[5];
	climb_rad[6] = init_rad[6];
	climb_rad[7] = init_rad[7];
    }
    TickType_t xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
		switch (BR_get_score_state)
		{
			case BR_GET_SCORE_IDLE:
			climb_down_67();
			climb_down_45();
			Pull_apart_3();
				break;
			case UP_67:
            climb_up_67();

				break;
			case DOWN_67:
            climb_down_67();

				break;
			case UP_45:
            climb_up_45();

				break;
			case DOWN_45:
            climb_down_45();

				break;
			case PULL_APART_3:
            Pull_apart_3();

				break;
			case RETRACT_3:
            Retract_3();

				break;
			case TURN_12:
            turn_12();

				break;
			case TURN_BACK_12:
            turn_back_12();

				break;
			default:
				break;
		}
		for (uint8_t i = 0; i < 8; i++)
        {
            PID_Control2(Manipulator_Rad(i), send_rad[i], &manipulator[i].pos_pid);
            PID_Control2((float)manipulator[i].motor.Speed,manipulator[i].pos_pid.pid_out,&manipulator[i].vel_pid);
        }
		for (uint8_t i = 0; i < 4; i++)
        {
            motorCurrentBuf_1[i] = (int16_t)manipulator[i].vel_pid.pid_out;      
            motorCurrentBuf_2[i] = (int16_t)manipulator[4 + i].vel_pid.pid_out;  
        }
        MotorSend(&hcan1, 0x200, motorCurrentBuf_1);
        MotorSend(&hcan2, 0x1FF, motorCurrentBuf_2);
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
