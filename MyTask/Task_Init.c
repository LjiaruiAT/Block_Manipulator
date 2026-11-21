#include "Task_Init.h"
#include "app.h"

void Task_Init()
{
    xTaskCreate(Block_Manipulator,
                "Block_Manipulator",
                256,
                NULL,
                4,
                &Block_Manipulator_Handle);
}
