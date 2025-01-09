#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "Shared/common.h"
#include "Shared/messages.h"

//Local variables

// 1 kHz tasks
void syncTaskApp(void)
{
    
}