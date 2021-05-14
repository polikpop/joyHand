#ifndef __ADC_READ_H__
#define __ADC_READ_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"


#include "esp_system.h"
#include "esp_event.h"
//#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_master.h"
#include "esp_log.h"
//#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include <sys/socket.h>
#include "esp_event_loop.h"
#include "esp_log.h"



#endif