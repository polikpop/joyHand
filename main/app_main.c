/***************************头文件引用***********************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/rmt_struct.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdio.h>
#include "ws2812.h"
#include <esp_log.h>
#include "Config.h"
#include "oled.h"
#include "fonts.h"
#include "keyboard.h"
#include <errno.h>
#include "esp_err.h"
#include "connect.h"
#include "driver/ledc.h"
/*************************************************************/

/*****************************RGB端口定义***************************/
#define WS2812_PIN1 16
#define WS2812_PIN2 17	
/*****************************************************************/

#define delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)					   // 延时函数的宏


/*****************************ADC相关定义********************************************/

//限幅宏，让x的值在min和max之间，这里用于ADC的数值限幅
#define LIMIT(x, max, min) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x))) 
//ADC的死区值，这个手柄目前15不太可能抖，也够用
#define DEAD_ZONE_VALUE 15
//死区宏，给ADC的变化搞个死区
#define DEAD_ZONE(x) (x > DEAD_ZONE_VALUE) ? (x) : ((x < -DEAD_ZONE_VALUE) ? (x) : 0)
//adc初始偏差 需要减去，因为我们的摇杆有四个所以数组大小为4
int adc_value_diff[4] = {0}; 
// ADC所接的通道，添加通道直接在后面 或 操作就行  
#define ADC1_TEST_CHANNEL ADC1_CHANNEL_0 | ADC1_CHANNEL_3 | ADC1_CHANNEL_4 | ADC1_CHANNEL_5 | ADC1_CHANNEL_6
// ADC斜率曲线
static esp_adc_cal_characteristics_t *adc_chars;
// 参考电压
#define DEFAULT_VREF 3300 
//电压预警信号位
int need_charge = 0;
/********************************************************************************************/

/*****************************************WIFI相关部分*************************************************/
//创建TCP的任务函数，里面一些具体的宏都定义在connect.h/c中
static void tcp_conn(void *pvParameters)
{
	while (1)
	{
		g_rxtx_need_restart = false;

		ESP_LOGI(TAG, "task tcp_conn...");

		/*wating for connecting to AP*/
		xEventGroupWaitBits(tcp_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
		TaskHandle_t tx_rx_task = NULL;

#if TCP_SERVER_CLIENT_OPTION

		ESP_LOGI(TAG, "tcp_server will start after 3s...");
		vTaskDelay(1000 / portTICK_RATE_MS);
		ESP_LOGI(TAG, "create_tcp_server.");
		int socket_ret = create_tcp_server(true);
#else
		ESP_LOGI(TAG, "tcp_client will start after 3s...");
		vTaskDelay(1000 / portTICK_RATE_MS);
		ESP_LOGI(TAG, "create_tcp_Client.");
		int socket_ret = create_tcp_client();
#endif
		if (socket_ret == ESP_FAIL)
		{
			ESP_LOGI(TAG, "create tcp socket error,stop...");
			continue;
		}
		else
		{
			ESP_LOGI(TAG, "create tcp socket succeed...");
		}

		if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096, NULL, 4, &tx_rx_task))
		{
			ESP_LOGI(TAG, "Recv task create fail!");
		}
		else
		{
			ESP_LOGI(TAG, "Recv task create succeed!");
		}

		double bps;

		while (1)
		{

			//vTaskDelay(1000 / portTICK_RATE_MS);

#if TCP_SERVER_CLIENT_OPTION

			if (g_rxtx_need_restart)
			{
				ESP_LOGE(TAG, "tcp server send or receive task encoutner error, need to restart...");

				if (ESP_FAIL != create_tcp_server(false))
				{
					if (pdPASS != xTaskCreate(&recv_data, "recv_data", 4096, NULL, 4, &tx_rx_task))
					{
						ESP_LOGE(TAG, "tcp server Recv task create fail!");
					}
					else
					{
						ESP_LOGE(TAG, "tcp server Recv task create succeed!");
					}
				}
			}
#else
			if (g_rxtx_need_restart)
			{
				ESP_LOGI(TAG, "tcp_client will reStart after 3s...");
				need_restart = 1;
				vTaskDelay(1000 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "create_tcp_Client...");
				int socket_ret = create_tcp_client();

				if (socket_ret == ESP_FAIL)
				{
					ESP_LOGE(TAG, "create tcp socket error,stop...");
					continue;
				}
				else
				{
					ESP_LOGI(TAG, "create tcp socket succeed...");
					g_rxtx_need_restart = false;
				}

				if (pdPASS != xTaskCreate(&send_data, "send_data", 4096, NULL, 6, &tx_rx_task))
				{
					ESP_LOGE(TAG, "Send task create fail!");
				}
				else
				{
					ESP_LOGI(TAG, "Send task create succeed!");
				}
			}
			vTaskDelay(50);
#endif
		}
	}

	vTaskDelete(NULL);
}
/*********************************************************************************************/


/***********************************ADC相关配置及读取********************************************/
void adc_Init()
{
	adc1_config_width(ADC_WIDTH_BIT_9);							   // 9位分辨率
	adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_6); // 电压输入衰减
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));	// 为斜率曲线分配内存
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_9, DEFAULT_VREF, adc_chars);
																   
}

void init_adc_scanner()
{
	int cnt = 0;
	int key_value_sum[4] = {0};
	while (1)
	{
		
		key_table[5] = adc1_get_raw(ADC1_CHANNEL_0);
		key_table[6] = adc1_get_raw(ADC1_CHANNEL_3);
		key_table[7] = adc1_get_raw(ADC1_CHANNEL_4);
		key_table[8] = adc1_get_raw(ADC1_CHANNEL_5);

		for (int i = 0; i < 4; i++)
		{
			key_value_sum[i] += key_table[i + 5];
		}
		cnt++;
		if (cnt == 40)
		{
			for (int i = 0; i < 4; i++)
			{
				adc_value_diff[i] = key_value_sum[i] / 40;
			}
			break;
		}
	}
}

void adc_scanner()
{

		key_table[5] = adc1_get_raw(ADC1_CHANNEL_0);
		key_table[6] = adc1_get_raw(ADC1_CHANNEL_3);
		key_table[7] = adc1_get_raw(ADC1_CHANNEL_4);
		key_table[8] = adc1_get_raw(ADC1_CHANNEL_5);


		for (int i = 5; i < 9; i++)
		{
			key_table[i] -= adc_value_diff[i - 5];
			key_table[i] = DEAD_ZONE(key_table[i]);
			// key_table
			key_table[i] = LIMIT(key_table[i], 195, -195);
		}
		
		uint16_t read_raw = adc1_get_raw(ADC1_CHANNEL_6);// 采集ADC原始值
		uint32_t voltage = esp_adc_cal_raw_to_voltage(read_raw, adc_chars);//通过一条斜率曲线把读取adc1_get_raw()的原始数值转变成了mV
		float true_vol = (voltage + 880.0) * 13.3 / 3300;
		//printf("%f\n", true_vol);
		if((true_vol - 7.08) < 1e-2) {
		 	need_charge = 1;
			//printf("yes\n");
		}  
		
		// for (int i = 0; i <= 7; i++)
		// {
		// 	printf("%d ", key_table[i]);
		// }
		// printf("\n");

}

/*********************************************************************************************/

/***************************************RGB相关************************************************/
// 彩虹效果实现函数
void rainbow(void *pvParameters)
{
	const uint8_t anim_step = 1;
	const uint8_t anim_max = 10;
	const uint8_t pixel_count = 128; // Number of your "pixels"
	const uint8_t delay = 5;		 // duration between color changes
	rgbVal color = makeRGBVal(anim_max, 0, 0);
	uint8_t step = 0;
	rgbVal color2 = makeRGBVal(anim_max, 0, 0);
	uint8_t step2 = 0;
	rgbVal *pixels;
	pixels = malloc(sizeof(rgbVal) * pixel_count);
	while (1)
	{
		color = color2;
		step = step2;
		for (uint8_t i = 0; i < pixel_count; i++)
		{
			pixels[i] = color;
			if (i == 1)
			{
				color2 = color;
				step2 = step;
			}
			switch (step)
			{
			case 0:
				color.g += anim_step;
				if (color.g >= anim_max)
					step++;
				break;
			case 1:
				color.r -= anim_step;
				if (color.r == 0)
					step++;
				break;
			case 2:
				color.b += anim_step;
				if (color.b >= anim_max)
					step++;
				break;
			case 3:
				color.g -= anim_step;
				if (color.g == 0)
					step++;
				break;
			case 4:
				color.r += anim_step;
				if (color.r >= anim_max)
					step++;
				break;
			case 5:
				color.b -= anim_step;
				if (color.b == 0)
					step = 0;
				break;
			}
		}
		ws2812_setColors(pixel_count, pixels);
		vTaskDelay(delay);
	}
}

/*************************************************************************/


/****************************OLED相关*************************************/
void oled_show(void *pvParameters)
{
	unsigned int cnt = 0;
	oled_init();
	oled_all_on();
	oled_show_str(0, 2, "JOYHAND", &Font_7x10, 0);
	oled_show_str(0, 15, "init now", &Font_7x10, 0);
	oled_show_str(0, 30, "kwaii", &Font_7x10, 0);
	oled_show_str(0, 45, "please wait", &Font_7x10, 0);
	//vTaskDelay(10000 / portTICK_PERIOD_MS);
	while(1)
	{
	//cnt++;
	 //oled_claer();
	 if (sending == 1) {
		oled_claer();
		oled_all_on();
		oled_show_str(0, 15, "sending now", &Font_7x10, 0);
		oled_show_str(0, 30, "You Can DO", &Font_7x10, 0);
		sending =  0;
	 }
	 if (need_restart) {
		oled_claer();
		oled_all_on();
		oled_show_str(0, 15, "disconnect now", &Font_7x10, 0);
		oled_show_str(0, 30, "need restart", &Font_7x10, 0);
		need_restart =  0;
	 }

	 if (need_charge) {
		 while(1) {
		   oled_claer();
		   oled_all_on();
		   oled_show_str(0, 15, "low power", &Font_7x10, 0);
		   oled_show_str(0, 30, "need charge", &Font_7x10, 0);
		 }
	 }
	 vTaskDelay(1 / portTICK_PERIOD_MS);
	
	 //vTaskDelay(1 / portTICK_PERIOD_MS);
	 //ESP_LOGI("OLED", "cnt = %d \r\n", cnt);
	}
}
/************************************************************************/



/***************************键盘扫描***************************************/
void key_scanner(void *pvParameters)
{
	
	init_adc_scanner();
	while (1)
	{
		
		get_gpio_state();
		key_value_to_table();
		adc_scanner();
	}
}
/************************************************************************/

/*****************************蜂鸣器相关**********************************/
void BEEP_Init() {
	ledc_timer_config_t ledc_timer;
	ledc_timer.duty_resolution = LEDC_TIMER_10_BIT;
	ledc_timer.freq_hz = 260;
	ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;
	ledc_timer.timer_num = LEDC_TIMER_1;
	ledc_timer.clk_cfg = LEDC_AUTO_CLK;
	ledc_timer_config(&ledc_timer);

	ledc_channel_config_t beep;
	beep.channel = LEDC_CHANNEL_1;
	beep.duty = 50;
	beep.gpio_num = 12;
	beep.speed_mode = LEDC_HIGH_SPEED_MODE;
	beep.timer_sel = LEDC_TIMER_1;
	ledc_channel_config(&beep);
}
/**************************************************************************/


void app_main()
{
	ESP_LOGI(TAG, "APP Start......");
	nvs_flash_init();

	#if TCP_SERVER_CLIENT_OPTION
		ESP_LOGI(TAG, "As a Tcp Server , will start wifi_init_softap...");
		wifi_init_softap();
	#else

		ESP_LOGI(TAG, "As a Tcp Client , will start wifi_init_sta...");
		ws2812B_init(WS2812_PIN1);
		ws2812B_init(WS2812_PIN2);
		wifi_init_sta();
		adc_Init();
		oled_init();
		KEY_Init();
		xTaskCreatePinnedToCore(key_scanner, "scan", 4096, NULL, 10, NULL, 1);
		xTaskCreatePinnedToCore(oled_show, "oled", 4096, NULL, 10, NULL, 1);
		//xTaskCreatePinnedToCore(rainbow, "ws2812 rainbow demo", 4096, NULL, 10, NULL, 1);
	#endif
		xTaskCreatePinnedToCore(&tcp_conn, "tcp_conn", 4096, NULL, 10, NULL, 0);
	// oled_init();
	// adc_Init();
	// KEY_Init();
	//BEEP_Init();
	//ws2812B_init(WS2812_PIN1);
	//ws2812B_init(WS2812_PIN2);
	// xTaskCreatePinnedToCore(key_scanner, "scan", 4096, NULL, 10, NULL, 1);
	//xTaskCreatePinnedToCore(rainbow, "ws2812 rainbow demo", 4096, NULL, 10, NULL, 1);
	// xTaskCreatePinnedToCore(oled_show, "oled", 4096, NULL, 10, NULL, 1);

	return;
}