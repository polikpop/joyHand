
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

#define WS2812_PIN1 16
#define WS2812_PIN2 17														   // WS2812 所连接的GPIO
#define delay_ms(ms) vTaskDelay((ms) / portTICK_RATE_MS)					   // 延时函数的宏
#define LIMIT(x, max, min) ((x) > (max) ? (max) : ((x) < (min) ? (min) : (x))) //限幅
// // #define ABS(x) ((x) > 0) ? (x) : (-x)
#define DEAD_ZONE_VALUE 15
#define DEAD_ZONE(x) (x > DEAD_ZONE_VALUE) ? (x) : ((x < -DEAD_ZONE_VALUE) ? (x) : 0)

int adc_value_diff[4] = {0}; //adc初始偏差 需要减去

// ADC所接的通道  GPIO34 if ADC1  = ADC1_CHANNEL_6
#define ADC1_TEST_CHANNEL ADC1_CHANNEL_0 | ADC1_CHANNEL_3 | ADC1_CHANNEL_4 | ADC1_CHANNEL_5 | ADC1_CHANNEL_6
// ADC斜率曲线
static esp_adc_cal_characteristics_t *adc_chars;
// 参考电压
#define DEFAULT_VREF 3300 //Use adc2_vref_to_gpio() to obtain a better estimate

//电压预警信号位
int need_charge = 0;


/*******************WIFI*****************/
//this task establish a TCP connection and receive data from TCP
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
//////////////////////////////////////////////////////////

//
typedef struct Kalman_filter
{
	float C_last; /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X_last; /*系统状态预测矩阵，列矩阵*/

	float Q; /*过程噪声协方差*/
	float R; /*量测噪声协方差*/

	float K; /*卡尔曼增益，列矩阵*/
	float X; /*最优估计输出矩阵，列矩阵*/
	float C; /*最优估计协方差矩阵C(k|k)*/

	float Input; /*量测值，即Z(k)*/
} Kal_Filter;
Kal_Filter Flt1 =
	{1.0f,	  //k_flt.C_last
	 0.0f,	  //k_flt.X_last
	 0.0001f, //k_flt.Q
	 0.005f,  //k_flt.R 4.0
	 0.0f, 0.0f, 0.0f, 0.0f};
float Kalman_Filter(Kal_Filter *K_Flt, float Input)
{
	/*量测更新，3组方程*/
	K_Flt->Input = Input;
	K_Flt->K = (K_Flt->C_last) / (K_Flt->C_last + K_Flt->R);
	K_Flt->X = K_Flt->X_last + K_Flt->K * (K_Flt->Input - K_Flt->X_last);
	K_Flt->C = (1 - K_Flt->K) * (K_Flt->C_last);

	/*时间更新，2组方程*/
	K_Flt->X_last = K_Flt->X;
	K_Flt->C_last = K_Flt->C + K_Flt->Q;

	return K_Flt->X;
}

void adc_Init()
{
	adc1_config_width(ADC_WIDTH_BIT_9);							   // 12位分辨率
	adc1_config_channel_atten(ADC1_TEST_CHANNEL, ADC_ATTEN_DB_6); // 电压输入衰减
										   //adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));	// 为斜率曲线分配内存

	//adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_6);// 电压输入衰减
	adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));	// 为斜率曲线分配内存
	esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_6, ADC_WIDTH_BIT_9, DEFAULT_VREF, adc_chars);
																   //esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

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

void init_adc_scanner()
{
	int cnt = 0;
	int key_value_sum[4] = {0};
	while (1)
	{
		// key_table[4] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_0) * 0.2 / 16  + pre_adc1 * 0.8 );// 采集ADC原始值
		// key_table[5] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_3) * 0.2 / 16  + pre_adc2 * 0.8 );
		// key_table[6] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_6) * 0.2/ 16  + pre_adc3 * 0.8 );
		// key_table[7] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_7) * 0.2/ 16  + pre_adc4 * 0.8 );
		
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
			// int adc_value_diff[4] = {0}; //adc初始偏差 需要减去
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
	// uint32_t pre_adc1, pre_adc2, pre_adc3, pre_adc4;
	// pre_adc1 = adc1_get_raw(ADC1_CHANNEL_0) / 16;
	// pre_adc2 = adc1_get_raw(ADC2_CHANNEL_3) / 16;
	// pre_adc3 = adc1_get_raw(ADC2_CHANNEL_6) / 16;
	// pre_adc4 = adc1_get_raw(ADC2_CHANNEL_7) / 16;

	
	
		// key_table[4] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_0) * 0.2 / 16  + pre_adc1 * 0.8 );// 采集ADC原始值
		// key_table[5] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_3) * 0.2 / 16  + pre_adc2 * 0.8 );
		// key_table[6] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_6) * 0.2/ 16  + pre_adc3 * 0.8 );
		// key_table[7] = (uint8_t) (adc1_get_raw(ADC1_CHANNEL_7) * 0.2/ 16  + pre_adc4 * 0.8 );

		key_table[5] = adc1_get_raw(ADC1_CHANNEL_0);
		key_table[6] = adc1_get_raw(ADC1_CHANNEL_3);
		key_table[7] = adc1_get_raw(ADC1_CHANNEL_4);
		key_table[8] = adc1_get_raw(ADC1_CHANNEL_5);

		// key_table[4] -= 291;
		// key_table[5] -= 255;
		// key_table[6] -= 269;
		// key_table[7] -= 276;

		for (int i = 5; i < 9; i++)
		{
			key_table[i] -= adc_value_diff[i - 5];
			key_table[i] = DEAD_ZONE(key_table[i]);
			// key_table
			key_table[i] = LIMIT(key_table[i], 195, -195);
		}
		// pre_adc1 = key_table[4];
		// pre_adc2 = key_table[5];
		// pre_adc3 = key_table[6];
		// pre_adc4 = key_table[7];
		
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

void key_scanner(void *pvParameters)
{
	//  KEY_ROW2_LOW();
	//  KEY_ROW1_HIGH();
	//  KEY_ROW3_HIGH();
	//  KEY_ROW4_HIGH();
	init_adc_scanner();
	while (1)
	{
		//key_value[1] = READ_KEY_COL1();
		//gpio_pad_set_drv(23,0);
		get_gpio_state();
		key_value_to_table();
		adc_scanner();
		//char* sendbuff;
		//sendbuff = convert_value_to_char(key_table);
		//printf("%s\n", sendbuff);
		//gpio_set_level(23,1);
		//vTaskDelay(5);
		//gpio_set_level(23,0);
		//xTaskCreate(adc_scanner, "adc_scan", 4096, NULL, 10, NULL);

		// for (int i = 0; i < 9; i ++) {
		// 	printf("%d ", key_table[i]);
		// }
		// printf("\n");
		// vTaskDelay(5);

		// for (int i = 0; i < 16; i ++) {
		// 	printf("%d ", key_value[i]);
		// }
		// printf("\n");
		// vTaskDelay(5);
	}
}

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

// BEEP_ON(void *pvParameters) {
// 	while(1) {
		
// 	}
// }

void app_main()
{
	ESP_LOGI(TAG, "APP Start......");
	nvs_flash_init();
	//BEEP_Init();
	//ws2812B_init(WS2812_PIN1);
	//ws2812B_init(WS2812_PIN2);
	//xTaskCreate(BEEP_ON, "BEEP", 4096, NULL, 10, NULL);
	//KEY_Init();
	//xTaskCreate(rainbow, "ws2812 rainbow demo", 4096, NULL, 10, NULL);
	//adc_Init();
	//xTaskCreate(oled_show, "print",4096, NULL, 10, NULL);
	//oled_show();
	//xTaskCreatePinnedToCore(adc_scanner, "adc_scan", 4096, NULL, 10, NULL, 1);
	//BEEP_Init();

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