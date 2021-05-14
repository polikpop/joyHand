#include "keyboard.h"
#include "driver/gpio.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
uint8_t key_value[18];
//uint8_t key_table[8];
int16_t key_table[9];
int16_t key_old_table[9];
void KEY_Init() {
    	//配置GPIO行结构体
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_ANYEDGE;		// 下降沿和上升沿触发中断
	//io_conf.pin_bit_mask = ((1 << (KEY_ROW1-1)) | (1 << (KEY_ROW2-1)) | (1 << (KEY_ROW3-1)) | (1 << (KEY_ROW4-1)));	// 设置GPIO号
	io_conf.pin_bit_mask = ((1 << KEY_ROW1) | (1 << KEY_ROW2) | (1 << KEY_ROW3) | (1 << KEY_ROW4));
	io_conf.mode = GPIO_MODE_OUTPUT;				// 模式输出
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;	//不上拉也不下拉
	gpio_config(&io_conf);


        //配置GPIO列结构体
	io_conf.intr_type = GPIO_INTR_ANYEDGE;		// 下降沿和上升沿触发中断
	io_conf.pin_bit_mask = ((1 << KEY_COL1) | (1 << KEY_COL2) | (1 << KEY_COL3) | (1 << KEY_COL4));	// 设置GPIO号
	io_conf.mode = GPIO_MODE_INPUT;				// 模式输入
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;	
	gpio_config(&io_conf);   

	io_conf.intr_type = GPIO_INTR_ANYEDGE;		// 下降沿和上升沿触发中断
	io_conf.pin_bit_mask = ((1 << 14) | (1 << 15));	// 设置GPIO号
	io_conf.mode = GPIO_MODE_INPUT;				// 模式输入
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;	
	gpio_config(&io_conf);   
}

void get_gpio_state() {
	uint8_t i;
	for (i = 0; i < 4; i ++) {
		KEY_ROW1_HIGH();
		KEY_ROW2_HIGH();
		KEY_ROW3_HIGH();
		KEY_ROW4_HIGH();
		switch(i) {
			case 0: KEY_ROW1_LOW(); break;
			case 1: KEY_ROW2_LOW(); break;
			case 2: KEY_ROW3_LOW(); break;
			case 3: KEY_ROW4_LOW(); break;
			default: break;
		}
		vTaskDelay(1);
		key_value[4*0 + i] = READ_KEY_COL1();
		key_value[4*1 + i] = READ_KEY_COL2();
		key_value[4*2 + i] = READ_KEY_COL3();
		key_value[4*3 + i] = READ_KEY_COL4();
	}

	key_value[16] = gpio_get_level(14);
	key_value[17] = gpio_get_level(15);
}


void key_value_to_table() {
	uint8_t i;
	for(i=0; i<9; i++)
	{
		key_old_table[i] = key_table[i];
		key_table[i] = 0x00;
	}
	
	if(!key_value[KEY_S1]) {
		key_table[0] |= 0x02;
	}
	if(!key_value[KEY_S2]) {
		key_table[0] |= 0x01;
	}
	if(!key_value[KEY_SW1]) {
		key_table[1] |= 0x08;
	}

	if(!key_value[KEY_SW2]) {
		key_table[1] |= 0x04;
	}

	if(!key_value[KEY_SW3]) {
		key_table[1] |= 0x02;
	}

	if(!key_value[KEY_SW4]) {
		key_table[1] |= 0x01;
	}

	if(!key_value[KEY_SW_UP]) {
		key_table[2] |= 0x08;
	}

	if(!key_value[KEY_SW_DOWN]) {
		key_table[2] |= 0x04;
	}

	if(!key_value[KEY_SW_LEFT]) {
		key_table[2] |= 0x02;
	}

	if(!key_value[KEY_SW_RIGHT]) {
		key_table[2] |= 0x01;
	}

	if(!key_value[KEY_PRESS_12]) {
		key_table[3] |= 0x08;
	}

	if(!key_value[KEY_PRESS_34]) {
		key_table[3] |= 0x04;
	}

	if(!key_value[KEY_PRESS_56]) {
		key_table[3] |= 0x02;
	}

	if(!key_value[KEY_PRESS_78]) {
		key_table[3] |= 0x01;
	}

	if(!key_value[KEY_SW5]) {
		key_table[4] |= 0x08;
	}

	if(!key_value[KEY_SW6]) {
		key_table[4] |= 0x04;
	}

	if(!key_value[KEY_TRIGGER_L]) {
		key_table[4] |= 0x02;
	}

	if(!key_value[KEY_TRIGGER_R]) {
		key_table[4] |= 0x01;
	}
}