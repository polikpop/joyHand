#include "driver/gpio.h"

#ifndef KEY_B_H
#define KEY_B_H

//键盘所连接的IO口的定义
#define KEY_ROW1 27
#define KEY_ROW2 26
#define KEY_ROW3 25
#define KEY_ROW4 23
#define KEY_COL1 18 
#define KEY_COL2 19
#define KEY_COL3 21 
#define KEY_COL4 22

//键盘名称与键值表的对应配置
#define KEY_SW1 0
#define KEY_SW2 4
#define KEY_SW3 8
#define KEY_SW4 12
#define KEY_SW_UP 1
#define KEY_SW_DOWN 5
#define KEY_SW_LEFT 9
#define KEY_SW_RIGHT 13
#define KEY_PRESS_12 2
#define KEY_PRESS_34 6
#define KEY_PRESS_56 10
#define KEY_PRESS_78 14
#define KEY_SW5 3
#define KEY_SW6 7
#define KEY_TRIGGER_L 11
#define KEY_TRIGGER_R 15
#define KEY_S1 16
#define KEY_S2 17



//相关的函数设置宏
#define HIGH 1
#define LOW 0
#define KEY_ROW1_HIGH()  gpio_set_level(KEY_ROW1,HIGH)
#define KEY_ROW2_HIGH()  gpio_set_level(KEY_ROW2,HIGH)
#define KEY_ROW3_HIGH()  gpio_set_level(KEY_ROW3,HIGH)
#define KEY_ROW4_HIGH()  gpio_set_level(KEY_ROW4,HIGH)

#define KEY_ROW1_LOW()   gpio_set_level(KEY_ROW1,LOW)
#define KEY_ROW2_LOW()   gpio_set_level(KEY_ROW2,LOW)
#define KEY_ROW3_LOW()   gpio_set_level(KEY_ROW3,LOW)
#define KEY_ROW4_LOW()   gpio_set_level(KEY_ROW4,LOW)


#define READ_KEY_COL1()  gpio_get_level(KEY_COL1)
#define READ_KEY_COL2()  gpio_get_level(KEY_COL2)
#define READ_KEY_COL3()  gpio_get_level(KEY_COL3)
#define READ_KEY_COL4()  gpio_get_level(KEY_COL4)


//下面是函数和变量的声明
extern void KEY_Init();
extern void get_gpio_state();
extern void key_value_to_table();


extern uint8_t key_value[18];
extern int16_t key_table[9];
extern int16_t key_old_table[9];

#endif