#ifndef __TCP_PERF_H__
#define __TCP_PERF_H__

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "lwip/igmp.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "soc/rtc_periph.h"
#include "driver/spi_slave.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include <sys/socket.h>
#include "esp_event_loop.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************WIFI SETTINGS**********************/
/*test options*/
// #define EXAMPLE_ESP_WIFI_MODE_AP CONFIG_TCP_PERF_WIFI_MODE_AP
// #define EXAMPLE_ESP_TCP_MODE_SERVER CONFIG_TCP_PERF_SERVER 
// #define EXAMPLE_ESP_TCP_PERF_TX CONFIG_TCP_PERF_TX
// #define EXAMPLE_ESP_TCP_DELAY_INFO CONFIG_TCP_PERF_DELAY_DEBUG

// /*AP info and tcp_server info*/
 #define EXAMPLE_DEFAULT_SSID "example"
 #define EXAMPLE_DEFAULT_PWD "123456789"
 #define EXAMPLE_DEFAULT_PORT 8266
 #define EXAMPLE_DEFAULT_PKTSIZE 1460
#define EXAMPLE_MAX_STA_CONN 1 

#ifdef CONFIG_TCP_PERF_SERVER_IP
#define EXAMPLE_DEFAULT_SERVER_IP CONFIG_TCP_PERF_SERVER_IP
#else
#define EXAMPLE_DEFAULT_SERVER_IP "192.168.4.1"
#endif 
#define EXAMPLE_PACK_BYTE_IS 97 //'a'





#define TCP_SERVER_CLIENT_OPTION false  //true为开启热点并且创建tcp服务器，fasle为连接到指定的路由器并且连接到指定的tcp服务器
#define TAG "LimitTCP-->" //打印的tag

//以下是softAP热点模式的配置信息，即要当服务端的esp32所需要使用的配置信息
#define SOFT_AP_SSID "rc2020"

#define SOFT_AP_PAS "123456789" //如果密码设置为空，则配置的热点是开放的，没有密码的。

#define SOFT_AP_MAX_CONNECT 4 //作为AP热点时候，最大的连接数目


//以下是station模式配置信息,即需要当客户端的esp32所需要使用的配置信息。注意这里要和上面相符
//192.168.4.1是esp32出厂时写死的地址。直接写不用改。
#define GATEWAY_SSID "rc2020"

#define GATEWAY_PAS "123456789"

#define TCP_SERVER_ADRESS "192.168.4.1" //要连接TCP服务器地址


//统一的端口号，包括TCP客户端或者服务端，默认就好
#define TCP_PORT 8266


//下面是freertos的相关配置，看不懂默认就好。
/* FreeRTOS event group to signal when we are connected to wifi*/
extern EventGroupHandle_t tcp_event_group;
#define WIFI_CONNECTED_BIT BIT0

extern int  g_total_data;
extern bool g_rxtx_need_restart;

#if EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO
extern int g_total_pack;
extern int g_send_success;
extern int g_send_fail;
extern int g_delay_classify[5];
#endif/*EXAMPLE_ESP_TCP_PERF_TX && EXAMPLE_ESP_TCP_DELAY_INFO*/

extern int sending;
extern int need_restart;

//using esp as station
void wifi_init_sta();
//using esp as softap
void wifi_init_softap();

//create a tcp server socket. return ESP_OK:success ESP_FAIL:error
esp_err_t create_tcp_server(bool isCreatServer);
//create a tcp client socket. return ESP_OK:success ESP_FAIL:error
esp_err_t create_tcp_client();

//send data task
void send_data(void *pvParameters);
//receive data task
void recv_data(void *pvParameters);

//close all socket
void close_socket();

//get socket error code. return: error code
int get_socket_error_code(int socket);

//show socket error code. return: error code
int show_socket_error_reason(const char* str, int socket);

//check working socket
int check_working_socket();


#ifdef __cplusplus
}
#endif



/*****************************************************/


/******************SPI SETTINGS**********************/
/*
Pins in use. The SPI Master can use the GPIO mux, so feel free to change these if needed.
*/
//这里是spi的设置宏
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define GPIO_HANDSHAKE 2
#define GPIO_MOSI 13
#define GPIO_MISO 12
#define GPIO_SCLK 14
#define GPIO_CS 15

#elif CONFIG_IDF_TARGET_ESP32C3
#define GPIO_HANDSHAKE 3
#define GPIO_MOSI 7
#define GPIO_MISO 2
#define GPIO_SCLK 6
#define GPIO_CS 10

#endif //CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2


#ifdef CONFIG_IDF_TARGET_ESP32
#define RCV_HOST    HSPI_HOST
#define DMA_CHAN    2

#elif defined CONFIG_IDF_TARGET_ESP32S2
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#elif defined CONFIG_IDF_TARGET_ESP32C3
#define RCV_HOST    SPI2_HOST
#define DMA_CHAN    RCV_HOST

#endif



//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans);

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans);
/*************************************************/
#endif /*#ifndef __TCP_PERF_H__*/