/*
 * wifiConnection.h
 *
 *  Created on: 2022.5.29
 *      Author: BC
 * This script is to implement the wifi connection functionalities of the touch rehab robots. 
 * 
 */


#ifndef WIFI_CONNECTION_H_
#define WIFI_CONNECTION_H_

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#endif /* WIFI_CONNECTION_H_ */


/* The examples use WiFi configuration that you can set via project configuration menu
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      "Redmi_mmzlab"
#define EXAMPLE_ESP_WIFI_PASS      "mmzlab123897987"
#define EXAMPLE_ESP_MAXIMUM_RETRY  10


// static void initialise_wifi(void);       

void wifi_init_sta(void);

static void smartconfig_example_task(void * parm);


void start_smart_config(void); 

// #if CONFIG_ESP_WIFI_AUTH_OPEN
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
// #elif CONFIG_ESP_WIFI_AUTH_WEP
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
// #elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
// #elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
// #define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK







