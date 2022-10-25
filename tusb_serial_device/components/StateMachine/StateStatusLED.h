#ifndef LED_STATE_H_
#define LED_STATE_H_

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"

enum state_codes { powerUp, enabled, initialising, ready, run, estop, wifiConfig, devMatching};
//enum state_codes cur_state;
static const char *TAG;
static uint8_t s_led_state;
static led_strip_t *pStrip_a;

void led_init();
void led_state(enum state_codes cur_state);


#endif /* MAIN_TEST_HEADER_H_ */
