#ifndef LED_DEV_H
#define LED_DEV_H

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "stateMachine.h"

static const char *TAG;
static uint8_t s_led_state;
static led_strip_t *pStrip_a;

void led_blink(led_strip_t led, int on_time, int off_time);

void led_state(state_codes cur_state);


#endif /* MAIN_TEST_HEADER_H_ */
