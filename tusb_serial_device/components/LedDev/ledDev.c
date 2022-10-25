#include "ledDev.h"

#define BLINK_GPIO               48 /* GPIO 48 for ESP32-S3 built-in addressable LED */
#define BLINK_LED_RMT_CHANNEL    0
#define BLINK_PERIOD             1000

static const char *TAG = "blinkStateStatus";
static uint8_t s_led_state = 0;
static led_strip_t *pStrip_a;

// enum state_codes cur_state;

void led_blink(led_strip_t led, int on_time, int off_time)
{
    pStrip_a->refresh(led, on_time);
    pStrip_a->clear(led, off_time);
}

void led_state(state_codes cur_state)
{
    // ESP_LOGI(TAG, "Indicating state status by blinking LED!");
    // pStrip_a = led_strip_init(BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1); /* LED strip initialization with the GPIO and pixels number*/
    // pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16); /* LED starts with White to indicate power on */
    // vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
    // pStrip_a->clear(pStrip_a, 50);                                          /* Set all LED off to clear all pixels */

    // while (1)
    // {
        if (cur_state == powerUp)                                           //上电：红色
        {
            // if (未连接)
            // {
            //     红闪
            // }
            pStrip_a->set_pixel(pStrip_a, 0, 255, 0, 0);                  
            ESP_LOGI(TAG, "Current state: powerUp");
            //pStrip_a->refresh(pStrip_a, 100);
            led_blink(led, 100, 5);

        }
        else if (cur_state == enabled)                                      //使能：蓝色
        {
            pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);                    
            ESP_LOGI(TAG, "Current state: enabled");
            //pStrip_a->refresh(pStrip_a, 100);
            led_blink(led, 100, 5);

        }
        else if (cur_state == initialising)                                  //初始化：蓝闪
        {
            pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);
            ESP_LOGI(TAG, "Current state: initialising");
            // pStrip_a->refresh(pStrip_a, 50);
            // pStrip_a->clear(pStrip_a, 50);
            led_blink(led, 50, 25);
        }
        else if (cur_state == ready)                                         //就绪：绿色
        {
            pStrip_a->set_pixel(pStrip_a, 0, 0, 255, 0);
            ESP_LOGI(TAG, "Current state: ready");
            //pStrip_a->refresh(pStrip_a, 100);
            led_blink(led, 100, 5);
        }
        else if (cur_state == run)                                          //使用：绿闪
        {
            pStrip_a->set_pixel(pStrip_a, 0, 0, 255, 0);
            ESP_LOGI(TAG, "Current state: run");
            // pStrip_a->refresh(pStrip_a, 50);
            // pStrip_a->clear(pStrip_a, 50);
            led_blink(led, 50, 25);
        }
        else if (cur_state == estop)                                        //estop:红蓝绿闪
        {
            ESP_LOGI(TAG, "Current state: estop");
            pStrip_a->set_pixel(pStrip_a, 0, 255, 0, 0);
            led_blink(led, 20, 10);
            pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);
            led_blink(led, 20, 10);
            pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);
            led_blink(led, 20, 10);    
        }

    //     vTaskDelay(BLINK_PERIOD / portTICK_PERIOD_MS);
    // }
}
