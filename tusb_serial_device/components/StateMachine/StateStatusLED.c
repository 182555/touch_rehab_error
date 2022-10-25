#include "StateStatusLED.h"

#define BLINK_GPIO               48 /* GPIO 48 for ESP32-S3 built-in addressable LED */
#define BLINK_LED_RMT_CHANNEL    0
#define BLINK_PERIOD             1000

static const char *TAG = "blinkStateStatus";
static uint8_t s_led_state = 0;
static led_strip_t *pStrip_a;

void led_init()
{
    pStrip_a = led_strip_init(BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1); /* LED strip initialization with the GPIO and pixels number*/
    // pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
    // pStrip_a->refresh(pStrip_a, 4);
    return;
}
//enum state_codes cur_state = powerUp;
// enum state_codes cur_state;

void led_state(enum state_codes cur_state)
{
    if (cur_state == powerUp)                                           //上电：红色
        {
            // if (未连接)
            // {
            //     红闪
            // }
            pStrip_a->set_pixel(pStrip_a, 0, 255, 0, 0);                  
            //ESP_LOGI(TAG, "Current state: powerUp");
            pStrip_a->refresh(pStrip_a, 40);

        }
        else if (cur_state == enabled)                                      //使能：蓝色
        {
            pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);                    
            //ESP_LOGI(TAG, "Current state: enabled");
            pStrip_a->refresh(pStrip_a, 40);

        }
        else if (cur_state == initialising)                                  //初始化：蓝闪
        {
            pStrip_a->clear(pStrip_a, 40);

            pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);
            //ESP_LOGI(TAG, "Current state: initialising");

            pStrip_a->refresh(pStrip_a, 80);
        }
        else if (cur_state == ready)                                         //就绪：绿色
        {
            pStrip_a->set_pixel(pStrip_a, 0, 0, 255, 0);
            //ESP_LOGI(TAG, "Current state: ready");
            pStrip_a->refresh(pStrip_a, 40);
        }
        else if (cur_state == run)                                          //使用：绿闪
        {
            pStrip_a->clear(pStrip_a, 40);
            pStrip_a->set_pixel(pStrip_a, 0, 0, 255, 0);
            //ESP_LOGI(TAG, "Current state: run");
            pStrip_a->refresh(pStrip_a, 80);
        }
        else if (cur_state == estop)                                        //estop:红蓝绿闪
        {
            pStrip_a->clear(pStrip_a, 40);
            pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
            pStrip_a->refresh(pStrip_a, 80);

            // pStrip_a->clear(pStrip_a, 40);
            // pStrip_a->set_pixel(pStrip_a, 0, 0, 0, 255);
            // pStrip_a->refresh(pStrip_a, 80);

            // pStrip_a->clear(pStrip_a, 40);
            // pStrip_a->set_pixel(pStrip_a, 0, 0, 255, 0);
            // pStrip_a->refresh(pStrip_a, 80);

        }
        return;
}