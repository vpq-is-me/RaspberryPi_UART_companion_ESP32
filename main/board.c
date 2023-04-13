/* board.c - Board-specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 * Additional Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "iot_button.h"

#define TAG "BOARD"

struct _led_state led_state = {LED_OFF, LED_OFF, LED_B, "blue"};

void board_led_operation(uint8_t pin, uint8_t onoff){
    if (led_state.pin == pin)    {
        if (onoff == led_state.previous)        {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state.name, (onoff ? "on" : "off"));
            return;
        }
        gpio_set_level(pin, onoff);
        led_state.previous = onoff;
        return;
    } 
    ESP_LOGE(TAG, "LED is not found!");
}
void board_led_toggle(uint8_t pin){
    if (led_state.pin == pin){
        board_led_operation(pin,!led_state.previous);
    }
}

static void board_led_init(void)
{
    gpio_reset_pin(led_state.pin);
    gpio_set_direction(led_state.pin, GPIO_MODE_OUTPUT);
    gpio_set_level(led_state.pin, LED_OFF);
    led_state.previous = LED_OFF;

}
//****************************************
#define BUTTON_IO_NUM 0
#define BUTTON_ACTIVE_LEVEL 0

extern void print_vnd_data(void);
void vendor_publish_message(void);
void SendMessage2Node(void);

static uint8_t dummy_fg=0;//!!!
static void button_tap_cb(void *arg) {
    //    print_vnd_data();
    // if(dummy_fg)vendor_publish_message();
    // else
     SendMessage2Node();
    dummy_fg=!dummy_fg;
}

static void board_button_init(void) {
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}
//****************************************

void board_init(void) {
    board_led_init();
    board_button_init();
}
