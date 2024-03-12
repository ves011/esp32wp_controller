/*
 * pump_screen.c
 *
 *  Created on: Mar 10, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/freertos.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_ili9341.h"
#include <time.h>
#include <sys/time.h>
#include "common_defines.h"
#include "lvgl.h"
#include "lcd.h"
#include "pump_screen.h"

extern QueueHandle_t ui_cmd_q;
static lv_style_t btn_norm, btn_sel;
extern SemaphoreHandle_t lvgl_mutex;
extern int lv_timer_stop;
static btn_main_t btns[2];
static lv_obj_t * watch;
static lv_style_t watch_style;
static int k_act = 0;

static void draw_pump_screen()
	{
	lv_obj_t *label;
	time_t now = 0;
	char buf[28];
	struct tm timeinfo = { 0 };
	gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    lv_style_init(&watch_style);
    lv_style_set_text_color(&watch_style, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&watch_style, &lv_font_montserrat_14);
    watch = lv_label_create(lv_scr_act());
    lv_obj_set_pos(watch, 10, 10);
    lv_obj_set_size(watch, 290, 20);
    lv_obj_add_style(watch, &watch_style, 0);
    time(&now);
	localtime_r(&now, &timeinfo);
    sprintf(buf, "%02d:%02d - %02d.%02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, (timeinfo.tm_mon + 1), (timeinfo.tm_year %100));
    lv_label_set_text(watch, buf);

    lv_style_init(&btn_sel);
    lv_style_init(&btn_norm);

    lv_style_set_text_color(&btn_norm, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&btn_norm, &seg_black_20);

    lv_style_set_translate_y(&btn_sel, 5);
    lv_style_set_shadow_ofs_y(&btn_sel, 10);
    lv_style_set_shadow_ofs_x(&btn_sel, 10);
    lv_style_set_text_color(&btn_sel, lv_color_hex(0xffff00));
    lv_style_set_text_font(&btn_sel, &seg_black_20);

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = { LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0 };
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);
    lv_style_set_transition(&btn_sel, &trans);


    btns[0].btn = lv_btn_create(lv_scr_act());
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[0].btn, 40, 90);
    lv_obj_set_size(btns[0].btn, 200, 45);

    label = lv_label_create(btns[0].btn);
    lv_label_set_text(label, "ONLINE");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    btns[0].state = 1;
    lv_obj_t * led_online  = lv_led_create(btns[0].btn);
    lv_obj_align(led_online, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_online, 255);
    lv_led_set_color(led_online, lv_color_hex(0xc0c0c0));

    btns[1].btn = lv_btn_create(lv_scr_act());
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[1].btn, 270, 160);
    lv_obj_set_size(btns[1].btn, 45, 45);

    label = lv_label_create(btns[1].btn);
    lv_label_set_text(label, "S");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t * ledw  = lv_led_create(btns[1].btn);
    lv_obj_align(ledw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(ledw, 255);
    lv_led_set_color(ledw, lv_color_hex(0xc0c0c0));
    btns[1].state = 0;

    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
	}

int do_pump_screen()
	{
	msg_t msg;
	int i, kesc = 0;
	draw_pump_screen();
	k_act = 1;
	xQueueReset(ui_cmd_q);
	while(!kesc)
		{
		if(xQueueReceive(ui_cmd_q, &msg, portMAX_DELAY))
			{
			if (msg.source == K_ROT) //rot left or right
				{
				if(k_act == 0)
					{
					k_act = 1;
					gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
					continue;
					}
				if(msg.val == K_ROT_LEFT)
					{
					for (i = 0; i < 2; i++)
						{
						//bs = lv_obj_get_state(btns[i].btn);
						if (btns[i].state)
							{
							lv_obj_clear_state(btns[i].btn, LV_STATE_PRESSED);
							btns[i].state = 0;
							i++;
							i %= 2;
							lv_obj_add_state(btns[i].btn, LV_STATE_PRESSED);
							btns[i].state = 1;
							break;
							}
						}
					if (i == 2)
						{
						lv_obj_add_state(btns[0].btn, LV_STATE_PRESSED);
						btns[0].state = 1;
						}
					}
				else if(msg.val == K_ROT_RIGHT)
					{
					for (i = 1; i >= 0; i--)
						{
						//bs = lv_obj_get_state(btns[i].btn);
						if (btns[i].state)
							{
							lv_obj_clear_state(btns[i].btn, LV_STATE_PRESSED);
							btns[i].state = 0;
							i--;
							if(i < 0)
								i = 1;
							lv_obj_add_state(btns[i].btn, LV_STATE_PRESSED);
							btns[i].state = 1;
							break;
							}
						}
					if (i < 0)
						{
						lv_obj_add_state(btns[1].btn, LV_STATE_PRESSED);
						btns[1].state = 1;
						}
					}
				}
			if (msg.source == K_PRESS)
				{
				if(k_act == 0)
					{
					k_act = 1;
					gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
					continue;
					}
				for(i = 0; i < 2; i++)
					{
					if(btns[i].state == 1)
						{
						if(i == 1)
							{
							kesc = 1;
							break;
							}
						}
					}
				}
			if(msg.source == INACT_TIME)
				{
				time_t now = 0;
				char buf[24];
				struct tm timeinfo = { 0 };
				time(&now);
				localtime_r(&now, &timeinfo);
				sprintf(buf, "%02d:%02d - %02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, timeinfo.tm_mon + 1);
				lv_label_set_text(watch, buf);
				/*
				if(k_act == 0)
					{
					gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
					}
				k_act = 0;
				*/
				}
			}
		}
	if(xSemaphoreTake(lvgl_mutex, ( TickType_t ) 100 )) // 1 sec wait
		{
		lv_obj_clean(lv_scr_act());
		xSemaphoreGive(lvgl_mutex);
		}

	return PUMP_SCREEN;
	}
