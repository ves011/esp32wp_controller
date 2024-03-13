/*
 * main_screen.c
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
#include "esp_lcd_ili9341.h"
#include <time.h>
#include <sys/time.h>
#include "common_defines.h"
#include "lvgl.h"
#include "lcd.h"

extern QueueHandle_t ui_cmd_q;
extern lv_style_t btn_norm, btn_sel;

static btn_main_t btns[2];
static lv_obj_t * watch;
static int k_act;
static lv_obj_t *main_scr;

/*
static void bpump_event_cb(lv_event_t* e)
    {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* btnl = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
        {
        //lv_log("app bpump click\n");
        }
    }
static void bwater_event_cb(lv_event_t* e)
    {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* btnl = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
        {
        //lv_log("app bwater click\n");
        }
    }
*/

static void draw_main_screen(lv_disp_t *disp, int active_screen)
	{
	lv_obj_t *label;

	gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
	main_scr = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(main_scr, lv_color_hex(0x0), LV_PART_MAIN);

    btns[0].btn = lv_btn_create(main_scr);
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[0].btn, 60, 90);
    lv_obj_set_size(btns[0].btn, 200, 45);
    //lv_obj_add_event_cb(btns[0].btn, bpump_event_cb, LV_EVENT_ALL, NULL);

    label = lv_label_create(btns[0].btn);
    lv_label_set_text(label, "Pompa foraj");
    lv_obj_center(label);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    btns[0].state = 0;
    lv_obj_t * ledp  = lv_led_create(btns[0].btn);
    lv_obj_align(ledp, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(ledp, 255);
    lv_led_set_color(ledp, lv_color_hex(0xc0c0c0));

    btns[1].btn = lv_btn_create(main_scr);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[1].btn, 60, 160);
    lv_obj_set_size(btns[1].btn, 200, 45);

    //lv_obj_add_event_cb(btns[1].btn, bwater_event_cb, LV_EVENT_ALL, NULL);
    label = lv_label_create(btns[1].btn);
    lv_label_set_text(label, "Irigatie");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_t * ledw  = lv_led_create(btns[1].btn);
    lv_obj_align(ledw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(ledw, 255);
    lv_led_set_color(ledw, lv_color_hex(0xc0c0c0));

    if(active_screen == PUMP_SCREEN)
    	{
    	lv_obj_add_state(btns[0].btn, LV_STATE_PRESSED);
    	btns[0].state = 1;
    	}
    if(active_screen == WATER_SCREEN)
    	{
    	lv_obj_add_state(btns[1].btn, LV_STATE_PRESSED);
    	btns[1].state = 1;
    	}

    watch = lv_label_create(main_scr);
    lv_obj_set_pos(watch, 10, 10);
    lv_obj_set_size(watch, 290, 60);
    lv_obj_set_style_text_font(watch, &seg_black_40, 0);
    lv_obj_set_style_text_color(watch, lv_color_hex(0xc0c0c0), 0);

    time_t now = 0;
	char buf[24];
	struct tm timeinfo = { 0 };
	time(&now);
	localtime_r(&now, &timeinfo);
    sprintf(buf, "%02d:%02d - %02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, (timeinfo.tm_mon + 1));
    lv_label_set_text(watch, buf);

    lv_scr_load_anim(main_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);

    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
	}

int do_main_screen(lv_disp_t *disp, int active_screen)
	{
	msg_t msg;
	int i, kesc = 0, ret = 0;
	k_act = 1;
	draw_main_screen(disp, active_screen);
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
						kesc = 1;
						if(i == 0)
							ret = PUMP_SCREEN;
						break;
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
	return ret;
	}
