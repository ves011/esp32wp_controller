/*
 * main_screen.c
 *
 *  Created on: Mar 10, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "esp_lcd_ili9341.h"
#include <time.h>
#include <sys/time.h>
#include "project_specific.h"
#include "common_defines.h"
#include "external_defs.h"
#include "common_defines.h"
#include "lvgl.h"
#include "pumpop.h"
#include "waterop.h"
#include "lcd.h"
#include "rot_enc.h"
#include "handle_ui_key.h"
#include "pump_screen.h"
#include "water_screen.h"
#include "main_screen.h"

extern lv_style_t btn_norm, btn_sel, btn_press, cell_style, cell_style_left;

static btn_main_t btns[2];
static lv_obj_t *watch, *ledp, *ledw;
static int k_act;
static lv_obj_t *main_scr;

static void draw_main_screen(int active_screen)
	{
	lv_obj_t *label;

	gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
	main_scr = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(main_scr, lv_color_hex(0x0), LV_PART_MAIN);

    btns[0].btn = lv_btn_create(main_scr);
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[0].btn, &btn_press, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[0].btn, 60, 90);
    lv_obj_set_size(btns[0].btn, 200, 45);
    //lv_obj_add_event_cb(btns[0].btn, bpump_event_cb, LV_EVENT_ALL, NULL);

    label = lv_label_create(btns[0].btn);
    lv_label_set_text(label, "Pompa foraj");
    lv_obj_center(label);
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    btns[0].state = 0;
    ledp  = lv_led_create(btns[0].btn);
    lv_obj_align(ledp, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(ledp, 255);
    lv_led_set_color(ledp, lv_color_hex(0x606060));

    btns[1].btn = lv_btn_create(main_scr);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[1].btn, &btn_press, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[1].btn, 60, 160);
    lv_obj_set_size(btns[1].btn, 200, 45);

    //lv_obj_add_event_cb(btns[1].btn, bwater_event_cb, LV_EVENT_ALL, NULL);
    label = lv_label_create(btns[1].btn);
    lv_label_set_text(label, "Irigatie");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    btns[1].state = 0;
    ledw  = lv_led_create(btns[1].btn);
    lv_obj_align(ledw, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(ledw, 255);
    lv_led_set_color(ledw, lv_color_hex(0x606060));

    if(active_screen == PUMP_SCREEN)
    	{
    	lv_obj_add_state(btns[0].btn, LV_STATE_FOCUSED);
    	btns[0].state = 1;
    	}
    if(active_screen == WATER_SCREEN)
    	{
    	lv_obj_add_state(btns[1].btn, LV_STATE_FOCUSED);
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

    //lv_scr_load_anim(main_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
    lv_scr_load_anim(main_scr, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);

    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
	}

int do_main_screen(int active_screen)
	{
	msg_t msg;
	int dvstate[DVCOUNT] = {0};
	int i, ret = 0, nbuttons = 2, lt = 0;
	int p_state, p_status, p_current, p_current_lim, p_min_pres, p_max_pres, p_press, p_debit;
	k_act = 1;
	draw_main_screen(active_screen);
	xQueueReset(ui_cmd_q);
	msg.source = PUMP_VAL_CHANGE;
	xQueueSend(ui_cmd_q, &msg, 0);
	msg.source = WATER_VAL_CHANGE;
	xQueueSend(ui_cmd_q, &msg, 0);
	while(1)
		{
		i = handle_ui_key(watch, btns, nbuttons);
		i &= 0xffff;
		if(i == KEY_PRESS_SHORT)
			{
			for(i = 0; i < nbuttons; i++)
				{
				if(btns[i].state == 1)
					{
					lv_obj_add_state(btns[i].btn, LV_STATE_PRESSED);
					if(i == 0)
						{
						ret = PUMP_SCREEN;
						do_pump_screen();
						draw_main_screen(PUMP_SCREEN);
						xQueueReset(ui_cmd_q);
						msg.source = PUMP_VAL_CHANGE;
						xQueueSend(ui_cmd_q, &msg, 0);
						msg.source = WATER_VAL_CHANGE;
						xQueueSend(ui_cmd_q, &msg, 0);
						}
					else if(i == 1)
						{
						ret = WATER_SCREEN;
						do_water_screen(0);
						draw_main_screen(WATER_SCREEN);
						xQueueReset(ui_cmd_q);
						msg.source = PUMP_VAL_CHANGE;
						xQueueSend(ui_cmd_q, &msg, 0);
						msg.source = WATER_VAL_CHANGE;
						xQueueSend(ui_cmd_q, &msg, 0);
						}
					break;
					}
				}
			}

		else if(i == PUMP_VAL_CHANGE)
			{
			get_pump_values(&p_state, &p_status, &p_current, &p_current_lim, &p_min_pres, &p_max_pres, &p_press, &p_debit);
			if(p_status == PUMP_FAULT)
				lv_led_set_color(ledp, lv_color_hex(0xff4040));
			else
				{
				if(p_state == PUMP_ON)
					lv_led_set_color(ledp, lv_color_hex(0x40ff40));
				else
					{
					if(p_status == PUMP_ONLINE)
						lv_led_set_color(ledp, lv_color_hex(0xffff00));
					else if(p_status == PUMP_OFFLINE)
						lv_led_set_color(ledp, lv_color_hex(0x606060));
					}
				}
			}
		else if(i == WATER_VAL_CHANGE)
			{
			int j;
			get_water_dv_state(dvstate);
			for(j = 0; j < DVCOUNT; j++)
				{
				if(dvstate[j] == DVOPEN)
					break;
				}
			if(j < DVCOUNT)
				lv_led_set_color(ledw, LEDON);
			else
				lv_led_set_color(ledw, LEDOFF);
			}
		else if(i == WATER_DV_OP)
			{
			lt = !lt;
			if(lt)
				lv_led_set_color(ledw, LEDON);
			else
				lv_led_set_color(ledw, LEDOFF);
			}
		}
	return ret;
	}
