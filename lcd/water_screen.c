/*
 * water_screen.c
 *
 *  Created on: Mar 10, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/queue.h"
//#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "esp_lcd_ili9341.h"
#include <time.h>
#include <sys/time.h>
#include "project_specific.h"
#include "common_defines.h"
#include "external_defs.h"
#include "lvgl.h"
#include "lcd.h"
#include "pumpop.h"
#include "rot_enc.h"
#include "handle_ui_key.h"
#include "waterop.h"
#include "pump_screen.h"
#include "water_screen_z.h"
#include "water_screen.h"

extern lv_style_t btn_norm, btn_sel, btn_press, cell_style;
static lv_obj_t *water_scr, *watch, *led_act[4];
static btn_main_t btns[5];

static int k_act = 0;

static void draw_water_screen(int active_screen)
    {
    time_t now = 0;
    char buf[28];
    struct tm* timeinfo;
    int dvstate[DVCOUNT];

    lv_style_init(&cell_style);
    lv_style_reset(&cell_style);
    lv_style_set_text_color(&cell_style, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&cell_style, &lv_font_montserrat_14);
    lv_style_set_text_align(&cell_style, LV_TEXT_ALIGN_RIGHT);

    water_scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(water_scr, lv_color_hex(0x0), LV_PART_MAIN);

    watch = lv_label_create(water_scr);
    lv_obj_add_style(watch, &cell_style, 0);
    lv_obj_set_style_text_align(watch, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(watch, 5, 10);
    lv_obj_set_size(watch, 120, 20);
    time(&now);
    timeinfo = localtime(&now);
    sprintf(buf, "%02d:%02d - %02d.%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_mday, (timeinfo->tm_mon + 1));
    lv_label_set_text(watch, buf);

    lv_obj_t* l_title = lv_label_create(water_scr);
    lv_obj_set_style_text_font(l_title, &seg_black_20, 0);
    lv_obj_set_style_text_color(l_title, lv_color_hex(0xc0c0c0), 0);
    lv_obj_set_pos(l_title, 170, 10);
    lv_obj_set_size(l_title, 140, 30);
    lv_label_set_text(l_title, "Irigatie");
    lv_obj_set_style_text_align(l_title, LV_TEXT_ALIGN_RIGHT, 0);

    btns[0].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[0].btn, &btn_press, LV_STATE_PRESSED);
    btns[0].state = 0;

    lv_obj_set_pos(btns[0].btn, 10, 40);
    lv_obj_set_size(btns[0].btn, 130, 40);

    lv_obj_t* label = lv_label_create(btns[0].btn);

    lv_label_set_text(label, "Zona 1");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    //lv_obj_add_state(btns[0].btn, LV_STATE_FOCUSED);

    led_act[0] = lv_led_create(btns[0].btn);
    lv_obj_align(led_act[0], LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act[0], 255);
    lv_led_set_color(led_act[0], LEDOFF);

    btns[1].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[1].btn, &btn_press, LV_STATE_PRESSED);
    btns[1].state = 0;

    lv_obj_set_pos(btns[1].btn, 180, 40);
    lv_obj_set_size(btns[1].btn, 130, 40);

    label = lv_label_create(btns[1].btn);

    lv_label_set_text(label, "Zona 2");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);


    led_act[1] = lv_led_create(btns[1].btn);
    lv_obj_align(led_act[1], LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act[1], 255);
    lv_led_set_color(led_act[1], LEDOFF);

    btns[2].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[2].btn, &btn_norm, 0);
    lv_obj_add_style(btns[2].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[2].btn, &btn_press, LV_STATE_PRESSED);
    btns[2].state = 0;

    lv_obj_set_pos(btns[2].btn, 10, 120);
    lv_obj_set_size(btns[2].btn, 130, 40);

    label = lv_label_create(btns[2].btn);

    lv_label_set_text(label, "Zona 3");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);


    led_act[2] = lv_led_create(btns[2].btn);
    lv_obj_align(led_act[2], LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act[2], 255);
    lv_led_set_color(led_act[2], LEDOFF);

    btns[3].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[3].btn, &btn_norm, 0);
    lv_obj_add_style(btns[3].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[3].btn, &btn_press, LV_STATE_PRESSED);
    btns[3].state = 0;

    lv_obj_set_pos(btns[3].btn, 180, 120);
    lv_obj_set_size(btns[3].btn, 130, 40);

    label = lv_label_create(btns[3].btn);

    lv_label_set_text(label, "Zona 4");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);


    led_act[3] = lv_led_create(btns[3].btn);
    lv_obj_align(led_act[3], LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act[3], 255);
    lv_led_set_color(led_act[3], LEDOFF);


    btns[4].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[4].btn, &btn_norm, 0);
    lv_obj_add_style(btns[4].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[4].btn, &btn_press, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[4].btn, 260, 190);
    lv_obj_set_size(btns[4].btn, 45, 25);
    btns[4].state = 0;

    label = lv_label_create(btns[4].btn);
    lv_label_set_text(label, "<<");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    get_water_dv_state(dvstate);
	for(int i = 0; i < DVCOUNT; i++)
		{
		if(dvstate[i] == DVOPEN)
			lv_led_set_color(led_act[i], LEDON);
		else if(dvstate[i] == DVCLOSE)
			lv_led_set_color(led_act[i], LEDOFF);
		else
			lv_led_set_color(led_act[i], LEDFAULT);
		}

    btns[active_screen].state = 1;
    lv_obj_add_state(btns[active_screen].btn, LV_STATE_FOCUSED);

    lv_scr_load_anim(water_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
    }

int do_water_screen(int active_screen)
	{

	int i, lt = 0, key, key_long, param, active_but = 0, nbuttons = 5;
	int dvstate[DVCOUNT] = {0};
	//saved_pump_state = saved_pump_status = saved_pump_pressure_kpa = -1;
	//saved_pump_current = -5;
	draw_water_screen(active_screen);
	k_act = 1;
	xQueueReset(ui_cmd_q);
	msg_t msg;
	msg.source = WATER_VAL_CHANGE;
	xQueueSend(ui_cmd_q, &msg, 0);
	while(1)
		{
		key_long = handle_ui_key(watch, btns, nbuttons);
		key = key_long & 0xffff;
		param = key_long >> 16;
		//ESP_LOGI("WSCR", "ui key: %x, %d", key, param);
		if(key == KEY_PRESS_SHORT)
			{
			if(btns[4].state == 1)
				break;
			}
		else if(key == KEY_PRESS_LONG)
			{
			for(i = 0; i < nbuttons - 1; i++)
				{
				if(btns[i].state == 1)
					{
					active_but = i;
					msg.val = active_but;
					if(dvstate[i] == DVCLOSE)
						msg.source = DVOPEN;
					else
						msg.source = DVCLOSE;
					xQueueSend(water_cmd_q, &msg, 0);
					lt = 0;
					break;
					}
				}
			}
		else if(key == WATER_DV_OP)
			{
			lt = 1 - lt;
			if(lt)
				lv_led_set_color(led_act[param], LEDON);
			else
				lv_led_set_color(led_act[param], LEDOFF);
			}
		else if(key == WATER_VAL_CHANGE)
			{
			get_water_dv_state(dvstate);
			for(i = 0; i < DVCOUNT; i++)
				{
				if(dvstate[i] == DVOPEN)
					lv_led_set_color(led_act[i], LEDON);
				else if(dvstate[i] == DVCLOSE)
					lv_led_set_color(led_act[i], LEDOFF);
				else
					lv_led_set_color(led_act[i], LEDFAULT);
				}
			}
		}
	return WATER_SCREEN;
	}

