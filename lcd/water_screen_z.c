/*
 * water_screen_0.c
 *
 *  Created on: Mar 16, 2024
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
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "esp_lcd_ili9341.h"
#include <time.h>
#include <sys/time.h>
#include "common_defines.h"
#include "external_defs.h"
#include "lvgl.h"
#include "lcd.h"
#include "pumpop.h"
#include "waterop.h"
#include "rot_enc.h"
#include "pump_screen.h"

#include "handle_ui_key.h"
#include "water_screen_z.h"

extern lv_style_t btn_norm, btn_sel, btn_press, cell_style, cell_style_left;
static lv_obj_t *water_scr_0, *led_act, *date_last, *date_next, *wtotal, *watch, *state, *timew;
static btn_main_t btns[2];


static int k_act = 0;
//static last_status2_t last_status[DVCOUNT];
static dvprogram_t dv_program;

extern int wpday;


static void draw_water_screen_z(int zone)
    {
    time_t now = 0;
    char buf[28];
    struct tm timeinfo = {0};
    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
    water_scr_0 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(water_scr_0, lv_color_hex(0x0), LV_PART_MAIN);

    watch = lv_label_create(water_scr_0);
    lv_obj_add_style(watch, &cell_style, 0);
    lv_obj_set_style_text_align(watch, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(watch, 5, 10);
    lv_obj_set_size(watch, 120, 20);
    time(&now);
    localtime_r(&now, &timeinfo);
    sprintf(buf, "%02d:%02d - %02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, (timeinfo.tm_mon + 1));
    lv_label_set_text(watch, buf);

    lv_obj_t* l_title = lv_label_create(water_scr_0);
    lv_obj_set_style_text_font(l_title, &seg_black_20, 0);
    lv_obj_set_style_text_color(l_title, lv_color_hex(0xc0c0c0), 0);
    lv_obj_set_pos(l_title, 130, 10);
    lv_obj_set_size(l_title, 180, 30);
    lv_label_set_text_fmt(l_title, "Irigatie - Zona %1d", zone + 1);
    lv_obj_set_style_text_align(l_title, LV_TEXT_ALIGN_RIGHT, 0);

    btns[0].btn = lv_btn_create(water_scr_0);
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[0].btn, &btn_press, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[0].btn, 40, 30);
    lv_obj_set_size(btns[0].btn, 200, 40);

    lv_obj_t* label = lv_label_create(btns[0].btn);

    lv_label_set_text(label, "Robinet");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_state(btns[0].btn, LV_STATE_FOCUSED);
    btns[0].state = 1;

    led_act = lv_led_create(btns[0].btn);
    lv_obj_align(led_act, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act, 255);
    lv_led_set_color(led_act, LEDOFF);

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_style_text_font(label, &seg_black_14, 0);
    lv_obj_set_pos(label, 5, 90);
    lv_obj_set_size(label, 140, 20);
    lv_label_set_text(label, "Ultima udare:");

    date_last = lv_label_create(water_scr_0);
    lv_obj_add_style(date_last, &cell_style_left, 0);
    lv_obj_set_style_text_font(date_last, &seg_black_14, 0);
    lv_obj_set_pos(date_last, 150, 90);
    lv_obj_set_size(date_last, 140, 20);
    lv_label_set_text(date_last, "");

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_pos(label, 45, 110);
    lv_obj_set_size(label, 100, 20);
    lv_label_set_text(label, "- stare:");

    state = lv_label_create(water_scr_0);
    lv_obj_add_style(state, &cell_style_left, 0);
    lv_obj_set_pos(state, 150, 110);
    lv_obj_set_size(state, 150, 20);
    lv_label_set_text(state, "");

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_pos(label, 45, 130);
    lv_obj_set_size(label, 100, 20);
    lv_label_set_text(label, "- durata:");

    timew = lv_label_create(water_scr_0);
    lv_obj_add_style(timew, &cell_style_left, 0);
    lv_obj_set_pos(timew, 150, 130);
    lv_obj_set_size(timew, 100, 20);
    lv_label_set_text(timew, "");

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_pos(label, 45, 150);
    lv_obj_set_size(label, 100, 20);
    lv_label_set_text(label, "- cant. apa:");

    wtotal = lv_label_create(water_scr_0);
    lv_obj_add_style(wtotal, &cell_style_left, 0);
    lv_obj_set_pos(wtotal, 150, 150);
    lv_obj_set_size(wtotal, 100, 20);
    lv_label_set_text(wtotal, "");


    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_style_text_font(label, &seg_black_14, 0);
    lv_obj_set_pos(label, 5, 175);
    lv_obj_set_size(label, 140, 20);
    lv_label_set_text(label, "Urmatoarea udare:");

    date_next = lv_label_create(water_scr_0);
    lv_obj_add_style(date_next, &cell_style_left, 0);
    lv_obj_set_style_text_font(date_next, &seg_black_14, 0);
    lv_obj_set_pos(date_next, 150, 175);
    lv_obj_set_size(date_next, 140, 20);
    lv_label_set_text(date_next, "");

    btns[1].btn = lv_btn_create(water_scr_0);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[1].btn, &btn_press, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[1].btn, 260, 195);
    lv_obj_set_size(btns[1].btn, 45, 25);

    label = lv_label_create(btns[1].btn);
    lv_label_set_text(label, "<<");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    btns[1].state = 0;

    lv_scr_load_anim(water_scr_0, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
    }

int do_water_screen_z(int zone)
	{
	msg_t msg;
	char buf[40];
	int w_status, qwater, starth = -1, startm = -1, lt = 0;
	int saved_w_status, saved_qwater, ret;
	int dvstate[DVCOUNT];
	//int p_state, p_status, p_current, p_current_lim, p_min_pres, p_max_pres, p_press;
	int i, key, j, kesc = 0, nbuttons = 2;
	//saved_pump_state = saved_pump_status = saved_pump_pressure_kpa = -1;
	//saved_pump_current = -5;
	draw_water_screen_z(zone);
	k_act = 1;
	xQueueReset(ui_cmd_q);
	//msg.source = PUMP_VAL_CHANGE;
	//xQueueSend(ui_cmd_q, &msg, 0);
	msg.source = WATER_VAL_CHANGE;
	xQueueSend(ui_cmd_q, &msg, 0);
	while(1)
		{
		key = handle_ui_key(watch, btns, nbuttons);
		if(key == KEY_PRESS_SHORT)
			{
			if(btns[1].state == 1)
				break;
			}
		if(key == KEY_PRESS_LONG)
			{
			if(btns[0].state == 1)
				{
				msg.val = zone;
				if(dvstate[zone] == DVCLOSE)
					msg.source = DVOPEN;
				else
					msg.source = DVCLOSE;
				xQueueSend(water_cmd_q, &msg, 0);
				lt = 0;
				}
			}
		if(key == WATER_DV_OP)
			{
			lt = 1 - lt;
			if(lt)
				lv_led_set_color(led_act, LEDON);
			else
				lv_led_set_color(led_act, LEDOFF);
			}
		if(key == WATER_DV_CHANGE)
			{
			get_water_dv_state(dvstate);
			if(dvstate[zone] == DVOPEN)
				lv_led_set_color(led_act, LEDON);
			else if(dvstate[zone] == DVCLOSE)
				lv_led_set_color(led_act, LEDOFF);
			else
				lv_led_set_color(led_act, LEDFAULT);
			}
		if(key == WATER_VAL_CHANGE)
			{
			struct tm timeinfo = { 0 };
			int tact, idx, start[2];
			//last_no_status_t *last_no;
			time_t now = 0;
			get_water_values(&dv_program, dvstate);
			if(dvstate[zone] == DVOPEN)
				lv_led_set_color(led_act, LEDON);
			else if(dvstate[zone] == DVCLOSE)
				lv_led_set_color(led_act, LEDOFF);
			else
				lv_led_set_color(led_act, LEDFAULT);


// next watering

			lv_label_set_text(date_next, "N/A");
			int k;
			for(j = 0; j < DVCOUNT; j++) //look for next program to start
				{
				if(dv_program.p[j].dv == zone)
					{
// next watering program
					start[0] = start[1] = -1;
					time(&now);
					localtime_r(&now, &timeinfo);
					tact = timeinfo.tm_hour * 60 + timeinfo.tm_min;
					for(k = 0; k < wpday; k++)
						start[k] = dv_program.p[j].w_prog[k].starth * 60 + dv_program.p[j].w_prog[k].startm ;

					if(start[0] < start[1])
						{
						if(tact < start[0] || tact > start[1]) idx = 0;
						else idx = 1;
						}
					else
						{
						if(tact < start[1] || tact > start[0]) idx = 1;
						else idx = 0;
						}
					if(tact > start[idx])
						{
						now += 86400;
						localtime_r(&now, &timeinfo);
						}
					lv_label_set_text_fmt(date_next, "%02d.%02d.%02d - %02d:%02d", timeinfo.tm_mday, timeinfo.tm_mon + 1, (timeinfo.tm_year % 100),
					    															dv_program.p[j].w_prog[idx].starth, dv_program.p[j].w_prog[idx].startm);
					// last watering with status
					k = 0;
					if(wpday > 1)
						{
						for(k = 0; k < wpday - 1; k++)
							{
							if(dv_program.p[j].w_prog[k].eff_start.tm_hour * 60 + dv_program.p[j].w_prog[k].eff_start.tm_min >=
								dv_program.p[j].w_prog[k + 1].eff_start.tm_hour * 60 + dv_program.p[j].w_prog[k + 1].eff_start.tm_min)
							break;
							}
						}
					lv_label_set_text_fmt(date_last, "%02d.%02d.%02d - %02d:%02d",
							dv_program.p[j].w_prog[k].eff_start.tm_year % 100,
							dv_program.p[j].w_prog[k].eff_start.tm_mon + 1,
							dv_program.p[j].w_prog[k].eff_start.tm_mday,
							dv_program.p[j].w_prog[k].eff_start.tm_hour,
							dv_program.p[j].w_prog[k].eff_start.tm_min);
					if(dv_program.p[j].w_prog[k].cs != -1)
						{
						if(dv_program.p[j].w_prog[k].cs == COMPLETED)
							{
							lv_label_set_text(state, "OK");
							}
						else if(dv_program.p[j].w_prog[k].cs == IN_PROGRESS)
							{
							lv_label_set_text(state, "In progress");
							}
						else
							{
							char buferr[10];
							switch(dv_program.p[j].w_prog[k].cs)
								{
								case ABORTED: strcpy(buf, "ABORTED: ");break;
								case START_ERROR: strcpy(buf, "START ERROR: ");break;
								case STOP_ERROR: strcpy(buf, "STOP_ERROR: ");break;
								default: strcpy(buf, "invalid: ");break;
								}
							sprintf(buferr, "(%d)", dv_program.p[j].w_prog[k].fault);
							strcat(buf, buferr);
							lv_label_set_text(state, buf);
							}
						}
					break;
					}
				}
			}
		}
	return zone;
	}

