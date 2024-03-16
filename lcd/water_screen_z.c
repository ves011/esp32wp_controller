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
#include "rot_enc.h"
#include "pump_screen.h"
#include "water_screen_z.h"

extern lv_style_t btn_norm, btn_sel, btn_press, cell_style, cell_style_left;
static lv_obj_t *water_scr_0, *led_act, *date_last, *date_next, *wtotal, *watch, *state, *timew;
static btn_main_t btns[2];


static int k_act = 0;

static void draw_water_screen_z(int zone)
    {
    time_t now = 0;
    char buf[28];
    struct tm* timeinfo;
    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
    water_scr_0 = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(water_scr_0, lv_color_hex(0x0), LV_PART_MAIN);

    watch = lv_label_create(water_scr_0);
    lv_obj_add_style(watch, &cell_style, 0);
    lv_obj_set_style_text_align(watch, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_pos(watch, 5, 10);
    lv_obj_set_size(watch, 120, 20);
    time(&now);
    timeinfo = localtime(&now);
    sprintf(buf, "%02d:%02d - %02d.%02d.%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_mday, (timeinfo->tm_mon + 1), (timeinfo->tm_year % 100));
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
    lv_led_set_color(led_act, lv_color_hex(0x606060));

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
    lv_label_set_text(date_last, "22.10.24 - 08.25");

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_pos(label, 45, 110);
    lv_obj_set_size(label, 100, 20);
    lv_label_set_text(label, "- stare:");

    state = lv_label_create(water_scr_0);
    lv_obj_add_style(state, &cell_style_left, 0);
    lv_obj_set_pos(state, 150, 110);
    lv_obj_set_size(state, 150, 20);
    lv_label_set_text(state, "eroare pompa");

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_pos(label, 45, 130);
    lv_obj_set_size(label, 100, 20);
    lv_label_set_text(label, "- durata:");

    timew = lv_label_create(water_scr_0);
    lv_obj_add_style(timew, &cell_style_left, 0);
    lv_obj_set_pos(timew, 150, 130);
    lv_obj_set_size(timew, 100, 20);
    lv_label_set_text(timew, "30 min");

    label = lv_label_create(water_scr_0);
    lv_obj_add_style(label, &cell_style, 0);
    lv_obj_set_pos(label, 45, 150);
    lv_obj_set_size(label, 100, 20);
    lv_label_set_text(label, "- cant. apa:");

    wtotal = lv_label_create(water_scr_0);
    lv_obj_add_style(wtotal, &cell_style_left, 0);
    lv_obj_set_pos(wtotal, 150, 150);
    lv_obj_set_size(wtotal, 100, 20);
    lv_label_set_text(wtotal, "250 l");


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
    lv_label_set_text(date_next, "23.10.24 - 08.25");

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
	char buf[10];
	int p_state, p_status, p_current, p_current_lim, p_min_pres, p_max_pres, p_press;
	int i, kesc = 0, nbuttons = 2;
	//saved_pump_state = saved_pump_status = saved_pump_pressure_kpa = -1;
	//saved_pump_current = -5;
	draw_water_screen_z(zone);
	k_act = 1;
	xQueueReset(ui_cmd_q);
	msg.source = PUMP_VAL_CHANGE;
	xQueueSend(ui_cmd_q, &msg, 0);
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
					for (i = 0; i < nbuttons; i++)
						{
						if (btns[i].state)
							{
							lv_obj_clear_state(btns[i].btn, LV_STATE_FOCUSED);
							btns[i].state = 0;
							i++;
							i %= nbuttons;
							lv_obj_add_state(btns[i].btn, LV_STATE_FOCUSED);
							btns[i].state = 1;
							break;
							}
						}
					if (i == nbuttons)
						{
						lv_obj_add_state(btns[0].btn, LV_STATE_FOCUSED);
						btns[0].state = 1;
						}
					}
				else if(msg.val == K_ROT_RIGHT)
					{
					for (i = nbuttons - 1; i >= 0; i--)
						{
						//bs = lv_obj_get_state(btns[i].btn);
						if (btns[i].state)
							{
							lv_obj_clear_state(btns[i].btn, LV_STATE_FOCUSED);
							btns[i].state = 0;
							i--;
							if(i < 0)
								i = nbuttons - 1;
							lv_obj_add_state(btns[i].btn, LV_STATE_FOCUSED);
							btns[i].state = 1;
							break;
							}
						}
					if (i < 0)
						{
						lv_obj_add_state(btns[nbuttons - 1].btn, LV_STATE_FOCUSED);
						btns[nbuttons - 1].state = 1;
						}
					}
				}
			if(msg.source == K_DOWN && k_act)
				{
				for(int i = 0; i < nbuttons; i++)
					{
					if(btns[i].state == 1)
						{
						lv_obj_add_state(btns[i].btn, LV_STATE_PRESSED);
						break;
						}
					}
				}
			if(msg.source == K_UP && k_act)
				{
				for(int i = 0; i < nbuttons; i++)
					{
					if(btns[i].state == 1)
						{
						lv_obj_clear_state(btns[i].btn, LV_STATE_PRESSED);
						break;
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
				if(msg.val == PUSH_TIME_SHORT)
					{
					if(btns[0].state == 1)
						{

						}
					else if(btns[1].state == 1)
						{
						kesc = 1;
						break;
						}
					}
				}
			if(msg.source == INACT_TIME)
				{
				time_t now = 0;
				char buf[28];
				struct tm timeinfo = { 0 };
				time(&now);
				localtime_r(&now, &timeinfo);
				sprintf(buf, "%02d:%02d - %02d.%02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, timeinfo.tm_mon + 1, (timeinfo.tm_year % 100));
				lv_label_set_text(watch, buf);

				if(k_act == 0)
					{
					gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
					}
				k_act = 0;
				}
			}
		}
	return zone;
	}

