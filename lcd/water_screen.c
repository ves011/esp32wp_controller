/*
 * water_screen.c
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
#include "water_screen.h"

extern lv_style_t btn_norm, btn_sel, btn_press, cell_style, cell_style_left;
static lv_obj_t *water_scr, *watch, *led_act0, *led_act1;
static btn_main_t btns[3];

static int k_act = 0;

static void draw_water_screen()
    {
    time_t now = 0;
    char buf[28];
    struct tm* timeinfo;

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
    sprintf(buf, "%02d:%02d - %02d.%02d.%02d", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_mday, (timeinfo->tm_mon + 1), (timeinfo->tm_year % 100));
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

    lv_obj_set_pos(btns[0].btn, 40, 40);
    lv_obj_set_size(btns[0].btn, 200, 40);

    lv_obj_t* label = lv_label_create(btns[0].btn);

    lv_label_set_text(label, "Zona 1");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_state(btns[0].btn, LV_STATE_FOCUSED);
    btns[0].state = 1;

    led_act0 = lv_led_create(btns[0].btn);
    lv_obj_align(led_act0, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act0, 255);
    lv_led_set_color(led_act0, lv_color_hex(0x606060));

    btns[1].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[1].btn, &btn_press, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[1].btn, 40, 110);
    lv_obj_set_size(btns[1].btn, 200, 40);

    label = lv_label_create(btns[1].btn);

    lv_label_set_text(label, "Zona 2");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    btns[1].state = 0;

    led_act1 = lv_led_create(btns[1].btn);
    lv_obj_align(led_act1, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_act1, 255);
    lv_led_set_color(led_act1, lv_color_hex(0x606060));

    btns[2].btn = lv_btn_create(water_scr);
    lv_obj_add_style(btns[2].btn, &btn_norm, 0);
    lv_obj_add_style(btns[2].btn, &btn_sel, LV_STATE_FOCUSED);
    lv_obj_add_style(btns[2].btn, &btn_press, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[2].btn, 260, 190);
    lv_obj_set_size(btns[2].btn, 45, 25);

    label = lv_label_create(btns[2].btn);
    lv_label_set_text(label, "<<");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    lv_scr_load_anim(water_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
    }

int do_water_screen()
	{
	msg_t msg;
	char buf[10];
	int p_state, p_status, p_current, p_current_lim, p_min_pres, p_max_pres, p_press;
	int i, kesc = 0, nbuttons = 3, ret = 0;
	//saved_pump_state = saved_pump_status = saved_pump_pressure_kpa = -1;
	//saved_pump_current = -5;
	draw_water_screen();
	k_act = 1;
	xQueueReset(ui_cmd_q);
	//msg.source = PUMP_VAL_CHANGE;
	//xQueueSend(ui_cmd_q, &msg, 0);
	while(!kesc)
		{

		lv_obj_add_state(btns[ret].btn, LV_STATE_FOCUSED);
		btns[ret].state = 1;
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
					for(i = 0; i < nbuttons; i++)
						{
						lv_obj_clear_state(btns[i].btn, LV_STATE_FOCUSED);
						btns[i].state = 0;
						}
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
						ret = do_water_screen_z(0);
						draw_water_screen();
						lv_obj_clear_state(btns[1].btn, LV_STATE_PRESSED);
						lv_obj_add_state(btns[0].btn, LV_STATE_PRESSED);
						}
					else if(btns[1].state == 1)
						{
						ret = do_water_screen_z(1);
						draw_water_screen();
						lv_obj_clear_state(btns[0].btn, LV_STATE_PRESSED);
						lv_obj_add_state(btns[1].btn, LV_STATE_PRESSED);
						}
					else if(btns[2].state == 1)
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
	return WATER_SCREEN;
	}

