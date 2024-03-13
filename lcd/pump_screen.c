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
#include "pumpop.h"
#include "rot_enc.h"
#include "pump_screen.h"

extern QueueHandle_t ui_cmd_q;
extern lv_style_t btn_norm, btn_sel;

static btn_main_t btns[2];
static lv_obj_t * watch;

static int k_act = 0;
static lv_obj_t *pump_scr;
static lv_obj_t *led_online, *led_run, *l_press_min, *cpress, *l_pressm;
static lv_obj_t *ccurrent, *l_current_max, *cdebit, *l_pump_state;
static lv_style_t cell_style;
static void draw_pump_screen()
	{
	time_t now = 0;
	char buf[28];
	struct tm timeinfo = { 0 };
	gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

	pump_scr = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(pump_scr, lv_color_hex(0x0), LV_PART_MAIN);

	lv_style_init(&cell_style);
    lv_style_reset(&cell_style);
    lv_style_set_text_color(&cell_style, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&cell_style, &lv_font_montserrat_14);
    lv_style_set_text_align(&cell_style, LV_TEXT_ALIGN_RIGHT);

    watch = lv_label_create(pump_scr);
    lv_obj_set_style_text_font(watch, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(watch, lv_color_hex(0xc0c0c0), 0);
    lv_obj_set_pos(watch, 10, 10);
    lv_obj_set_size(watch, 290, 20);
    //lv_obj_add_style(watch, &watch_style, 0);
    time(&now);
	localtime_r(&now, &timeinfo);
    sprintf(buf, "%02d:%02d - %02d.%02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, (timeinfo.tm_mon + 1), (timeinfo.tm_year %100));
    lv_label_set_text(watch, buf);

    lv_obj_t *l_title = lv_label_create(pump_scr);
    lv_obj_set_style_text_font(l_title, &seg_black_20, 0);
    lv_obj_set_style_text_color(l_title, lv_color_hex(0xc0c0c0), 0);
    lv_obj_set_pos(l_title, 170, 10);
    lv_obj_set_size(l_title, 140, 30);
    lv_label_set_text(l_title, "Pompa foraj");
    lv_obj_set_style_text_align(l_title, LV_TEXT_ALIGN_RIGHT, 0);

    btns[0].btn = lv_btn_create(pump_scr);
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[0].btn, 40, 50);
    lv_obj_set_size(btns[0].btn, 200, 35);

    lv_obj_t * label = lv_label_create(btns[0].btn);
    lv_label_set_text(label, "ONLINE");
    lv_obj_align(label, LV_ALIGN_LEFT_MID, 0, 0);
    //lv_obj_add_state(btns[0].btn, LV_STATE_PRESSED);
    btns[0].state = 0;
    led_online  = lv_led_create(btns[0].btn);
    lv_obj_align(led_online, LV_ALIGN_LEFT_MID, 100, 0);
    lv_led_set_brightness(led_online, 255);
    lv_led_set_color(led_online, lv_color_hex(0xc0c0c0));

    led_run  = lv_led_create(btns[0].btn);
    lv_obj_align(led_run, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_led_set_brightness(led_run, 255);
    lv_led_set_color(led_run, lv_color_hex(0xc0c0c0));

    lv_obj_t* l_press = lv_label_create(pump_scr);
    lv_obj_add_style(l_press, &cell_style, 0);
    lv_obj_set_pos(l_press, 20, 110);
    lv_obj_set_size(l_press, 70, 20);
    lv_label_set_text(l_press, "Presiune:");

    l_press_min = lv_label_create(pump_scr);
    lv_obj_add_style(l_press_min, &cell_style, 0);
    lv_obj_set_pos(l_press_min, 95, 110);
    lv_obj_set_size(l_press_min, 50, 20);
    lv_label_set_text(l_press_min, "255");


    cpress = lv_label_create(pump_scr);
    lv_obj_add_style(cpress, &cell_style, 0);
    lv_obj_set_pos(cpress, 150, 110);
    lv_obj_set_size(cpress, 50, 20);
    lv_label_set_text(cpress, "380");

    l_pressm = lv_label_create(pump_scr);
    lv_obj_add_style(l_pressm, &cell_style, 0);
    lv_obj_set_pos(l_pressm, 205, 110);
    lv_obj_set_size(l_pressm, 50, 20);
    lv_label_set_text(l_pressm, "275");

    lv_obj_t* l_pum = lv_label_create(pump_scr);
    lv_obj_add_style(l_pum, &cell_style, 0);
    lv_obj_set_pos(l_pum, 260, 110);
    lv_obj_set_size(l_pum, 40, 20);
    lv_obj_set_style_text_align(l_pum, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(l_pum, "kPa");


    lv_obj_t* l_current = lv_label_create(pump_scr);
    lv_obj_add_style(l_current, &cell_style, 0);
    lv_obj_set_pos(l_current, 20, 135);
    lv_obj_set_size(l_current, 70, 20);
    lv_label_set_text(l_current, "Curent:");

    lv_obj_t* l_current_min = lv_label_create(pump_scr);
    lv_obj_add_style(l_current_min, &cell_style, 0);
    lv_obj_set_pos(l_current_min, 95, 135);
    lv_obj_set_size(l_current_min, 50, 20);
    lv_label_set_text(l_current_min, "- ");

    ccurrent = lv_label_create(pump_scr);
    lv_obj_add_style(ccurrent, &cell_style, 0);
    lv_obj_set_pos(ccurrent, 150, 135);
    lv_obj_set_size(ccurrent, 50, 20);
    lv_label_set_text(ccurrent, "3.5");

    l_current_max = lv_label_create(pump_scr);
    lv_obj_add_style(l_current_max, &cell_style, 0);
    lv_obj_set_pos(l_current_max, 205, 135);
    lv_obj_set_size(l_current_max, 50, 20);
    lv_label_set_text(l_current_max, "5.0");

    lv_obj_t* l_cum = lv_label_create(pump_scr);
    lv_obj_add_style(l_cum, &cell_style, 0);
    lv_obj_set_pos(l_cum, 260, 135);
    lv_obj_set_size(l_cum, 40, 20);
    lv_obj_set_style_text_align(l_cum, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(l_cum, "A");

    lv_obj_t* l_debit = lv_label_create(pump_scr);
    lv_obj_add_style(l_debit, &cell_style, 0);
    lv_obj_set_pos(l_debit, 20, 160);
    lv_obj_set_size(l_debit, 70, 20);
    lv_label_set_text(l_debit, "Debit:");

    cdebit = lv_label_create(pump_scr);
    lv_obj_add_style(cdebit, &cell_style, 0);
    lv_obj_set_pos(cdebit, 150, 160);
    lv_obj_set_size(cdebit, 50, 20);
    lv_label_set_text(cdebit, "48");

    lv_obj_t* l_dum = lv_label_create(pump_scr);
    lv_obj_add_style(l_dum, &cell_style, 0);
    lv_obj_set_pos(l_dum, 260, 160);
    lv_obj_set_size(l_dum, 40, 20);
    lv_obj_set_style_text_align(l_dum, LV_TEXT_ALIGN_LEFT, 0);
    lv_label_set_text(l_dum, "l/min");

    lv_obj_t* l_ps = lv_label_create(pump_scr);
    lv_obj_add_style(l_ps, &cell_style, 0);
    lv_obj_set_pos(l_ps, 20, 185);
    lv_obj_set_size(l_ps, 70, 20);
    lv_label_set_text(l_ps, "Stare:");

    l_pump_state = lv_label_create(pump_scr);
    lv_obj_add_style(l_pump_state, &cell_style, 0);
    lv_obj_set_pos(l_pump_state, 95, 185);
    lv_obj_set_size(l_pump_state, 120, 20);
    lv_obj_set_style_text_align(l_dum, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(l_pump_state, "Eroare start");
    lv_obj_set_style_text_font(l_pump_state, &seg_black_20, 0);
    lv_obj_set_style_text_color(l_pump_state, lv_color_hex(0x40ff40), 0);

    btns[1].btn = lv_btn_create(pump_scr);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[1].btn, 260, 180);
    lv_obj_set_size(btns[1].btn, 45, 35);

    label = lv_label_create(btns[1].btn);
    lv_label_set_text(label, "<<");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    btns[1].state = 0;

    lv_scr_load_anim(pump_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);

    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
	}

int do_pump_screen()
	{
	msg_t msg;
	char buf[10];
	int p_state, p_status, p_current, p_current_lim, p_min_pres, p_max_pres, p_press;
	int i, kesc = 0;
	get_pump_values(&p_state, &p_status, &p_current, &p_current_lim, &p_min_pres, &p_max_pres, &p_press);
	draw_pump_screen();
	lv_label_set_text_fmt(l_press_min, "%3d", p_min_pres);
	lv_label_set_text_fmt(cpress, "%3d", p_press);
	lv_label_set_text_fmt(l_pressm, "%3d", p_max_pres);
	sprintf(buf, "%3.1f", p_current / 1000.);
	lv_label_set_text(ccurrent, buf);
	sprintf(buf, "%3.1f", p_current_lim / 1000.);
	lv_label_set_text(l_current_max, buf);


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
				if(msg.val == PUSH_TIME_SHORT && btns[1].state == 1)
					{
					kesc = 1;
					break;
					}
				if(msg.val == PUSH_TIME_LONG && btns[0].state == 1)
					{
					//change pump state
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
	return PUMP_SCREEN;
	}
