/*
 * boot_screen.c
 *
 *  Created on: Apr 24, 2024
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
#include "boot_screen.h"

extern lv_style_t btn_norm, btn_sel, btn_press, cell_style, cell_style_left;
static lv_obj_t *boot_scr;
static lv_obj_t *bstep[8];

static void draw_boot_screen()
	{
	gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

	boot_scr = lv_obj_create(NULL);
	lv_obj_set_style_bg_color(boot_scr, lv_color_hex(0x0), LV_PART_MAIN);
	int y0 = 5, sy = 22;
	for(int i = 0; i < 8; i++)
		{
		bstep[i] = lv_label_create(boot_scr);
		lv_obj_add_style(bstep[i], &cell_style_left, 0);
		lv_obj_set_pos(bstep[i], 5, y0 + i * sy);
		lv_obj_set_size(bstep[i], 300, 20);
		lv_label_set_text(bstep[i], "");
		}
	lv_scr_load_anim(boot_scr, LV_SCR_LOAD_ANIM_MOVE_LEFT, 100, 0, true);
	gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
	}

void do_boot_screen()
	{
	int kesc = 0;
	msg_t msg;
	draw_boot_screen();
	char bmsg[80];
	while(!kesc)
		{
		if(xQueueReceive(ui_cmd_q, &msg, portMAX_DELAY))
			{
			if(msg.source == BOOT_MSG)
				{
				if(msg.val == 8)
					break;
				bmsg[0] = 0;
				switch(msg.val)
					{
					case 0:
						strcat(bmsg, "LCD init completed ");
						break;
					case 1:
						strcat(bmsg, "spiffs, nvs - initit completed ");
						break;
					case 2:
						strcat(bmsg, "connected to network ");
						break;
					case 3:
						strcat(bmsg, "NTP sync task created ");
						break;
					case 4:
						strcat(bmsg, "MQTT client started ");
						break;
					case 5:
						strcat(bmsg, "system commands registered ");
						break;
					case 6:
						strcat(bmsg, "pump operation registered");
						break;
					case 7:
						strcat(bmsg, "registering water op ");
						break;
					}
				lv_label_set_text(bstep[msg.val], bmsg);
				}
			}
		}
	}

