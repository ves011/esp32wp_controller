/*
 * handle_ui_key.c
 *
 *  Created on: Mar 17, 2024
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

static int k_act = 1;
static int b_light = LCD_BK_LIGHT_ON_LEVEL;

int handle_ui_key(lv_obj_t *watch, btn_main_t *btns, int nbuttons)
	{
	msg_t msg;
	int i, kesc = 0, ret = ESP_OK;
	while(!kesc)
		{
		if(xQueueReceive(ui_cmd_q, &msg, portMAX_DELAY))
			{
			//ESP_LOGI("UI KEY", "src: %lu, val: %lu - %d, %d", msg.source, msg.val, b_light, k_act);
			if (msg.source & K_ROT) //rot left or right
				{
				//ESP_LOGI("UI KEY", "rot %lu, %d, %d", msg.val, b_light, k_act);
				if(k_act == 0)
					{
					k_act = 1;
					if(b_light == LCD_BK_LIGHT_OFF_LEVEL)
						{
						gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
						b_light = LCD_BK_LIGHT_ON_LEVEL;
						continue;
						}
					}
				if(msg.val == K_ROT_RIGHT)
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
					}
				else if(msg.val == K_ROT_LEFT)
					{
					for (i = nbuttons - 1; i >= 0; i--)
						{
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
					}
				}
			else if(msg.source & K_KEY)
				{
				if(k_act == 0)
					{
					k_act = 1;
					if(b_light == LCD_BK_LIGHT_OFF_LEVEL)
						{
						gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
						b_light = LCD_BK_LIGHT_ON_LEVEL;
						continue;
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
				else if(msg.source == K_UP && k_act)
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
				else if (msg.source == K_PRESS)
					{
					if(k_act == 0)
						{
						k_act = 1;
						gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
						continue;
						}
					if(msg.val == PUSH_TIME_SHORT)
						{
						for(int i = 0; i < nbuttons; i++)
							{
							if(btns[i].state == 1)
								{
								kesc = 1;
								ret = KEY_PRESS_SHORT;
								}
							}
						}
					else if(msg.val == PUSH_TIME_LONG)
						{
						for(int i = 0; i < nbuttons; i++)
							{
							if(btns[i].state == 1)
								{
								kesc = 1;
								ret = KEY_PRESS_LONG;
								}
							}
						}
					}
				}
			else if(msg.source == INACT_TIME)
				{
				time_t now = 0;
				char buf[28];
				struct tm timeinfo = { 0 };
				time(&now);
				localtime_r(&now, &timeinfo);
				sprintf(buf, "%02d:%02d - %02d.%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_mday, timeinfo.tm_mon + 1);
				lv_label_set_text(watch, buf);
				if(k_act == 0)
					{
					gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);
					b_light = LCD_BK_LIGHT_OFF_LEVEL;
					}
				k_act = 0;
				}

			else if(msg.source == PUMP_VAL_CHANGE)
				{
				kesc = 1;
				ret = PUMP_VAL_CHANGE;
				}
			else if(msg.source == PUMP_OP_ERROR)
				{
				kesc = 1;
				ret = PUMP_OP_ERROR;
				}
			else if(msg.source == WATER_VAL_CHANGE)
				{
				kesc = 1;
				ret = WATER_VAL_CHANGE;
				}
			else if(msg.source == WATER_OP_ERROR)
				{
				kesc = 1;
				ret = WATER_OP_ERROR;
				}
			else if(msg.source == WATER_DV_OP)
				{
				kesc = 1;
				ret = msg.val << 16 | WATER_DV_OP;
				//ret = WATER_DV_OP;
				}
			}
		}
	return ret;
	}
