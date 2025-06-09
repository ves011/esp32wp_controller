/*
 * lcd.h
 *
 *  Created on: Mar 8, 2024
 *      Author: viorel_serbu
 */

#ifndef LCD_LCD_H_
#define LCD_LCD_H_

//#define CONFIG_LCD_CONTROLLER_ILI9341 1

#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !LCD_BK_LIGHT_ON_LEVEL
#define LCD_SCLK           	5
#define LCD_MOSI           	18
#define LCD_MISO           -1
#define LCD_DC         		19
#define LCD_RST        		21
#define LCD_CS         		22
#define LCD_BK_LIGHT       	17
#define TOUCH_CS       		-1

// The pixel number in horizontal and vertical
#define LCD_H_RES              240
#define LCD_V_RES              320

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    2

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

#define MAIN_SCREEN		1
#define PUMP_SCREEN		2
#define WATER_SCREEN	3

#define INACTIVITY_TIME	60000000		//msec of inactivity

#define LEDON		lv_color_hex(0xf0f040)
#define LEDOFF		lv_color_hex(0x606060)
#define LEDFAULT	lv_color_hex(0xff4040)

typedef struct
    {
    lv_obj_t *btn;
    uint32_t state;
    }btn_main_t;

void lcd_init(void);
void lvgl_task(void *pvParameters);
//void ui_cmd_task();


#endif /* LCD_LCD_H_ */
