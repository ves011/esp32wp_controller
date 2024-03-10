/*
 * lcd.h
 *
 *  Created on: Mar 8, 2024
 *      Author: viorel_serbu
 */

#ifndef LCD_LCD_H_
#define LCD_LCD_H_

#define CONFIG_EXAMPLE_LCD_CONTROLLER_ILI9341 1

#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           2
#define EXAMPLE_PIN_NUM_MOSI           4
#define EXAMPLE_PIN_NUM_MISO           -1
#define EXAMPLE_PIN_NUM_LCD_DC         16
#define EXAMPLE_PIN_NUM_LCD_RST        21
#define EXAMPLE_PIN_NUM_LCD_CS         22
#define EXAMPLE_PIN_NUM_BK_LIGHT       15
#define EXAMPLE_PIN_NUM_TOUCH_CS       -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              320

// Bit number used to represent command and parameter
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

// Using SPI2 in the example
#define LCD_HOST  SPI2_HOST

//input source
#define K_ROT		10
#define K_ROT_LEFT	11
#define K_ROT_RIGHT	12
#define K_PRESS		20

typedef struct
    {
    lv_obj_t *btn;
    uint32_t state;
    }btn_main_t;

void lcd_init(void);
//void ui_cmd_task();


#endif /* LCD_LCD_H_ */
