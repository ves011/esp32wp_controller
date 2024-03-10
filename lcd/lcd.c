/*
 * lcd.c
 *
 *  Created on: Mar 8, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/freertos.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_ili9341.h"
#include "unistd.h"
#include "common_defines.h"
#include "lvgl.h"
#include "lcd.h"

static TaskHandle_t lvgl_task_handle, ui_cmd_task_handle;
extern QueueHandle_t ui_cmd_q;
static btn_main_t btns[2];
static lv_style_t btn_norm, btn_sel;

const char *TAG = "LCD";

static bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static void lvgl_task(void *pvParameters)
	{
	 while (1)
		 {
		 // raise the task priority of LVGL and/or reduce the handler period can improve the performance
		 vTaskDelay(pdMS_TO_TICKS(20));
		 //usleep(5000);
		 // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
		 lv_timer_handler();
		 }
	}

static void bpump_event_cb(lv_event_t* e)
    {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* btnl = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
        {
        //lv_log("app bpump click\n");
        }
    }
static void bwater_event_cb(lv_event_t* e)
    {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* btnl = (lv_obj_t *)lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
        {
        //lv_log("app bwater click\n");
        }
    }

static void ui_cmd_task(void *pvParameters)
    {
	msg_t msg;
	int i;
	while(1)
		{
		if(xQueueReceive(ui_cmd_q, &msg, portMAX_DELAY))
			{
			if (msg.source == K_ROT) //rot left or right
				{
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
			}
        }
    }


void main_screen(lv_disp_t *disp)
	{
	lv_obj_t *label;
	lv_disp_set_rotation(disp, LV_DISP_ROT_270);

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x0), LV_PART_MAIN);
    //lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE_GREY), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, font_normal);

    lv_style_init(&btn_sel);
    lv_style_init(&btn_norm);

    //lv_style_set_outline_width(&btn_norm, 10);
    //lv_style_set_outline_opa(&btn_norm, LV_OPA_COVER);
    //lv_style_set_translate_y(&btn_norm, 5);
    //lv_style_set_shadow_ofs_y(&btn_norm, 10);
    //lv_style_set_bg_color(&btn_norm, lv_palette_darken(LV_PALETTE_GREY, 2));
    lv_style_set_text_color(&btn_norm, lv_color_hex(0x808080));
    lv_style_set_text_font(&btn_norm, &seg_black_10);

    //lv_style_set_outline_width(&btn_sel, 10);
    //lv_style_set_outline_opa(&btn_sel, LV_OPA_100);
    lv_style_set_translate_y(&btn_sel, 5);
    lv_style_set_shadow_ofs_y(&btn_sel, 10);
    lv_style_set_shadow_ofs_x(&btn_sel, 10);
    //lv_style_set_bg_color(&btn_sel, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_text_color(&btn_sel, lv_color_hex(0xffff00));
    //lv_style_set_bg_grad_color(&btn_sel, lv_palette_darken(LV_PALETTE_BLUE, 3));
    lv_style_set_text_font(&btn_sel, &seg_black_10);

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = { LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0 };
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);
    lv_style_set_transition(&btn_sel, &trans);


    btns[0].btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    //lv_obj_remove_style_all(btns[0].btn);
    lv_obj_add_style(btns[0].btn, &btn_norm, 0);
    lv_obj_add_style(btns[0].btn, &btn_sel, LV_STATE_PRESSED);

    lv_obj_set_pos(btns[0].btn, 80, 10);                            /*Set its position*/
    lv_obj_set_size(btns[0].btn, 160, 35);                          /*Set its size*/
    lv_obj_add_event_cb(btns[0].btn, bpump_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/

    label = lv_label_create(btns[0].btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Pompa foraj");                     /*Set the labels text*/
    lv_obj_center(label);
    btns[0].state = 0;

    btns[1].btn = lv_btn_create(lv_scr_act());     /*Add a button the current screen*/
    //lv_obj_remove_style_all(btns[1].btn);
    lv_obj_add_style(btns[1].btn, &btn_norm, 0);
    lv_obj_add_style(btns[1].btn, &btn_sel, LV_STATE_PRESSED);
    lv_obj_set_pos(btns[1].btn, 80, 70);                            /*Set its position*/
    lv_obj_set_size(btns[1].btn, 160, 35);

    lv_obj_add_event_cb(btns[1].btn, bwater_event_cb, LV_EVENT_ALL, NULL);           /*Assign a callback to the button*/
    label = lv_label_create(btns[1].btn);          /*Add a label to the button*/
    lv_label_set_text(label, "Irigatie");                     /*Set the labels text*/
    lv_obj_center(label);
    btns[1].state = 1;
    lv_obj_add_state(btns[1].btn, LV_STATE_PRESSED);

	}


void lcd_init(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));


    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    ui_cmd_q = xQueueCreate(10, sizeof(msg_t));
	if(!ui_cmd_q)
		{
		ESP_LOGE(TAG, "Unable to create UI cmd queue");
		esp_restart();
		}

    //example_lvgl_demo_ui(disp);
    main_screen(disp);

    xTaskCreatePinnedToCore(ui_cmd_task, "ui_cmd", 8192, NULL, 5, &ui_cmd_task_handle, 1);
	if(!ui_cmd_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start ui cmd task");
		esp_restart();
		}

    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 8192, NULL, 5, &lvgl_task_handle, 1);
	if(!lvgl_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start lvgl task");
		esp_restart();
		}
	}

