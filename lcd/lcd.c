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
#include "unistd.h"
#include "driver/gptimer.h"
#include "common_defines.h"
#include "lvgl.h"
#include "external_defs.h"
#include "main_screen.h"
#include "pump_screen.h"
#include "water_screen.h"
#include "lcd.h"

lv_style_t btn_norm, btn_sel, btn_press, cell_style, cell_style_left;
QueueHandle_t ui_cmd_q;

int lv_timer_stop;
static uint32_t active_screen;
static lv_disp_t *disp;
static TaskHandle_t lvgl_task_handle, ui_task_handle;


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
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static bool IRAM_ATTR inactivity_timer_callback(gptimer_handle_t c_timer, const gptimer_alarm_event_data_t *edata, void *args)
	{
	msg_t msg;
    BaseType_t high_task_awoken = pdFALSE;
    msg.source = INACT_TIME;
	xQueueSendFromISR(ui_cmd_q, &msg, NULL);
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}

static void config_inactivity_timer()
	{
	int ret;
	gptimer_handle_t inactivity_timer;
	inactivity_timer = NULL;
	gptimer_config_t gptconf = 	{
								.clk_src = GPTIMER_CLK_SRC_DEFAULT,
								.direction = GPTIMER_COUNT_UP,
								.resolution_hz = 1000000,					//1 usec resolution

								};
	gptimer_alarm_config_t al_config = 	{
										.reload_count = 0,
										.alarm_count = INACTIVITY_TIME,
										.flags.auto_reload_on_alarm = true,
										};

	gptimer_event_callbacks_t cbs = {.on_alarm = &inactivity_timer_callback,}; // register user callback
	ESP_ERROR_CHECK(gptimer_new_timer(&gptconf, &inactivity_timer));
	ESP_ERROR_CHECK(gptimer_set_alarm_action(inactivity_timer, &al_config));
	ret = gptimer_register_event_callbacks(inactivity_timer, &cbs, NULL);
	ESP_LOGI(TAG, "cb register %d", ret);
	ESP_ERROR_CHECK(gptimer_enable(inactivity_timer));
	gptimer_start(inactivity_timer);
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

static void ui_task(void *pvParameters)
	{
	int ret;
	active_screen = PUMP_SCREEN;
	while(1)
		{
		do_main_screen(active_screen);
		}
	}

void lcd_init(void)
{
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions
    lv_timer_stop = 0;
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_SCLK,
        .mosi_io_num = LCD_MOSI,
        .miso_io_num = LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DC,
        .cs_gpio_num = LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = example_notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    ESP_LOGI(TAG, "Install ILI9341 panel driver");
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9341(io_handle, &panel_config, &panel_handle));


    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(LCD_BK_LIGHT, LCD_BK_LIGHT_OFF_LEVEL);

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    // alloc draw buffers used by LVGL
    // it's recommended to choose the size of the draw buffer(s) to be at least 1/10 screen sized
    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.drv_update_cb = example_lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    disp = lv_disp_drv_register(&disp_drv);
    lv_indev_enable(NULL, false);

    lv_style_init(&cell_style);
    lv_style_reset(&cell_style);
    lv_style_set_text_color(&cell_style, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&cell_style, &lv_font_montserrat_14);
    lv_style_set_text_align(&cell_style, LV_TEXT_ALIGN_RIGHT);

    lv_style_init(&cell_style_left);
    lv_style_reset(&cell_style_left);
    lv_style_set_text_color(&cell_style_left, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&cell_style_left, &lv_font_montserrat_14);
    lv_style_set_text_align(&cell_style_left, LV_TEXT_ALIGN_LEFT);

    lv_style_init(&btn_sel);
    lv_style_init(&btn_norm);

    lv_style_set_text_color(&btn_norm, lv_color_hex(0xc0c0c0));
    lv_style_set_text_font(&btn_norm, &seg_black_20);

    lv_style_set_translate_y(&btn_sel, 5);
    lv_style_set_shadow_ofs_y(&btn_sel, 10);
    lv_style_set_shadow_ofs_x(&btn_sel, 10);
    lv_style_set_text_color(&btn_sel, lv_color_hex(0xffff00));
    lv_style_set_text_font(&btn_sel, &seg_black_20);

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = { LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0 };
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);
    lv_style_set_transition(&btn_sel, &trans);

    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);
    lv_style_set_transition(&btn_sel, &trans);

    /*Init the pressed style*/
    lv_style_init(&btn_press);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&btn_press, 30);
    lv_style_set_outline_opa(&btn_press, LV_OPA_TRANSP);

    lv_style_set_translate_y(&btn_press, 5);
    lv_style_set_shadow_ofs_y(&btn_press, 3);
    lv_style_set_bg_color(&btn_press, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_color(&btn_press, lv_palette_darken(LV_PALETTE_BLUE, 4));

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t transp;
    static lv_style_prop_t propsp[] = { LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0 };
    lv_style_transition_dsc_init(&transp, propsp, lv_anim_path_linear, 300, 0, NULL);

    lv_style_set_transition(&btn_press, &trans);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lv_disp_set_rotation(disp, LV_DISP_ROT_270);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x0), LV_PART_MAIN);

    ui_cmd_q = xQueueCreate(10, sizeof(msg_t));
	if(!ui_cmd_q)
		{
		ESP_LOGE(TAG, "Unable to create UI cmd queue");
		esp_restart();
		}

    active_screen = 0;

    config_inactivity_timer();

    xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 8192, NULL, 10, &lvgl_task_handle, 1);
	if(!lvgl_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start lvgl task");
		esp_restart();
		}
	xTaskCreatePinnedToCore(ui_task, "ui_task", 8192, NULL, 5, &ui_task_handle, 1);
	if(!ui_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start UI task");
		esp_restart();
		}
	}

