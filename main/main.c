/*
 * water and pump controller main.c
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "spi_flash_mmap.h"
#include "esp_spiffs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wear_levelling.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "linenoise/linenoise.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_pm.h"
//#include "esp32/clk.h"
#include "common_defines.h"
#include "cmd_wifi.h"
#include "cmd_system.h"
#include "utils.h"
#include "tcp_log.h"
#include "mqtt_client.h"
#include "mqtt_ctrl.h"
#include "ntp_sync.h"
#include "esp_ota_ops.h"
#include "hal/adc_types.h"
#include "lvgl.h"
#include "dev_mon.h"
#include "rot_enc.h"
#include "gpios.h"
#include "ad7811.h"
#include "pumpop.h"
#include "waterop.h"
#include "lcd.h"

#include "external_defs.h"

#define TAG "ctrl_dev"

#define CONFIG_STORE_HISTORY 1
#define CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH	1024

#include "wifi_credentials.h"

console_state_t console_state;
int restart_in_progress;
int controller_op_registered;

int init_completed;

static void initialize_nvs(void)
	{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
		{
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
		}
	ESP_ERROR_CHECK(err);
	}

void app_main(void)
	{
	msg_t msg;
	msg.source = BOOT_MSG;
	ESP_LOGI(TAG, "app main");
	gpio_install_isr_service(0);

	int bp_val = 1;
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER
	int bp_ctrl = PUMP_ONLINE_CMD;
#elif ACTIVE_CONTROLLER == WP_CONTROLLER
	int bp_ctrl = ROT_ENC_KEY;
	bp_val = 1;
#elif ACTIVE_CONTROLLER == AGATE_CONTROLLER
	int bp_ctrl = 6; //IO6 on J5 pin 8
#elif ACTIVE_CONTROLLER == WESTA_CONTROLLER
	int bp_ctrl = 8; //IO8 on J4 pin 8
#elif ACTIVE_CONTROLLER == WATER_CONTROLLER
	adc_init5();
	int bp_ctrl = 8; //TBD
	gpio_install_isr_service(0);
#endif
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << bp_ctrl);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    //ESP_LOGI(TAG, "app main 1 / %lu", esp_get_free_heap_size());
    /*
     * if BOOT_CTRL_PIN is bp_val at boot restart with esp32_ota
     */
    if(gpio_get_level(bp_ctrl) == bp_val)
    	{
    	//msg.val = 10;
    	//xQueueSend(ui_cmd_q, &msg, 0);
    	ESP_LOGI(TAG, "app main 1:1");
    	const esp_partition_t *sbp = NULL;;
    	esp_partition_iterator_t pit = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
    	while(pit)
    		{
    		sbp = esp_partition_get(pit);
    		if(sbp)
    			{
   				if(!strcmp(sbp->label, OTA_PART_NAME))
    				break;
    			}
    		pit = esp_partition_next(pit);
    		}
    	if(sbp)
			{
			int err = esp_ota_set_boot_partition(sbp);
			if(err == ESP_OK)
				esp_restart();
    		}
    	}
	gpio_reset_pin(bp_ctrl);
	setenv("TZ","EET-2EEST,M3.4.0/03,M10.4.0/04",1);

	TaskHandle_t lvgl_task_handle;
	init_completed = 0;
	xTaskCreatePinnedToCore(lvgl_task, "lvgl_task", 8192, NULL, 10, &lvgl_task_handle, 1);
	if(!lvgl_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start lvgl task");
		esp_restart();
		}
	while(!init_completed)
		vTaskDelay(pdMS_TO_TICKS(10));
	msg.val = 0;
    xQueueSend(ui_cmd_q, &msg, 0);

	initialize_nvs();
	spiffs_storage_check();
	msg.val = 1;
    xQueueSend(ui_cmd_q, &msg, 0);

	tsync = 0;
	wifi_join(DEFAULT_SSID, DEFAULT_PASS, JOIN_TIMEOUT_MS);
	if(xTaskCreate(dev_mon_task, "dev mon task", 4096, NULL, 5, NULL) != pdPASS)
		{
		ESP_LOGE(TAG, "Cannot create dev_mon_task task");
		esp_restart();
		}
	rw_console_state(PARAM_READ, &console_state);
	//rw_params(PARAM_READ, PARAM_CONSOLE, &console_state);
    //tcp_log_task_handle = NULL;
    tcp_log_evt_queue = NULL;
	
	restart_in_progress = 0;
	controller_op_registered = 0;
	msg.val = 2;
    xQueueSend(ui_cmd_q, &msg, 0);

	// *
	// * the line below is required to make interrupts on gpio 36,39 to work properly
	// * https://github.com/espressif/esp-idf/issues/4585
	// *
	esp_wifi_set_ps(WIFI_PS_NONE);
	//--------------------------------------------------------------------------//

	// start task to sync local time with NTP server
	//xTaskCreate(ntp_sync, "NTP_sync_task", 4096, NULL, USER_TASK_PRIORITY, &ntp_sync_task_handle);
	sync_NTP_time();
	//ESP_LOGI(TAG, "NTP Sync task / %lu", esp_get_free_heap_size());
	msg.val = 3;
    xQueueSend(ui_cmd_q, &msg, 0);
	if(mqtt_start() != ESP_OK)
		esp_restart();
	tcp_log_init();
	esp_log_set_vprintf(my_log_vprintf);
	//ESP_LOGI(TAG, "MQTT task / %lu", esp_get_free_heap_size());
	msg.val = 4;
    xQueueSend(ui_cmd_q, &msg, 0);
#ifdef WITH_CONSOLE
	esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;


#if CONFIG_STORE_HISTORY
	//initialize_filesystem();
	repl_config.history_save_path = BASE_PATH HISTORY_FILE;
	ESP_LOGI(TAG, "Command history enabled");
#else
	ESP_LOGI(TAG, "Command history disabled");
#endif
#endif

	/* Register commands */
	esp_console_register_help_command();
	register_system();
	register_wifi();
	msg.val = 5;
    xQueueSend(ui_cmd_q, &msg, 0);



#if ACTIVE_CONTROLLER == AGATE_CONTROLLER
	register_gateop();
#endif
#if ACTIVE_CONTROLLER == PUMP_CONTROLLER || ACTIVE_CONTROLLER == WP_CONTROLLER

	register_ad();
	register_pumpop();
	msg.val = 6;
    xQueueSend(ui_cmd_q, &msg, 0);
#endif
#if ACTIVE_CONTROLLER == WESTA_CONTROLLER
	register_westaop();
#endif
#if ACTIVE_CONTROLLER == WP_CONTROLLER
	msg.val = 7;
    xQueueSend(ui_cmd_q, &msg, 0);
//#ifndef TEST1
	register_waterop();
//#endif
#endif
	controller_op_registered = 1;
	msg.val = 8;
    xQueueSend(ui_cmd_q, &msg, 0);

	//lcd_init();
	//init_rotenc();
	//ESP_LOGI(TAG, "init rot enc / %lu", esp_get_free_heap_size());
#ifdef WITH_CONSOLE
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    repl_config.task_stack_size = 8192;
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    repl_config.task_stack_size = 8192;
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    //ESP_LOGI(TAG, "console stack: %d", repl_config.task_stack_size);

#else
	#error Unsupported console type
#endif

	ESP_ERROR_CHECK(esp_console_start_repl(repl));
	ESP_LOGI(TAG, "console start / %lu", (unsigned long)esp_get_free_heap_size());
#endif
	//while(1)
	//	vTaskDelay(pdMS_TO_TICKS(100));
	}
