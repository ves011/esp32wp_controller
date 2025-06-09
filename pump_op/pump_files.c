/*
 * pump_files.c
 *
 *  Created on: Nov 14, 2024
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sys/stat.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "common_defines.h"
#include "project_specific.h"
#include "external_defs.h"
#include "hal/adc_types.h"
//#include "driver/timer.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "gpios.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#ifdef ADC_ESP32
	#include "adc_op.h"
#endif
#ifdef ADC_AD7811
	#include "ad7811.h"
#endif
#include "waterop.h"
#include "pumpop.h"

static const char *TAG = "PUMP_files";

int rw_twater(int rw, float *total_qw)
	{
	int ret = ESP_FAIL;
	FILE *f = NULL;
	char buf[64], bufw[64];
	buf[0] = 0;
	if(rw == PARAM_READ)
		{
		f = fopen(BASE_PATH"/"TWATER_FILE, "r");
		if(f)
			{
			if(fgets(buf, 64, f))
				{
				*total_qw = atof(buf + 18);
				//sscanf(buf + 18, "%f", total_qw);
				//*total_qw = atoll(buf + 18);
				ret = ESP_OK;
				}
			fclose(f);
			}
		}
	if(rw == PARAM_WRITE)
		{
		f = fopen(BASE_PATH"/"TWATER_FILE, "r");
		if(f)
			{
			if(fgets(buf, 64, f))
				buf[19] = 0;
			fclose(f);
			}
		f = fopen(BASE_PATH"/"TWATER_FILE, "w");
		if(f)
			{
			if(buf[0] == 0)
				{
				struct tm tminfo;
				time_t ltime;
				ltime = time(NULL);
				localtime_r(&ltime, &tminfo);
				sprintf(buf, "%4d-%02d-%02dT%02d:%02d > ", tminfo.tm_year + 1900, tminfo.tm_mon + 1, tminfo.tm_mday, tminfo.tm_hour, tminfo.tm_min);
				}
			sprintf(bufw, "%.2f\n", *total_qw);
			strcat(buf, bufw);
			if(fputs(buf, f) != EOF)
				ret = ESP_OK;
			fclose(f);
			}
		}
	return ret;
	}

int rw_poffset(int rw, int *v_offset)
	{
	int ret  = ESP_OK;
	struct stat st;
	char buf[20];
	*v_offset = 0;
	if(rw == PARAM_READ)
		{
		if (stat(BASE_PATH"/"OFFSET_FILE, &st) != 0)
			{
			// file does no exists
			ESP_LOGI(TAG, "file: %s does not exists", BASE_PATH"/"OFFSET_FILE);
			ret = ESP_OK;
			}
		else
			{
			FILE *f = fopen(BASE_PATH"/"OFFSET_FILE, "r");
			if (f != NULL)
				{
				if(fgets(buf, 64, f))
					*v_offset = atoi(buf);
				fclose(f);
				ret = ESP_OK;
				}
			else
				{
				ESP_LOGE(TAG, "Failed to open offset file for reading");
				return ret;
				}
			}
		}
	else if(rw == PARAM_WRITE)
		{
		FILE *f = fopen(BASE_PATH"/"OFFSET_FILE, "w");
		if (f == NULL)
			{
			ESP_LOGE(TAG, "Failed to create offset param file");
			}
		else
			{
			sprintf(buf, "%d\n", *v_offset);
			if(fputs(buf, f) >= 0)
				ret = ESP_OK;
			fclose(f);
			}
		}
	return ret;
	}
int rw_plimits(int rw, pump_limits_t *param_val)
	{
	int ret = ESP_FAIL;
	struct stat st;
	char buf[80];
	((pump_limits_t *)param_val)->min_val = DEFAULT_PRES_MIN_LIMIT;
	((pump_limits_t *)param_val)->faultc = DEFAULT_PUMP_CURRENT_LIMIT;
	if(rw == PARAM_READ)
		{
		if (stat(BASE_PATH"/"LIMITS_FILE, &st) != 0)
			{
			// file does no exists
			ESP_LOGI(TAG, "file: %s does not exists. Taking default values", BASE_PATH"/"LIMITS_FILE);
			ret = ESP_OK;
			}
		else
			{
			FILE *f = fopen(BASE_PATH"/"LIMITS_FILE, "r");
			if (f != NULL)
				{
				((pump_limits_t *)param_val)->min_val = ((pump_limits_t *)param_val)->faultc = 0xffffff;
				if(fgets(buf, 64, f))
					{
					((pump_limits_t *)param_val)->min_val = atoi(buf);
					if(fgets(buf, 64, f))
						((pump_limits_t *)param_val)->faultc = atoi(buf);
					}
				else
					ESP_LOGI(TAG, "Limits file exists but its empty");
				fclose(f);
				}
			else
				ESP_LOGE(TAG, "Failed to open limits file for reading. Taking default values");
			}
		}
	else if(rw == PARAM_WRITE)
		{
		FILE *f = fopen(BASE_PATH"/"LIMITS_FILE, "w");
		if (f == NULL)
			ESP_LOGE(TAG, "Failed to create limits param file");
		else
			{
			sprintf(buf, "%u\n%u\n",
					(unsigned int)(pump_limits_t *)param_val->min_val, (unsigned int)(pump_limits_t *)param_val->faultc);
			if(fputs(buf, f) >= 0)
				ret = ESP_OK;
			fclose(f);
			}
		}
	return ret;
	}
int rw_poperational(int rw, int *param_val)
	{
	int ret = ESP_FAIL;
	struct stat st;
	char buf[80];
	if(rw == PARAM_READ)
		{
		*(int *)param_val = PUMP_OFFLINE;
		if (stat(BASE_PATH"/"OPERATIONAL_FILE, &st) != 0)
			{
			// file does no exists
			ret = ESP_OK;
			}
		else
			{
			FILE *f = fopen(BASE_PATH"/"OPERATIONAL_FILE, "r");
			if (f != NULL)
				{
				if(fgets(buf, 64, f))
					{
					*(int *)param_val = atoi(buf);
					ret = ESP_OK;
					}
				else
					ESP_LOGI(TAG, "Operational file exists but its empty");
				fclose(f);
				}
			else
				ESP_LOGE(TAG, "Failed to open operational file for reading");
			}
		}
	else if(rw == PARAM_WRITE)
		{
		FILE *f = fopen(BASE_PATH"/"OPERATIONAL_FILE, "w");
		if (f == NULL)
			{
			ESP_LOGE(TAG, "Failed to create operational file");
			}
		else
			{
			sprintf(buf, "%d\n", *(int *)param_val);
			if(fputs(buf, f) >= 0)
				ret = ESP_OK;
			fclose(f);
			}
		}
	return ret;
	}