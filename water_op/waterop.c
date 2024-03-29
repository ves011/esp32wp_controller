/*
 * water_op.c
 *
 *  Created on: Jun 8, 2023
 *      Author: viorel_serbu
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "hal/gpio_types.h"
#include "freertos/freertos.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_netif.h"
#include "esp_spiffs.h"
#include "esp_log.h"
#include "sys/stat.h"
#include "driver/gptimer.h"
#include "esp_timer.h"
#include "common_defines.h"
#include "gpios.h"
#include "mqtt_client.h"
#include "mqtt_ctrl.h"
#include "utils.h"
#ifdef ADC_ESP32
	#include "adc_op.h"
#endif
#ifdef ADC_AD7811
	#include "ad7811.h"
#endif
#include "pumpop.h"
#include "waterop.h"
#include "external_defs.h"

#if ACTIVE_CONTROLLER == WATER_CONTROLLER || ACTIVE_CONTROLLER == WP_CONTROLLER

extern int pump_max_lim, pump_pressure_kpa;

static gptimer_handle_t test_timer;
static int timer_state;

static struct {
    struct arg_str *op;
    struct arg_int *dv;
    struct arg_str *start;
    struct arg_str *stop;
    struct arg_end *end;
} waterop_args;

static const char *TAG = "WATER OP";
static TaskHandle_t water_task_handle;
static int watering_status;
static last_status_t last_wstatus[DVCOUNT];
//static int pump_present, pstate, pstatus, ppressure, pminlim, pmaxlim;
//static volatile uint64_t last_pump_state, last_pump_mon;
static dvprogram_t dv_program;
dvconfig_t dvconfig[DVCOUNT];
int activeDV;

static int read_program_status(int mq);
static int read_program(dvprogram_t *param_val);
static int write_program(dvprogram_t *param_val);
static int start_watering(int idx);
static int stop_watering(int idx, int reason);
static int write_status(int idx);
static int get_act_state(int dvnum);
static int test_mode;
static void config_test_mode();
int get_dv_adc_values(int *dv_mv)
	{
	int16_t dvv[5];
	int ret = ESP_OK;
	if(test_mode)
		{
		if(timer_state == 1)
			*dv_mv = 150;
		else
			*dv_mv = 0;
		return ret;
		}
	if(adc_get_data(MOTSENSE_CHN, dvv, 3) == ESP_OK)
		*dv_mv = (dvv[0] + dvv[1] + dvv[2]) / 3;
	else
		ret = ESP_FAIL;
	ESP_LOGI(TAG, "adc dv %d", *dv_mv);
	return ret;
	}

static int open_dv(int dvnum)
	{
	int dv_current, op_start = 0, ret = ESP_OK;
	int coff = 0, i;
	ret = get_act_state(dvnum);
	/*
	if(ret == DVOPEN)
		{
		ESP_LOGI(TAG, "DV%d already in DVOPEN state", dvnum);
		activeDV =  dvconfig[dvnum].dvno;
		dvconfig[dvnum].state = DVOPEN;
		ret = ESP_OK;
		}
	else*/
		{
		if(test_mode && timer_state == 0)
			{
			gptimer_set_raw_count(test_timer, 0);
			gptimer_start(test_timer);
			timer_state = 1;
			}
		gpio_set_level(PINMOT_A1, PIN_ON);
		gpio_set_level(PINMOT_B1, PIN_OFF);
		if(dvnum == 0)
			gpio_set_level(PINEN_DV0, PIN_ON);
		else if(dvnum == 1)
			gpio_set_level(PINEN_DV1, PIN_ON);

		/*
		 * loop until measured current is < CURRENT_OFF_LIM
		 * 30 loops ~ 15 sec
		 */
		for(i = 0; i < 30; i++)
			{
			vTaskDelay(500 / portTICK_PERIOD_MS);
			if((ret = get_dv_adc_values(&dv_current)) == ESP_OK)
				{
				if(dv_current < CURRENT_OFF_LIM)
					coff++;
				else
					{
					coff = 0;
					op_start = 1;
					}
				if(coff >= CURRENT_OFF_COUNT)
					break;
				}
			else
				break;
			}
		gpio_set_level(PINMOT_A1, PIN_OFF);
		gpio_set_level(PINMOT_B1, PIN_OFF);
		gpio_set_level(PINEN_DV0, PIN_OFF);
		gpio_set_level(PINEN_DV1, PIN_OFF);
		if(ret == ESP_OK)
			{
			if(op_start) //dv current started
				{
				if(dv_current < CURRENT_OFF_LIM) // open dv OK
					{
					ESP_LOGI(TAG, "open DV%d OK %d %d", dvnum, i, dv_current);
					activeDV =  dvconfig[dvnum].dvno;
					dvconfig[dvnum].state = DVOPEN;
					ret = ESP_OK;
					}
				else // still dv consumes current after 15 secs --> open failure
					{
					ESP_LOGI(TAG, "open DV%d failure %d %d", dvnum, i, dv_current);
					activeDV =  -1;
					dvconfig[dvnum].state = DVSTATE_FAULT;
					ret = DV_OPEN_FAIL;
					}
				}
			else
				{
				ESP_LOGI(TAG, "DV%d already in open state %d %d", dvnum, i, dv_current);
				activeDV =  dvconfig[dvnum].dvno;;
				dvconfig[dvnum].state = DVOPEN;
				ret = ESP_OK;
				}
			}
		else
			{
			ESP_LOGI(TAG, "DV%d open failure: ADC read error %d %d", dvnum, i, dv_current);
			//dvconfig[dvnum].state = DVCLOSE;
			ret = DV_OPEN_FAIL;
			}
		}
	return ret;
	}
static int close_dv(int dvnum)
	{
	int dv_current, op_start = 0, ret = ESP_OK;
	int coff = 0, i;
	/*
	ret = get_act_state(dvnum);
	if(ret == DVCLOSE)
		{
		ESP_LOGI(TAG, "DV%d already in DVCLOSED state", dvnum);
		activeDV =  -1;
		dvconfig[dvnum].state = DVCLOSE;
		ret = ESP_OK;
		}
	else */
		{
		if(test_mode && timer_state == 0)
			{
			gptimer_set_raw_count(test_timer, 0);
			gptimer_start(test_timer);
			timer_state = 1;
			}
		gpio_set_level(PINMOT_A1, PIN_OFF);
		gpio_set_level(PINMOT_B1, PIN_ON);
		if(dvnum == 0)
			gpio_set_level(PINEN_DV0, PIN_ON);
		else if(dvnum == 1)
			gpio_set_level(PINEN_DV1, PIN_ON);
		for(i = 0; i < 30; i++)
			{
			vTaskDelay(500 / portTICK_PERIOD_MS);
			if((ret = get_dv_adc_values(&dv_current)) == ESP_OK)
				{
				if(dv_current < CURRENT_OFF_LIM)
					coff++;
				else
					{
					op_start = 1;
					coff = 0;
					}

				if(coff >= CURRENT_OFF_COUNT)
					break;
				}
			else
				break;
			}
		gpio_set_level(PINMOT_A1, PIN_OFF);
		gpio_set_level(PINMOT_B1, PIN_OFF);
		gpio_set_level(PINEN_DV0, PIN_OFF);
		gpio_set_level(PINEN_DV1, PIN_OFF);
		if(ret == ESP_OK)
			{
			if(op_start) //dv current started
				{
				if(dv_current < CURRENT_OFF_LIM) // close dv OK
					{
					ESP_LOGI(TAG, "close DV%d OK %d %d", dvnum, i, dv_current);
					activeDV =  -1;
					dvconfig[dvnum].state = DVCLOSE;
					ret = ESP_OK;
					}
				else // still dv consumes current after 15 secs --> close failure
					{
					ESP_LOGI(TAG, "close DV%d failure %d %d", dvnum, i, dv_current);
					activeDV =  -1;
					dvconfig[dvnum].state = DVSTATE_FAULT;
					}
				}
			else
				{
				ESP_LOGI(TAG, "DV%d already in closed state %d %d", dvnum, i, dv_current);
				activeDV =  -1;
				dvconfig[dvnum].state = DVCLOSE;
				ret = ESP_OK;
				}
			}
		else
			{
			ESP_LOGI(TAG, "DV%d close failure: ADC read error  %d %d", dvnum, i, dv_current);
			activeDV =  dvnum;
			dvconfig[dvnum].state = DVOPEN;
			ret = DV_CLOSE_FAIL;
			}
		}
	return ret;
	}
static void config_dv_gpio(void)
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 	(1ULL << PINMOT_A1) | (1ULL << PINMOT_B1) | (1ULL << PINEN_DV0) | (1ULL << PINEN_DV1);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PINEN_DV0, PIN_OFF);
    gpio_set_level(PINEN_DV1, PIN_OFF);
    gpio_set_level(PINMOT_A1, PIN_OFF);
    gpio_set_level(PINMOT_B1, PIN_OFF);
	}
/*
 * stop ongoing program, if any
 */
static void stop_program()
	{
	for(int i = 0; i < DVCOUNT; i++)
		{
		if(dv_program.p[i].cs == IN_PROGRESS)
			stop_watering(i, ABORTED);
		}
	}
int do_dvop(int argc, char **argv)
	{
	dvprogram_t pval;
	char mqttbuf[256];
	char buf[50], bufs[20];
	int nerrors = arg_parse(argc, argv, (void **)&waterop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, waterop_args.end, argv[0]);
        return 1;
    	}
    if(strcmp(waterop_args.op->sval[0], "test") == 0)
    	{
    	if(waterop_args.dv->count == 1)
    		{
			test_mode = waterop_args.dv->ival[0];
			config_test_mode();
    		}
    	else
    		ESP_LOGI(TAG, "test: %d", test_mode);
    	}
    else if(strcmp(waterop_args.op->sval[0], "open") == 0)
    	{
    	if(waterop_args.dv->count == 1)
			open_dv(waterop_args.dv->ival[0]);
		else
			ESP_LOGI(TAG, "open: no #dv provided");
    	}
    else if(strcmp(waterop_args.op->sval[0], "close") == 0)
    	{
    	if(waterop_args.dv->count == 1)
			close_dv(waterop_args.dv->ival[0]);
		else
			ESP_LOGI(TAG, "close: no #dv provided");
    	}
    else if(strcmp(waterop_args.op->sval[0], "state") == 0)
    	{
    	int st;
    	ESP_LOGI(TAG, "Watering status: %-d / activeDV: %d", watering_status, activeDV);
    	read_program(&dv_program);
    	buf[0] = 0;
    	for(int i = 0; i < DVCOUNT; i++)
			{
			st = get_act_state(dvconfig[i].dvno);
			ESP_LOGI(TAG, "DV%d state: %d / %d", dvconfig[i].dvno, dvconfig[i].state, st);
			sprintf(bufs, "%d\1%d\1", dvconfig[i].dvno, dvconfig[i].state);
			strcat(buf, bufs);
			}
    	if(watering_status == WATER_ON)
    		{
    		ESP_LOGI(TAG, "Watering ON on DV%d started @%02d:%02d", activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
    		}
		/*for(int i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].cs == NOT_STARTED)
				ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
			}*/
		sprintf(mqttbuf, "%s\1%d\1%d\1", STATE_W, watering_status, activeDV);
		strcat(mqttbuf, buf);
		for(int i = 0; i < DVCOUNT; i++)
		    {
		    sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1%d\1",
		    		dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm, dv_program.p[i].stoph,
					dv_program.p[i].stopm, dv_program.p[i].cs, dv_program.p[i].fault);
		    strcat(mqttbuf, buf);
		    ESP_LOGI(TAG, "DV%d program:  %d %d %d %d %d %d %d", i, dv_program.p[i].dv, dv_program.p[i].starth,
		    			dv_program.p[i].startm, dv_program.p[i].stoph, dv_program.p[i].stopm,
						dv_program.p[i].cs, dv_program.p[i].fault);
		    }
		//sprintf(buf, "%d\1%d\1%d\1", pump_status, pump_state, pump_pressure_kpa);
		//strcat(mqttbuf, buf);
		publish_state(mqttbuf, 1, 0);
    	}
    else if(strcmp(waterop_args.op->sval[0], "resetps") == 0)
    	{
    	read_program(&pval);
    	for(int i = 0; i < DVCOUNT; i++)
    		pval.p[i].cs = NOT_STARTED;
    	write_program(&pval);
    	read_program(&dv_program);
    	}
    else if(strcmp(waterop_args.op->sval[0], "program") == 0)
    	{
    	int starth, startm, stoph, stopm, i;
    	read_program(&dv_program);
    	memcpy(&pval, &dv_program, sizeof(dvprogram_t));
    	sprintf(mqttbuf, "%s\1", STATE_P);
    	if(waterop_args.dv->count == 0) // no params -> just show DVs program
    		{
    		for(int i = 0; i < DVCOUNT; i++)
    			{
    			sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm, dv_program.p[i].stoph, dv_program.p[i].stopm, dv_program.p[i].cs);
    			strcat(mqttbuf, buf);
    			ESP_LOGI(TAG, "DV%d program:  %d %d %d %d %d %d %d", i, dv_program.p[i].dv, dv_program.p[i].starth,
    					    			dv_program.p[i].startm, dv_program.p[i].stoph, dv_program.p[i].stopm,
    									dv_program.p[i].cs, dv_program.p[i].fault);
    			}
    		publish_state(mqttbuf, 1, 0);
    		}
    	else
    		{
    		if(waterop_args.start->count == 0)
    			{
    			ESP_LOGI(TAG, "program: no start time provided");
    			}
    		else
    			{
    			if(waterop_args.stop->count == 0)
					{
					ESP_LOGI(TAG, "program: no stop time provided");
					}
				else
					{
					if(waterop_args.dv->ival[0] >=0 && waterop_args.dv->ival[0] < DVCOUNT)
						{
						sscanf(waterop_args.start->sval[0], "%d:%d", &starth, &startm);
						sscanf(waterop_args.stop->sval[0], "%d:%d", &stoph, &stopm);
						if(starth >= 0 && starth <= 23 && stoph >= 0 && stoph <= 23 &&
								startm >= 0 && startm <= 59 && stopm >= 0 && stopm <= 59)
							{
							for(i = 0; i < DVCOUNT; i++)
								{
								if(pval.p[i].dv == waterop_args.dv->ival[0])
									{
									pval.p[i].starth = starth;
									pval.p[i].startm = startm;
									pval.p[i].stoph = stoph;
									pval.p[i].stopm = stopm;
									pval.p[i].cs = 0;
									pval.p[i].fault = 0;
									break;
									}
								}
							if(i == DVCOUNT)
								{
								for(i = 0; i < DVCOUNT; i++)
									{
									if(pval.p[i].dv == -1)
										break;
									}
								if(i < DVCOUNT)
									{
									pval.p[i].dv = waterop_args.dv->ival[0];
									pval.p[i].starth = starth;
									pval.p[i].startm = startm;
									pval.p[i].stoph = stoph;
									pval.p[i].stopm = stopm;
									pval.p[i].cs = 0;
									pval.p[i].fault = 0;
									}
								}
							if(write_program(&pval) == ESP_OK)
								{
								read_program(&dv_program);
								}
							}
						}
					}
    			}
    		}
    	}
    else if(strcmp(waterop_args.op->sval[0], "readps") == 0)
    	read_program_status(1);
    else if(strcmp(waterop_args.op->sval[0], "stop") == 0)
    	stop_program();
    else
    	ESP_LOGI(TAG, "Invalid op: %s", waterop_args.op->sval[0]);
    return 0;
    }
void water_mon_task(void *pvParameters)
	{
	struct tm tminfo;
	time_t ltime;
	int i, timem, ret, reset_done = 0;
	int saved_status = -1, savedDV = -1;
	char mqttbuf[256], buf[50];
	while(1)
		{
		ltime = time(NULL);
		localtime_r(&ltime, &tminfo);
		for(i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].dv >= 0 && dv_program.p[i].dv < DVCOUNT)
				{
				timem = tminfo.tm_hour * 60 + tminfo.tm_min;
				if(timem >= dv_program.p[i].starth * 60 + dv_program.p[i].startm &&
						timem < dv_program.p[i].stoph * 60 + dv_program.p[i].stopm &&
						dv_program.p[i].cs == NOT_STARTED)
					{
					ret = start_watering(i);
					if(ret != ESP_OK)
						ESP_LOGI(TAG, "Watering program for DV%d could not be started", dv_program.p[i].dv);
					sprintf(mqttbuf, "%s\1%d\1%d\1%d\1", WATERING_STATE, dv_program.p[i].dv, dv_program.p[i].cs, dv_program.p[i].fault);
					publish_monitor(mqttbuf, 1, 0);
					}
				else if(timem >= dv_program.p[i].stoph * 60 + dv_program.p[i].stopm &&
						dv_program.p[i].cs == IN_PROGRESS)
					{
					ret = stop_watering(i, COMPLETED);
					if(ret != ESP_OK)
						ESP_LOGI(TAG, "Watering program for DV%d could not be stopped", dv_program.p[i].dv);
					sprintf(mqttbuf, "%s\1%d\1%d\1%d\1", WATERING_STATE, dv_program.p[i].dv, dv_program.p[i].cs, dv_program.p[i].fault);
					publish_monitor(mqttbuf, 1, 0);
					}
				}
			}
		if(tminfo.tm_hour * 60 + tminfo.tm_min != RESET_PROGRAM_H * 60 + RESET_PROGRAM_M)
			reset_done = 0;
		if(tminfo.tm_hour == RESET_PROGRAM_H &&
				tminfo.tm_min == RESET_PROGRAM_M &&
				!reset_done)
			{
			read_program(&dv_program);
			for(i = 0; i < DVCOUNT; i++)
				dv_program.p[i].cs = NOT_STARTED;
			if(write_program(&dv_program) == ESP_OK)
				{
				reset_done = 1;
				ESP_LOGI(TAG, "Watering program status reset");
				}
			}
		if(saved_status != watering_status ||
					savedDV != activeDV)
			{
			if(watering_status == WATER_OFF)
				{
				ESP_LOGI(TAG, "Watering OFF");
				sprintf(mqttbuf, "%s\1woff\1", WATERING_STATE);
				}
			else
				{
				ESP_LOGI(TAG, "Watering ON on DV%d started @%02d:%02d", activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
				sprintf(mqttbuf, "%s\1won\1%d\1%d\1%d\1", WATERING_STATE, activeDV, dv_program.p[activeDV].starth, dv_program.p[activeDV].startm);
				}
			for(i = 0; i < DVCOUNT; i++)
				{
				if(dv_program.p[i].cs == NOT_STARTED)
					{
					ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
					sprintf(buf, "np\1%d%d%d\1", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
					strcat(mqttbuf, buf);
					}
				}
			publish_monitor(mqttbuf, 1, 0);
			saved_status = watering_status;
			savedDV = activeDV;
			}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}

static int get_act_state(int dvnum)
	{
	char op[20];
	int i, dv_current = 0, c_med, ret_state;
	int coff = 0;
	//try close
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_ON);
	//try "close" first
	strcpy(op, "close");
	if(dvnum == 0)
		gpio_set_level(PINEN_DV0, PIN_ON);
	else if(dvnum == 1)
		gpio_set_level(PINEN_DV1, PIN_ON);
	if(test_mode && timer_state == 0)
		{
		gptimer_set_raw_count(test_timer, 0);
		gptimer_start(test_timer);
		timer_state = 1;
		}
	vTaskDelay(500 / portTICK_PERIOD_MS);
	c_med = 0;
	for(i = 0; i < 3; i++)
		{
		vTaskDelay(500 / portTICK_PERIOD_MS);
		get_dv_adc_values(&dv_current);
		ESP_LOGI(TAG, "DV%d, %s %d", dvnum, op, dv_current);
		c_med += dv_current;
		}
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(PINEN_DV0, PIN_OFF);
	gpio_set_level(PINEN_DV1, PIN_OFF);
	c_med /= 3;
	if(c_med < CURRENT_OFF_LIM) // DV in closed state
		ret_state = DVCLOSE;
	else // DV is open -> move back to "open" state
		{
		ret_state = DVOPEN;
		//revert op
		gpio_set_level(PINMOT_A1, PIN_ON);
		gpio_set_level(PINMOT_B1, PIN_OFF);
		for(i = 0; i < 30; i++)
			{
			vTaskDelay(500 / portTICK_PERIOD_MS);
			if((ret_state = get_dv_adc_values(&dv_current)) == ESP_OK)
				{
				if(dv_current < CURRENT_OFF_LIM)
					coff++;
				else
					coff = 0;

				if(coff >= CURRENT_OFF_COUNT)
					break;
				}
			else
				break;
			}
		gpio_set_level(PINMOT_A1, PIN_OFF);
		gpio_set_level(PINMOT_B1, PIN_OFF);
		gpio_set_level(PINEN_DV0, PIN_OFF);
		gpio_set_level(PINEN_DV1, PIN_OFF);
		if(dv_current > CURRENT_OFF_LIM)
			{
			ESP_LOGI(TAG, "open DV%d failure %d %d", dvnum, i, dv_current);
			activeDV =  -1;
			dvconfig[dvnum].state = DVSTATE_FAULT;
			ret_state = DV_OPEN_FAIL;
			}
		}
	return ret_state;
	}

void register_waterop()
	{
	water_task_handle = NULL;
	config_dv_gpio();
	//config_cmd_timer();
	test_mode = 0;
	timer_state = -1;
	dvconfig[0].dvno = DV0;
	dvconfig[1].dvno = DV1;
	close_dv(0);
	close_dv(1);

   	activeDV = -1;
   	//get_dv_state();
	watering_status = WATER_OFF;
	waterop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation");
	waterop_args.dv = arg_int0(NULL, NULL, "<#DV>", "DV number");
	waterop_args.start = arg_str0(NULL, NULL, "hh:mm", "start time");
	waterop_args.stop = arg_str0(NULL, NULL, "hh:mm", "end time");
    waterop_args.end = arg_end(1);
    const esp_console_cmd_t dvop_cmd =
    	{
        .command = "dv",
        .help = "dv # open | close",
        .hint = NULL,
        .func = &do_dvop,
        .argtable = &waterop_args
    	};
    ESP_ERROR_CHECK(esp_console_cmd_register(&dvop_cmd));
    publish_reqID();
    read_program(&dv_program);
	xTaskCreate(water_mon_task, "wmon_task", 8192, NULL, 5, &water_task_handle);
	if(!water_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start watering monitor task");
		esp_restart();
		}
	}
static int read_program_status(int mq)
	{
	char bufr[64];
	char mqttbuf[128];
	esp_err_t ret;
	int i;
	struct stat st;
	//int yst, mst, dst, hst, minst,
	int dv, cs, fst, qwst;
	for (i = 0; i < DVCOUNT; i++)
		last_wstatus[i].dv = -1;

	ret = ESP_FAIL;
	if (stat(BASE_PATH"/"STATUS_FILE, &st) == 0)
		{
		FILE *f = fopen(BASE_PATH"/"STATUS_FILE, "r");
		if (f != NULL)
			{
			i = 0;
			if(mq)
				{
				sprintf(mqttbuf, "%s\1start\1", PROG_HISTORY);
				publish_state(mqttbuf, 1, 0);
				}
			struct tm tinfo;
			while(!feof(f))
				{
				fgets(bufr, 64, f);
				if(feof(f))
					break;
				strptime(bufr, "%Y-%m-%dT%H:%M:%S", &tinfo);
				//char *br = strptime(bufr, "%F", &tinfo);
				//if(br)
					{
					sscanf(bufr + 20, "%d %d %d %d", &dv, &cs, &fst, &qwst);
					if(dv >=0 && dv <DVCOUNT)
						{
						last_wstatus[dv].year = tinfo.tm_year; last_wstatus[dv].mon = tinfo.tm_mon; last_wstatus[dv].day = tinfo.tm_mday;
						last_wstatus[dv].hour = tinfo.tm_hour; last_wstatus[dv].min = tinfo.tm_min; last_wstatus[dv].sec = tinfo.tm_sec;
						last_wstatus[dv].dv = dv; last_wstatus[dv].cs = cs; last_wstatus[dv].fault = fst;
						last_wstatus[dv].qwater = qwst;
						}
					}
				if(mq)
					{
					sprintf(mqttbuf, "%s\1%s\1", PROG_HISTORY, bufr);
					publish_state(mqttbuf, 1, 0);
					}
				i++;
				}
			if(mq)
				{
				sprintf(mqttbuf, "%s\1end\1%d\1", PROG_HISTORY, i);
				publish_state(mqttbuf, 1, 0);
				}
			ret = ESP_OK;
			fclose(f);
			}
		else
			{
			sprintf(mqttbuf, "%s\1err_open\1", PROG_HISTORY);
			publish_state(mqttbuf, 1, 0);
			}
		}
	else
		{
		sprintf(mqttbuf, "%s\1not_found\1", PROG_HISTORY);
		publish_state(mqttbuf, 1, 0);
		ret = ESP_ERR_NOT_FOUND;
		}
	return ret;
	}

static int read_program(dvprogram_t *param_val)
	{
	char buf[64];
	esp_err_t ret;
    struct stat st;
    ret = ESP_FAIL;
    memset(param_val, 0, sizeof(dvprogram_t));
    for(int i = 0; i < DVCOUNT; i++)
    	{
		param_val->p[i].cs = -1;
		param_val->p[i].dv = -1;
    	}
	if (stat(BASE_PATH"/"PROGRAM_FILE, &st) != 0)
		{
		// file does no exists
		ret = ESP_FAIL;
		}
	else
		{
		FILE *f = fopen(BASE_PATH"/"PROGRAM_FILE, "r");
		if (f != NULL)
			{
			int dv, starth, startm, stoph, stopm, cs, fault;
			for(int i = 0; i < DVCOUNT; i++)
				{
				if(fgets(buf, 64, f))
					{
					sscanf(buf, "%d %d %d %d %d %d %d", &dv, &starth, &startm, &stoph, &stopm, &cs, &fault);
					param_val->p[i].dv = dv;
					param_val->p[i].starth = starth;
					param_val->p[i].startm = startm;
					param_val->p[i].stoph = stoph;
					param_val->p[i].stopm = stopm;
					param_val->p[i].cs = cs;
					param_val->p[i].fault = fault;
					}
				}
			fclose(f);
			ret = ESP_OK;
			}
		else
			{
			ESP_LOGE(TAG, "Failed to open program file for reading");
			return ret;
			}
		}
    return ret;
	}
static int write_program(dvprogram_t *param_val)
	{
	char buf[64];
	esp_err_t ret;
    ret = ESP_OK;
	FILE *f = fopen(BASE_PATH"/"PROGRAM_FILE, "w");
	if (f == NULL)
		{
		ESP_LOGE(TAG, "Failed to create dv program file");
		ret = ESP_FAIL;
		}
	else
		{
		for(int i = 0; i < DVCOUNT; i++)
			{
			if(param_val->p[i].starth * 60 +  param_val->p[i].startm > param_val->p[i].stoph * 60 + param_val->p[i].stopm)
				{
				param_val->p[i].cs = INVALID;
				ESP_LOGI(TAG, "Start time is after stop time for DV%d program : INVALID", param_val->p[i].dv);
				}
			else
				{
				sprintf(buf, "%2d %2d %2d %2d %2d %2d %d\n", param_val->p[i].dv, param_val->p[i].starth, param_val->p[i].startm,
															 param_val->p[i].stoph, param_val->p[i].stopm, param_val->p[i].cs, param_val->p[i].fault);
				if(fputs(buf, f) == EOF)
					{
					ret = ESP_FAIL;
					break;
					}
				}
			}
		fclose(f);
		}
	return ret;
	}
static int write_status(int idx)
	{
	int ret;
	struct tm tminfo;
	char strtime[100], dvpbuf[50];
	time_t ltime;
	ltime = time(NULL);
	localtime_r(&ltime, &tminfo);
	strftime(strtime, sizeof(strtime), "%Y-%m-%dT%H:%M:%S", &tminfo);
	sprintf(dvpbuf, " %d %d %d\n", dv_program.p[idx].dv, dv_program.p[idx].cs, dv_program.p[idx].fault);
	strcat(strtime, dvpbuf);
	FILE *f = fopen(BASE_PATH"/"STATUS_FILE, "a");
	if (f == NULL)
		{
		ESP_LOGE(TAG, "Failed to open status file for append");
		ret = ESP_FAIL;
		}
	else
		{
		if(fputs(strtime, f) >= 0)
			ret = ESP_OK;
		else
			{
			ESP_LOGE(TAG, "Cannot update status file");
			ret = ESP_FAIL;
			}
		fclose(f);
		}
	return ret;
	}
static int start_watering(int idx)
	{
	int ret = START_WATERING_ERROR;
	int dvop;

	close_dv(0);
	close_dv(1);
	activeDV = -1;
	if(test_mode == 2)
		{
		dv_program.p[idx].cs = IN_PROGRESS;
		dv_program.p[idx].fault = 0;
		watering_status = WATER_ON;
		ESP_LOGI(TAG, "Watering program for DV%d started", dv_program.p[idx].dv);
		activeDV = dv_program.p[idx].dv;
		write_program(&dv_program);
		write_status(idx);
		return 0;
		}
	//set pump online
	if(pump_operational(PUMP_ONLINE) == ESP_OK)
		{
		//wait 1 sec for pressure > max limit
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		if(pump_pressure_kpa > pump_max_lim)
			{
			dvop = open_dv(dv_program.p[idx].dv);
			if(dvop == ESP_OK)
				{
				ESP_LOGI(TAG, "open DV%d OK", dv_program.p[idx].dv);
				//now wait for pressure to fall between max and min limit
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				if(pump_pressure_kpa > pump_max_lim)
					{
					ESP_LOGI(TAG, "Error! DV%d - pump pressure too high: %d", dv_program.p[idx].dv, pump_pressure_kpa);
					dv_program.p[idx].cs = START_ERROR;
					dv_program.p[idx].fault = PRESS2HIGH;
					//revert
					pump_operational(PUMP_OFFLINE);
					close_dv(dv_program.p[idx].dv);
					}
				else if(pump_pressure_kpa < MINPRES)
					{
					ESP_LOGI(TAG, "Error! pump pressure too low: %d", pump_pressure_kpa);
					dv_program.p[idx].cs = START_ERROR;
					dv_program.p[idx].fault = PRESS2LOW;
					//revert
					pump_operational(PUMP_OFFLINE);
					close_dv(dv_program.p[idx].dv);
					}
				else
					{
					dv_program.p[idx].cs = IN_PROGRESS;
					dv_program.p[idx].fault = 0;
					watering_status = WATER_ON;
					ESP_LOGI(TAG, "Watering program for DV%d started", dv_program.p[idx].dv);
					activeDV = dv_program.p[idx].dv;
					ret = 0;
					}
				}
			else
				{
				ESP_LOGI(TAG, "Error open! DV%d: %d", dv_program.p[idx].dv, dvop);
				dv_program.p[idx].cs = START_ERROR;
				dv_program.p[idx].fault = dvop;
				//revert
				close_dv(dv_program.p[idx].dv);
				pump_operational(PUMP_OFFLINE);
				}
			}
		else
			{
			ESP_LOGI(TAG, "Error! pump pressure too low before open DV: %d", pump_pressure_kpa);
			dv_program.p[idx].cs = START_ERROR;
			dv_program.p[idx].fault = PRESS2LOWDVCLOSED;
			//revert
			close_dv(dv_program.p[idx].dv);
			pump_operational(PUMP_OFFLINE);
			}
		}
	else
		{
		dv_program.p[idx].cs = START_ERROR;
		dv_program.p[idx].fault = FAULT_PUMP;
		ESP_LOGI(TAG, "Error setting pump ONLINE! DV%d", dv_program.p[idx].dv);
		pump_operational(PUMP_OFFLINE);
		}
	write_program(&dv_program);
	write_status(idx);
	return ret;
	}

static int stop_watering(int idx, int reason)
	{
	int ret = STOP_WATERING_ERROR;
	if(test_mode == 2)
		{
		watering_status = WATER_OFF;
		dv_program.p[idx].cs = reason;
		dv_program.p[idx].fault = 0;
		write_program(&dv_program);
		write_status(idx);
		return ESP_OK;
		}
	if(dv_program.p[idx].cs == IN_PROGRESS)
		{
		ESP_LOGI(TAG, "Stop watering");
		ret = close_dv(dv_program.p[idx].dv);
		if(ret == ESP_OK)
			{
			dv_program.p[idx].cs = reason;
			dv_program.p[idx].fault = 0;
			}
		else
			{
			dv_program.p[idx].cs = STOP_ERROR;
			dv_program.p[idx].fault = ret;
			}
		}
	pump_operational(PUMP_OFFLINE);
	watering_status = WATER_OFF;
	write_program(&dv_program);
	write_status(idx);
	return ret;
	}

int get_water_values(last_status_t *lst, dvprogram_t *dvprog)
	{
	int i, ret = ESP_FAIL;
	if((ret = read_program_status(0)) == ESP_OK)
		{
		for(i = 0; i < DVCOUNT; i++)
			{
			lst[i].dv = last_wstatus[i].dv;
			lst[i].cs = last_wstatus[i].cs;
			lst[i].fault = last_wstatus[i].fault;
			lst[i].qwater = last_wstatus[i].qwater;
			lst[i].year = last_wstatus[i].year;
			lst[i].mon = last_wstatus[i].mon;
			lst[i].day = last_wstatus[i].day;
			lst[i].hour = last_wstatus[i].hour;
			lst[i].min = last_wstatus[i].min;
			lst[i].sec = last_wstatus[i].sec;
			}
		}
	for(i = 0; i < DVCOUNT; i++)
		{
		dvprog->p[i].cs = dv_program.p[i].cs;
		dvprog->p[i].fault = dv_program.p[i].fault;
		dvprog->p[i].starth = dv_program.p[i].starth;
		dvprog->p[i].startm = dv_program.p[i].startm;
		dvprog->p[i].stoph = dv_program.p[i].stoph;
		dvprog->p[i].stopm = dv_program.p[i].stopm;
		dvprog->p[i].dv = dv_program.p[i].dv;
		dvprog->p[i].qwater = dv_program.p[i].qwater;
		}
	return ret;
	}

static bool IRAM_ATTR test_timer_callback(gptimer_handle_t c_timer, const gptimer_alarm_event_data_t *edata, void *args)
	{
    BaseType_t high_task_awoken = pdFALSE;
    gptimer_stop(c_timer);
    timer_state = 0;
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}


static void config_test_mode()
	{
	if(timer_state == -1)
		{
		test_timer = NULL;
		gptimer_config_t gptconf = 	{
									.clk_src = GPTIMER_CLK_SRC_DEFAULT,
									.direction = GPTIMER_COUNT_UP,
									.resolution_hz = 1000000,					//1 usec resolution

									};
		gptimer_alarm_config_t al_config = 	{
											.reload_count = 0,
											.alarm_count = 5000000,
											.flags.auto_reload_on_alarm = true,
											};

		gptimer_event_callbacks_t cbs = {.on_alarm = &test_timer_callback,}; // register user callback
		ESP_ERROR_CHECK(gptimer_new_timer(&gptconf, &test_timer));
		ESP_ERROR_CHECK(gptimer_set_alarm_action(test_timer, &al_config));
		ESP_ERROR_CHECK(gptimer_register_event_callbacks(test_timer, &cbs, NULL));
		ESP_ERROR_CHECK(gptimer_enable(test_timer));
		}
	timer_state = 0;
	}
#endif

