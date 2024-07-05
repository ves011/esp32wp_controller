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

extern int pump_max_lim, pump_min_lim, pump_pressure_kpa;

//static gptimer_handle_t test_timer;
//static int timer_state;
QueueHandle_t water_cmd_q = NULL;

static struct {
    struct arg_str *op;
    struct arg_int *dv;
    struct arg_int *w_count;
    struct arg_str *start;
    struct arg_str *stop;
    struct arg_int *qwater;
    struct arg_end *end;
} waterop_args;

static const char *TAG = "WATER OP";
static TaskHandle_t water_task_handle, water_cmd_task_handle;
static int watering_status;
static uint64_t qwater = 0, qwater_start = 0;
//static last_status_t last_wstatus[DVCOUNT];
//static last_status2_t last_wstatus2[DVCOUNT];
//static int pump_present, pstate, pstatus, ppressure, pminlim, pmaxlim;
//static volatile uint64_t last_pump_state, last_pump_mon;
static dvprogram_t dv_program;
static int dvop_progress;
dvconfig_t dvconfig[DVCOUNT];
int wpday;
int activeDV, activeNO;
extern RTC_NOINIT_ATTR  int total_qwater; //defined in pumpop.c

static int read_program_status(int mq);
static int read_program(dvprogram_t *param_val);
static int write_program(dvprogram_t *param_val);
static int start_watering(int idx, int w_count);
static int stop_watering(int idx, int no, int reason);
static int write_status(int idx, int no);
static int get_act_state(int dvnum);
//static int test_mode;
//static void config_test_mode();
int get_dv_adc_values(int *dv_mv)
	{
	int16_t dvv[5];
	int ret = ESP_OK;
	/*
	if(test_mode)
		{
		if(timer_state == 1)
			*dv_mv = 150;
		else
			*dv_mv = 0;
		return ret;
		}
	*/
	if(adc_get_data(MOTSENSE_CHN, dvv, 5) == ESP_OK)
		*dv_mv = (dvv[0] + dvv[1] + dvv[2]  + dvv[3]  + dvv[4]) / 5;
	else
		ret = ESP_FAIL;
	//ESP_LOGI(TAG, "adc dv %d", *dv_mv);
	return ret;
	}

int open_dv(int dvnum)
	{
	int dv_current, op_start = 0, ret = ESP_OK;
	int coff = 0, i;
	msg_t msg_ui;
	char mqttbuf[256];

	dvop_progress = DVOP_INPROGRESS;
	gpio_set_level(PINMOT_A1, PIN_ON);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(dvconfig[dvnum].pin_enable, PIN_ON);

	/*
	 * loop until measured current is < CURRENT_OFF_LIM
	 * 60 loops ~ 30 sec @12V
	 */
	for(i = 0; i < 60; i++)
		{
		if((ret = get_dv_adc_values(&dv_current)) == ESP_OK)
			{
			if(dv_current < DV_CURRENT_OFF_LIM)
				coff++;
			else
				{
				coff = 0;
				op_start = 1;
				msg_ui.val = dvnum;
				msg_ui.source = WATER_DV_OP;
				xQueueSend(ui_cmd_q, &msg_ui, 0);
				sprintf(mqttbuf, "%s\1%d\1%d\1", DVOP, dvnum, dvconfig[dvnum].state);
				publish_monitor_a(mqttbuf, 1, 0);
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				}
			if(coff >= DV_CURRENT_OFF_COUNT)
				break;
			}
		else
			break;
		}
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(dvconfig[dvnum].pin_enable, PIN_OFF);
	if(ret == ESP_OK)
		{
		if(op_start) //dv current started
			{
			if(dv_current < DV_CURRENT_OFF_LIM) // open dv OK
				{
				ESP_LOGI(TAG, "open DV%d OK %d %d", dvnum, i, dv_current);
				activeDV =  dvconfig[dvnum].dvno;
				dvconfig[dvnum].state = DVOPEN;
				ret = ESP_OK;
				}
			else // still dv consumes current after 30 secs --> open failure
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
	dvop_progress = DVOP_STEADY;
	msg_ui.source = WATER_VAL_CHANGE;
	msg_ui.val = dvnum;
	xQueueSend(ui_cmd_q, &msg_ui, 0);
	sprintf(mqttbuf, "%s\1%d\1%d\1", DVSTATE, dvnum, dvconfig[dvnum].state);
	publish_monitor_a(mqttbuf, 1, 0);
	return ret;
	}
int close_dv(int dvnum)
	{
	int dv_current, op_start = 0, ret = ESP_OK;
	int coff = 0, i;
	msg_t msg_ui;
	char mqttbuf[256];

	dvop_progress = DVOP_INPROGRESS;
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_ON);
	gpio_set_level(dvconfig[dvnum].pin_enable, PIN_ON);
	for(i = 0; i < 60; i++)
		{
		if((ret = get_dv_adc_values(&dv_current)) == ESP_OK)
			{
			if(dv_current < DV_CURRENT_OFF_LIM)
				coff++;
			else
				{
				op_start = 1;
				coff = 0;
				msg_ui.source = WATER_DV_OP;
				msg_ui.val = dvnum;
				xQueueSend(ui_cmd_q, &msg_ui, 0);
				sprintf(mqttbuf, "%s\1%d\1%d\1", DVOP, dvnum, dvconfig[dvnum].state);
				publish_monitor_a(mqttbuf, 1, 0);
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				}

			if(coff >= DV_CURRENT_OFF_COUNT)
				break;
			}
		else
			break;
		}
	gpio_set_level(PINMOT_A1, PIN_OFF);
	gpio_set_level(PINMOT_B1, PIN_OFF);
	gpio_set_level(dvconfig[dvnum].pin_enable, PIN_OFF);
	if(ret == ESP_OK)
		{
		if(op_start) //dv current started
			{
			if(dv_current < DV_CURRENT_OFF_LIM) // close dv OK
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
	dvop_progress = DVOP_STEADY;
	msg_ui.source = WATER_VAL_CHANGE;
	msg_ui.val = dvnum;
	xQueueSend(ui_cmd_q, &msg_ui, 0);
	sprintf(mqttbuf, "%s\1%d\1%d\1", DVSTATE, dvnum, dvconfig[dvnum].state);
	publish_monitor_a(mqttbuf, 1, 0);
	return ret;
	}
static void config_dv_gpio(void)
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 	(1ULL << PINMOT_A1) | (1ULL << PINMOT_B1) | (1ULL << PINEN_DV0) | (1ULL << PINEN_DV1) | (1ULL << PINEN_DV2)| (1ULL << PINEN_DV3);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(PINEN_DV0, PIN_OFF);
    gpio_set_level(PINEN_DV1, PIN_OFF);
    gpio_set_level(PINEN_DV2, PIN_OFF);
    gpio_set_level(PINEN_DV3, PIN_OFF);
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
		for(int j = 0; j < wpday; j++)
			{
			if(dv_program.p[i].w_prog[j].cs == IN_PROGRESS)
				stop_watering(i, j, ABORTED);
			}
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
    if(strcmp(argv[0], "dv"))
    	return 0;

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
    	ESP_LOGI(TAG, "Watering status: %-d / activeDV: %d - %d", watering_status, activeDV, activeNO);
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
    		ESP_LOGI(TAG, "Watering ON on DV%d - %d started @%02d:%02d", activeDV, activeNO, dv_program.p[activeDV].w_prog[activeNO].starth, dv_program.p[activeDV].w_prog[activeNO].startm);
    		}
		/*for(int i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].cs == NOT_STARTED)
				ESP_LOGI(TAG, "Next program to start for DV%d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].starth, dv_program.p[i].startm);
			}*/
		sprintf(mqttbuf, "%s\1%d\1%d\1%d\1", STATE_W, watering_status, activeDV, activeNO);
		strcat(mqttbuf, buf);
		/*
		for(int i = 0; i < DVCOUNT; i++)
		    {
			for(int j = 0; j < wpday ; j++)
				{
				sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1",
						dv_program.p[i].dv, dv_program.p[i].w_prog[j].no, dv_program.p[i].w_prog[j].starth, dv_program.p[i].w_prog[j].startm, dv_program.p[i].w_prog[j].stoph,
						dv_program.p[i].w_prog[j].stopm, dv_program.p[i].w_prog[j].qwater, dv_program.p[i].w_prog[j].cs, dv_program.p[i].w_prog[j].fault);
				strcat(mqttbuf, buf);
				ESP_LOGI(TAG, "DV%d program:  %d %d %d %d %d %d %d %d %d", i, dv_program.p[i].dv, dv_program.p[i].w_prog[j].no, dv_program.p[i].w_prog[j].starth, dv_program.p[i].w_prog[j].startm, dv_program.p[i].w_prog[j].stoph,
						dv_program.p[i].w_prog[j].stopm, dv_program.p[i].w_prog[j].qwater, dv_program.p[i].w_prog[j].cs, dv_program.p[i].w_prog[j].fault);;
				}
		    }
		    */
		publish_state_a(mqttbuf, 1, 0);
    	}
    else if(strcmp(waterop_args.op->sval[0], "resetps") == 0)
    	{
    	for(int i = 0; i < DVCOUNT; i++)
    		{
    		for(int j = 0; j < wpday && dv_program.p[i].w_prog[j].cs != PROG_SKIP; j++)
    			dv_program.p[i].w_prog[j].cs = NOT_STARTED;
    		}
    	write_program(&dv_program);
    	}
    else if(strcmp(waterop_args.op->sval[0], "program") == 0)
    	{
    	int starth, startm, stoph, stopm, i, j, w_count, lqwater = 0;
    	read_program(&dv_program);
    	memcpy(&pval, &dv_program, sizeof(dvprogram_t));
    	sprintf(mqttbuf, "%s\1", STATE_P);
    	if(waterop_args.dv->count == 0) // no params -> just show DVs program
    		{
    		for(i = 0; i < DVCOUNT; i++)
    			{
    			for(j = 0; j < wpday; j++)
    				{
					sprintf(buf, "%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1", dv_program.p[i].dv, dv_program.p[i].w_prog[j].no,
											dv_program.p[i].w_prog[j].starth, dv_program.p[i].w_prog[j].startm,
											dv_program.p[i].w_prog[j].stoph, dv_program.p[i].w_prog[j].stopm,
											dv_program.p[i].w_prog[j].qwater, dv_program.p[i].w_prog[j].cs, dv_program.p[i].w_prog[j].fault);
					strcat(mqttbuf, buf);
					ESP_LOGI(TAG, "DV%d program: %d %d %d %d %d %d %d %d %d", i, dv_program.p[i].dv, dv_program.p[i].w_prog[j].no,
											dv_program.p[i].w_prog[j].starth, dv_program.p[i].w_prog[j].startm,
											dv_program.p[i].w_prog[j].stoph, dv_program.p[i].w_prog[j].stopm,
											dv_program.p[i].w_prog[j].qwater, dv_program.p[i].w_prog[j].cs, dv_program.p[i].w_prog[j].fault);
    				}
    			}
    		publish_state_a(mqttbuf, 1, 0);
    		}
    	else
    		{
    		if(waterop_args.w_count->count == 0)
				ESP_LOGI(TAG, "program: watering number not provided");
			else
				{
				w_count = waterop_args.w_count->ival[0];
				if(w_count < 0 || w_count > 1)
					{
					ESP_LOGI(TAG, "watering number out of range");
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
								lqwater = waterop_args.qwater->ival[0];
								sscanf(waterop_args.start->sval[0], "%d:%d", &starth, &startm);
								sscanf(waterop_args.stop->sval[0], "%d:%d", &stoph, &stopm);
								if(starth >= 0 && starth <= 23 && stoph >= 0 && stoph <= 23 &&
										startm >= 0 && startm <= 59 && stopm >= 0 && stopm <= 59)
									{
									for(i = 0; i < DVCOUNT; i++)
										{
										if(pval.p[i].dv == waterop_args.dv->ival[0])
											{
											for(j = 0; j < wpday; j++)
												{
												if(pval.p[i].w_prog[j].no == w_count)
													{
													pval.p[i].w_prog[j].starth = starth;
													pval.p[i].w_prog[j].startm = startm;
													pval.p[i].w_prog[j].stoph = stoph;
													pval.p[i].w_prog[j].stopm = stopm;
													pval.p[i].w_prog[j].qwater = lqwater;
													if(pval.p[i].w_prog[j].cs != PROG_SKIP)
														pval.p[i].w_prog[j].cs = 0;
													pval.p[i].w_prog[j].fault = 0;
													break;
													}
												}
											break;
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
    		}
    	}
    else if(strcmp(waterop_args.op->sval[0], "readps") == 0)
    	read_program_status(1);
    else if(strcmp(waterop_args.op->sval[0], "stop") == 0)
    	stop_program();
    else if(strcmp(waterop_args.op->sval[0], "wpday") == 0)
    	{
    	int no;
    	if(waterop_args.w_count->count && waterop_args.w_count->ival[0] >= 1 && waterop_args.w_count->ival[0] <= 2)
    		{
    		no = waterop_args.w_count->ival[0];
   			for(int i = 0; i < DVCOUNT; i++)
				{
				if(dv_program.p[i].dv == waterop_args.dv->ival[0])
					{
					for(int j = 0; j < wpday; j++)
						{
						if(dv_program.p[i].w_prog[j].no + 1 > no)
							dv_program.p[i].w_prog[j].cs = PROG_SKIP;
						else
							dv_program.p[i].w_prog[j].cs = 0;
						}
					}
				}
   			write_program(&dv_program);
    		}
		else
			{
			ESP_LOGI(TAG, "no #watering / day specified or invalid value ");
			}
    	}
    else
    	ESP_LOGI(TAG, "Invalid op: %s", waterop_args.op->sval[0]);
    return 0;
    }
void water_mon_task(void *pvParameters)
	{
	struct tm tminfo;
	time_t ltime;
	int i, timem, ret, reset_done = 0;
	int saved_status = -1, savedDV = -1, saved_qwater = -1;
	char mqttbuf[256], buf[50];
	msg_t msg_ui;
	while(1)
		{
		ltime = time(NULL);
		localtime_r(&ltime, &tminfo);
		timem = tminfo.tm_hour * 60 + tminfo.tm_min;
// test if any program to be start or stop
		for(i = 0; i < DVCOUNT; i++)
			{
			if(dv_program.p[i].dv >= 0 && dv_program.p[i].dv < DVCOUNT)
				{
				for(int j = 0; j < wpday; j++)
					{
					if(timem >= dv_program.p[i].w_prog[j].starth * 60 + dv_program.p[i].w_prog[j].startm &&
							timem < dv_program.p[i].w_prog[j].stoph * 60 + dv_program.p[i].w_prog[j].stopm &&
							dv_program.p[i].w_prog[j].cs == NOT_STARTED)
						{
						ret = start_watering(i, j);
						if(ret != ESP_OK)
							{
							ESP_LOGI(TAG, "Watering program for DV%d - %d could not be started", dv_program.p[i].dv, dv_program.p[i].w_prog[j].no);
							qwater_start = total_qwater;
							qwater = 0;
							}
						sprintf(mqttbuf, "%s\1%d\1%d\1%d\1%d\1", WATERING_START, dv_program.p[i].dv, dv_program.p[i].w_prog[j].no, dv_program.p[i].w_prog[j].cs, dv_program.p[i].w_prog[j].fault);
						publish_monitor_a(mqttbuf, 1, 0);
						}
					else if(timem >= dv_program.p[i].w_prog[j].stoph * 60 + dv_program.p[i].w_prog[j].stopm &&
							dv_program.p[i].w_prog[j].cs == IN_PROGRESS)
						{
						ret = stop_watering(i, j, COMPLETED);
						if(ret != ESP_OK)
							ESP_LOGI(TAG, "Watering program for DV%d - %d could not be stopped", dv_program.p[i].dv, dv_program.p[i].w_prog[j].no);

						sprintf(mqttbuf, "%s\1%d\1%d\1%d\1%d\1", WATERING_STOP, dv_program.p[i].dv, dv_program.p[i].w_prog[j].no, dv_program.p[i].w_prog[j].cs, dv_program.p[i].w_prog[j].fault);
						publish_monitor_a(mqttbuf, 1, 0);
						}
					}
				}
			}
//reste program status which means all completion status will be set to NOT_STARTED
		if(tminfo.tm_hour * 60 + tminfo.tm_min != RESET_PROGRAM_H * 60 + RESET_PROGRAM_M)
			reset_done = 0;
		if(tminfo.tm_hour == RESET_PROGRAM_H &&
				tminfo.tm_min == RESET_PROGRAM_M &&
				!reset_done)
			{
			read_program(&dv_program);
			for(i = 0; i < DVCOUNT; i++)
				{
				for(int j = 0; j < wpday && dv_program.p[i].w_prog[j].cs != PROG_SKIP; j++)
					dv_program.p[i].w_prog[j].cs = NOT_STARTED;
				}
			if(write_program(&dv_program) == ESP_OK)
				{
				reset_done = 1;
				ESP_LOGI(TAG, "Watering program status reset");
				}
			}
		if(watering_status == WATER_ON)
			qwater = total_qwater - qwater_start;
		if(saved_status != watering_status ||
					savedDV != activeDV ||
					saved_qwater != qwater)
			{
			if(watering_status == WATER_OFF)
				{
				ESP_LOGI(TAG, "Watering OFF");
				sprintf(mqttbuf, "%s\1woff\1", WATERING_STATE);
				for(i = 0; i < DVCOUNT; i++)
					{
					for(int j = 0; j < wpday; j++)
						{
						if(dv_program.p[i].w_prog[j].cs == NOT_STARTED)
							{
							ESP_LOGI(TAG, "Next program to start for DV%d - %d @%02d:%02d", dv_program.p[i].dv, dv_program.p[i].w_prog[j].no, dv_program.p[i].w_prog[j].starth, dv_program.p[i].w_prog[j].startm);
							sprintf(buf, "np\1%d\1%d\1%d\1%d\1", dv_program.p[i].dv, dv_program.p[i].w_prog[j].no, dv_program.p[i].w_prog[j].starth, dv_program.p[i].w_prog[j].startm);
							strcat(mqttbuf, buf);
							}
						}
					}
				}
			else
				{
				dv_program.p[activeDV].w_prog[activeNO].qwater = qwater / 1000;
				ESP_LOGI(TAG, "Watering ON on DV%d - %d started @%02d:%02d - %llu", activeDV, activeNO, dv_program.p[activeDV].w_prog[activeNO].starth, dv_program.p[activeDV].w_prog[activeNO].startm, qwater / 1000);
				sprintf(mqttbuf, "%s\1won\1%d\1%d\1%d\1%d\1%d\1", WATERING_STATE, activeDV, activeNO, dv_program.p[activeDV].w_prog[activeNO].starth, dv_program.p[activeDV].w_prog[activeNO].startm, dv_program.p[activeDV].w_prog[activeNO].qwater);
				}

			publish_monitor_a(mqttbuf, 1, 0);
			saved_status = watering_status;
			savedDV = activeDV;
			saved_qwater = qwater;
			msg_ui.source = WATER_VAL_CHANGE;
			xQueueSend(ui_cmd_q, &msg_ui, 0);
			//ESP_LOGI(TAG, "wmon_task loop");
			}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}

static void water_cmd_task(void *pvParameters)
	{
	msg_t msg;
	while(1)
		{
		if(xQueueReceive(water_cmd_q, &msg, portMAX_DELAY))
			{
			if(msg.source == DVOPEN)
				open_dv(msg.val);
			else if(msg.source == DVCLOSE)
				close_dv(msg.val);
			}
		}
	}

static int get_act_state(int dvnum)
	{
	char op[20];
	int i, dv_current = 0, c_med, ret_state;
	int coff = 0;
	if(dvop_progress == DVOP_INPROGRESS)
		{
		ret_state = dvconfig[dvnum].state;
		}
	else
		{
		//try close
		gpio_set_level(PINMOT_A1, PIN_OFF);
		gpio_set_level(PINMOT_B1, PIN_ON);
		//try "close" first
		strcpy(op, "close");
		gpio_set_level(dvconfig[dvnum].pin_enable, PIN_ON);

		vTaskDelay(500 / portTICK_PERIOD_MS);
		c_med = 0;
		for(i = 0; i < 3; i++)
			{
			get_dv_adc_values(&dv_current);
			ESP_LOGI(TAG, "DV%d, %s %d", dvnum, op, dv_current);
			c_med += dv_current;
			}
		gpio_set_level(PINMOT_A1, PIN_OFF);
		gpio_set_level(PINMOT_B1, PIN_OFF);
		gpio_set_level(dvconfig[dvnum].pin_enable, PIN_OFF);
		c_med /= 3;
		if(c_med < DV_CURRENT_OFF_LIM) // DV in closed state
			ret_state = DVCLOSE;
		else // DV is open -> move back to "open" state
			{
			ret_state = DVOPEN;
			//revert op
			gpio_set_level(PINMOT_A1, PIN_ON);
			gpio_set_level(PINMOT_B1, PIN_OFF);
			gpio_set_level(dvconfig[dvnum].pin_enable, PIN_ON);
			for(i = 0; i < 30; i++)
				{
				if(get_dv_adc_values(&dv_current) == ESP_OK)
					{
					if(dv_current < DV_CURRENT_OFF_LIM)
						coff++;
					else
						{
						coff = 0;
						vTaskDelay(500 / portTICK_PERIOD_MS);
						}

					if(coff >= DV_CURRENT_OFF_COUNT)
						break;
					}
				else
					break;
				}
			gpio_set_level(PINMOT_A1, PIN_OFF);
			gpio_set_level(PINMOT_B1, PIN_OFF);
			gpio_set_level(dvconfig[dvnum].pin_enable, PIN_OFF);
			if(dv_current > DV_CURRENT_OFF_LIM)
				{
				ESP_LOGI(TAG, "open DV%d failure %d %d", dvnum, i, dv_current);
				activeDV =  -1;
				dvconfig[dvnum].state = DVSTATE_FAULT;
				ret_state = DV_OPEN_FAIL;
				}
			}
		dvconfig[dvnum].state = ret_state;
		}
	return ret_state;
	}

void register_waterop()
	{
	water_task_handle = NULL;
	water_cmd_q = xQueueCreate(10, sizeof(msg_t));
	config_dv_gpio();
	//config_cmd_timer();
	//test_mode = 0;
	//timer_state = -1;
	for(int i = 0; i < DVCOUNT; i++)
		{
		dvconfig[i].dvno = i;
		}
	dvconfig[0].pin_enable = PINEN_DV0;
	dvconfig[1].pin_enable = PINEN_DV1;
#if DVCOUNT == 4
	dvconfig[2].pin_enable = PINEN_DV2;
	dvconfig[3].pin_enable = PINEN_DV3;
#endif

   	activeNO = -1;
   	wpday = 2;
   	//get_dv_state();
	watering_status = WATER_OFF;
	waterop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation");
	waterop_args.dv = arg_int0(NULL, NULL, "<#DV>", "DV number");
	waterop_args.w_count = arg_int0(NULL, NULL, "<#watering>", "watering/day");
	waterop_args.start = arg_str0(NULL, NULL, "hh:mm", "start time");
	waterop_args.stop = arg_str0(NULL, NULL, "hh:mm", "end time");
	waterop_args.qwater = arg_int0(NULL, NULL, "<qwater>", "water quantity");
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
    for(int i = 0; i < DVCOUNT; i++)
    	{
    	dv_program.p[i].dv = i;
    	//last_wstatus2[i].dv = -1;
    	for(int j = 0; j < wpday; j++)
    		{
    		dv_program.p[i].w_prog[j].no = j;
    		dv_program.p[i].w_prog[j].cs = PROG_SKIP;
    		//last_wstatus2[i].last_no_b[j].cs = -1;
    		//last_wstatus2[i].last_no_e[j].cs = -1;
    		}
    	}
    read_program(&dv_program);
    read_program_status(0);
	xTaskCreate(water_mon_task, "wmon_task", 4096, NULL, 5, &water_task_handle);
	if(!water_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start watering monitor task");
		esp_restart();
		}
	xTaskCreate(water_cmd_task, "wcmd_task", 4096, NULL, 5, &water_cmd_task_handle);
	if(!water_cmd_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start watering cmd task");
		esp_restart();
		}
	int dvstate[DVCOUNT], i;
	msg_t msg;
	msg.source = DVCLOSE;
	for(i = 0; i < DVCOUNT; i++)
		dvstate[i] = get_act_state(dvconfig[i].dvno);

	for(i = 0; i < DVCOUNT; i++)
		{
		if(dvstate[i] != DVCLOSE)
			{
			msg.val = i;
			xQueueSend(water_cmd_q, &msg, 0);
			}
		}
	activeDV = -1;
	}

/*
 * program_status.txt structure
 * |  2    |    2    |   4    |    20      |    20    |     2      |     2    |
 * | dv no | prog no | qwater | start time | end time | end status | end fault|
 */
static int read_program_status(int mq)
	{
	char bufr[128];
	char mqttbuf[256];
	//last_no_status_t *last_no;
	esp_err_t ret;
	int i, dv_no, prog_no, lqwater, end_status, end_fault;
	char eff_start[30], eff_stop[30];
	struct stat st;
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
				publish_state_a(mqttbuf, 1, 0);
				}
			struct tm tinfo_start, tinfo_stop;
			while(!feof(f))
				{
				fgets(bufr, 64, f);
				if(feof(f))
					break;
				sscanf(bufr, "%2d %2d %4d %20s %20s %2d %2d", &dv_no, &prog_no, &lqwater, eff_start, eff_stop, &end_status, &end_fault);
				strptime(eff_start, "%Y-%m-%dT%H:%M:%S", &tinfo_start);
				strptime(eff_stop, "%Y-%m-%dT%H:%M:%S", &tinfo_stop);
				if(dv_no >= 0 && dv_no < DVCOUNT &&
						prog_no >= 0 && prog_no < wpday &&
						dv_no == dv_program.p[dv_no].dv &&
						dv_program.p[dv_no].w_prog[prog_no].no == prog_no)
					{
					memcpy(&dv_program.p[dv_no].w_prog[prog_no].eff_start, &tinfo_start, sizeof(struct tm));
					memcpy(&dv_program.p[dv_no].w_prog[prog_no].eff_stop, &tinfo_stop, sizeof(struct tm));
					dv_program.p[dv_no].w_prog[prog_no].cs = end_status;
					dv_program.p[dv_no].w_prog[prog_no].fault = end_fault;
					}
				if(mq)
					{
					sprintf(bufr, "%d\1%d\1%d\1%s\1%s\1%d\1%d\1", dv_no, prog_no, lqwater, eff_start, eff_stop, end_status, end_fault);
					sprintf(mqttbuf, "%s\1%s\1", PROG_HISTORY, bufr);
					publish_state_a(mqttbuf, 1, 0);
					}
				i++;
				}
			if(mq)
				{
				sprintf(mqttbuf, "%s\1end\1%d\1", PROG_HISTORY, i);
				publish_state_a(mqttbuf, 1, 0);
				}
			ret = ESP_OK;
			fclose(f);
			}
		else
			{
			sprintf(mqttbuf, "%s\1err_open\1", PROG_HISTORY);
			publish_state_a(mqttbuf, 1, 0);
			}
		}
	else
		{
		sprintf(mqttbuf, "%s\1not_found\1", PROG_HISTORY);
		publish_state_a(mqttbuf, 1, 0);
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
    //memset(param_val, 0, sizeof(dvprogram_t));
    for(int i = 0; i < DVCOUNT; i++)
    	{
    	for(int j = 0; j < wpday; j++)
    		{
			param_val->p[i].w_prog[j].cs = PROG_SKIP;
    		}
		//param_val->p[i].dv = -1;
    	}
	if (stat(BASE_PATH"/"PROGRAM_FILE, &st) != 0)
		{
		// file does no exists
		ret = ESP_FAIL;
		}
	else
		{
		int i, j, k;
		FILE *f = fopen(BASE_PATH"/"PROGRAM_FILE, "r");
		if (f != NULL)
			{
			int dv, starth, startm, stoph, stopm, cs, fault, w_no, lqwater;
			for(i = 0; i < DVCOUNT * wpday; i++)
				{
				if(fgets(buf, 64, f))
					{
					sscanf(buf, "%d %d %d %d %d %d %d %d %d", &dv, &w_no, &starth, &startm, &stoph, &stopm, &lqwater, &cs, &fault);
					if(dv >=0 && dv < DVCOUNT && w_no >= 0 && w_no < 2 )
						{
						for(j = 0; j < DVCOUNT; j++)
							{
							if(param_val->p[j].dv == dv)
								{
								for(k = 0; k < wpday; k++)
									{
									if(param_val->p[j].w_prog[k].no == w_no)
										break;
									}
								if(k < wpday)
									{
									param_val->p[j].dv = dv;
									param_val->p[j].w_prog[k].no = w_no;
									param_val->p[j].w_prog[k].starth = starth;
									param_val->p[j].w_prog[k].startm = startm;
									param_val->p[j].w_prog[k].stoph = stoph;
									param_val->p[j].w_prog[k].stopm = stopm;
									param_val->p[j].w_prog[k].qwater = lqwater;
									param_val->p[j].w_prog[k].cs = cs;
									param_val->p[j].w_prog[k].fault = fault;
									}
								break;
								}
							}
						}
					}
				else
					break;
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
		for(int i = 0; i < DVCOUNT && param_val->p[i].dv != -1; i++)
			{
			for(int j = 0; j < wpday; j++)
				{
				if(param_val->p[i].w_prog[j].cs != PROG_SKIP)
					{
					if(param_val->p[i].w_prog[j].starth * 60 +  param_val->p[i].w_prog[j].startm > param_val->p[i].w_prog[j].stoph * 60 + param_val->p[i].w_prog[j].stopm)
						{
						param_val->p[i].w_prog[j].cs = INVALID;
						ESP_LOGI(TAG, "Start time is after stop time for DV%d - %d program : INVALID", param_val->p[i].dv, param_val->p[i].w_prog[j].no);
						}
					}
				sprintf(buf, "%2d %2d %2d %2d %2d %2d %2d %2d %2d\n", param_val->p[i].dv, param_val->p[i].w_prog[j].no,
														 param_val->p[i].w_prog[j].starth, param_val->p[i].w_prog[j].startm,
														 param_val->p[i].w_prog[j].stoph, param_val->p[i].w_prog[j].stopm,
														 param_val->p[i].w_prog[j].qwater, param_val->p[i].w_prog[j].cs, param_val->p[i].w_prog[j].fault);
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
/*
 * program_status.txt structure
 * |  2    |    2    |   4    |    20      |    20    |     2      |     2    |
 * | dv no | prog no | qwater | start time | end time | end status | end fault|
 */
static int write_status(int idx, int no)
	{
	int ret;
	struct tm tminfo;
	//last_no_status_t *last_no = NULL;
	char strtime_b[25], strtime_e[25], dvpbuf[100];
	time_t ltime;
	ltime = time(NULL);
	localtime_r(&ltime, &tminfo);

	strftime(strtime_b, sizeof(strtime_b), "%Y-%m-%dT%H:%M:%S", &dv_program.p[idx].w_prog[no].eff_start);
	if(dv_program.p[idx].w_prog[no].eff_stop.tm_hour != -1)
		strftime(strtime_e, sizeof(strtime_e), "%Y-%m-%dT%H:%M:%S", &dv_program.p[idx].w_prog[no].eff_stop);
	else
		strcpy(strtime_e, "0000-00-00T00:00:00");

	sprintf(dvpbuf, "%2d %2d %4d %20s %20s, %2d %2d\n", dv_program.p[idx].dv, dv_program.p[idx].w_prog[no].no, dv_program.p[idx].w_prog[no].qwater,
										 strtime_b, strtime_e, dv_program.p[idx].w_prog[no].cs, dv_program.p[idx].w_prog[no].fault);
	/*
	if(dv_program.p[idx].w_prog[no].cs == IN_PROGRESS || dv_program.p[idx].w_prog[no].cs == START_ERROR)
		last_no = last_wstatus2[idx].last_no_b;
	if(dv_program.p[idx].w_prog[no].cs == COMPLETED || dv_program.p[idx].w_prog[no].cs == ABORTED || dv_program.p[idx].w_prog[no].cs == STOP_ERROR)
		last_no = last_wstatus2[idx].last_no_e;
	if(last_no)
		{
		last_wstatus2[idx].dv = dv_program.p[idx].dv;
		last_no[no].cs = dv_program.p[idx].w_prog[no].cs;
		last_no[no].day = tminfo.tm_mday;
		last_no[no].fault = dv_program.p[idx].w_prog[no].fault;
		last_no[no].hour = tminfo.tm_hour;
		last_no[no].min = tminfo.tm_min;
		last_no[no].mon = tminfo.tm_mon;
		last_no[no].qwater = dv_program.p[idx].w_prog[no].qwater;
		last_no[no].sec = tminfo.tm_sec;
		last_no[no].w_count = no;
		last_no[no].year = tminfo.tm_year % 100;
		}
		*/
	FILE *f = fopen(BASE_PATH"/"STATUS_FILE, "a");
	if (f == NULL)
		{
		ESP_LOGE(TAG, "Failed to open status file for append");
		ret = ESP_FAIL;
		}
	else
		{
		if(fputs(dvpbuf, f) >= 0)
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
static int start_watering(int idx, int w_count)
	{
	int ret = START_WATERING_ERROR;
	int dvop;

	for(int i = 0; i < DVCOUNT; i++)
		close_dv(i);
	activeDV = -1;
	time_t ltime;
	struct tm tminfo;
	ltime = time(NULL);
	localtime_r(&ltime, &tminfo);
	dv_program.p[idx].w_prog[w_count].qwater = 0;
	memcpy(&dv_program.p[idx].w_prog[w_count].eff_start, &tminfo, sizeof(struct tm));
	dv_program.p[idx].w_prog[w_count].eff_stop.tm_hour = -1;
	//set pump online
	if(pump_operational(PUMP_ONLINE) == ESP_OK)
		{
		//wait 1 sec for pressure > max limit
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		if(pump_pressure_kpa > pump_min_lim)
			{
			dvop = open_dv(dv_program.p[idx].dv);
			if(dvop == ESP_OK)
				{
				ESP_LOGI(TAG, "open DV%d OK", dv_program.p[idx].dv);
				//now wait for pressure to fall between max and min limit
				vTaskDelay(1000 / portTICK_PERIOD_MS);
				if(pump_pressure_kpa > pump_max_lim) //?????
					{
					ESP_LOGI(TAG, "Error! DV%d - pump pressure too high: %d", dv_program.p[idx].dv, pump_pressure_kpa);
					dv_program.p[idx].w_prog[w_count].cs = START_ERROR;
					dv_program.p[idx].w_prog[w_count].fault = PRESS2HIGH;
					//revert
					pump_operational(PUMP_OFFLINE);
					close_dv(dv_program.p[idx].dv);
					write_status(idx, w_count);
					}
				else if(pump_pressure_kpa < MINPRES)
					{
					ESP_LOGI(TAG, "Error! pump pressure too low: %d", pump_pressure_kpa);
					dv_program.p[idx].w_prog[w_count].cs = START_ERROR;
					dv_program.p[idx].w_prog[w_count].fault = PRESS2LOW;
					//revert
					pump_operational(PUMP_OFFLINE);
					close_dv(dv_program.p[idx].dv);
					write_status(idx, w_count);
					}
				else
					{
					dv_program.p[idx].w_prog[w_count].cs = IN_PROGRESS;
					dv_program.p[idx].w_prog[w_count].fault = 0;
					watering_status = WATER_ON;
					ESP_LOGI(TAG, "Watering program for DV%d - %d started", dv_program.p[idx].dv, w_count);
					activeDV = dv_program.p[idx].dv;
					activeNO = w_count;
					qwater_start = total_qwater;
					qwater = 0;
					ret = 0;
					}
				}
			else
				{
				ESP_LOGI(TAG, "Error open! DV%d: %d", dv_program.p[idx].dv, dvop);
				dv_program.p[idx].w_prog[w_count].cs = START_ERROR;
				dv_program.p[idx].w_prog[w_count].fault = dvop;
				//revert
				close_dv(dv_program.p[idx].dv);
				pump_operational(PUMP_OFFLINE);
				write_status(idx, w_count);
				}
			}
		else
			{
			ESP_LOGI(TAG, "Error! pump pressure too low before open DV: %d", pump_pressure_kpa);
			dv_program.p[idx].w_prog[w_count].cs = START_ERROR;
			dv_program.p[idx].w_prog[w_count].fault = PRESS2LOWDVCLOSED;
			//revert
			close_dv(dv_program.p[idx].dv);
			pump_operational(PUMP_OFFLINE);
			write_status(idx, w_count);
			}
		}
	else
		{
		dv_program.p[idx].w_prog[w_count].cs = START_ERROR;
		dv_program.p[idx].w_prog[w_count].fault = FAULT_PUMP;
		ESP_LOGI(TAG, "Error setting pump ONLINE! DV%d", dv_program.p[idx].dv);
		pump_operational(PUMP_OFFLINE);
		write_status(idx, w_count);
		}
	write_program(&dv_program);
	return ret;
	}

static int stop_watering(int idx, int no, int reason)
	{
	int ret = STOP_WATERING_ERROR;
	if(dv_program.p[idx].w_prog[no].cs == IN_PROGRESS)
		{
		ESP_LOGI(TAG, "Stop watering");
		ret = close_dv(dv_program.p[idx].dv);
		if(ret == ESP_OK)
			{
			dv_program.p[idx].w_prog[no].cs = reason;
			dv_program.p[idx].w_prog[no].fault = 0;
			}
		else
			{
			dv_program.p[idx].w_prog[no].cs = STOP_ERROR;
			dv_program.p[idx].w_prog[no].fault = ret;
			}
		}
	dv_program.p[idx].w_prog[no].qwater = qwater / 1000;
	pump_operational(PUMP_OFFLINE);
	watering_status = WATER_OFF;
	write_program(&dv_program);
	write_status(idx, no);
	return ret;
	}

void get_water_values(dvprogram_t *dvprog, int *dvstate)
	{
	//memcpy(lst, last_wstatus2, sizeof(last_status2_t));
	memcpy(dvprog, &dv_program, sizeof(dvprogram_t));
	for(int i = 0; i < DVCOUNT; i++)
		dvstate[i] = dvconfig[i].state;
	}

void get_water_dv_state(int *dvstate)
	{
	for(int i = 0; i < DVCOUNT; i++)
		dvstate[i] = dvconfig[i].state;
	}

#endif

