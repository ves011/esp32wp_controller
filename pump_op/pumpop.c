/*
 * pumpop.c
 *
 *  Created on: Jan 28, 2023
 *      Author: viorel_serbu
 */

/**
 * @file pumpop.c
 * @brief implementation for control logic of submersible water pump controller
 */

#include <stdio.h>
#include <string.h>
#include "sys/reent.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sys/stat.h"
#include "esp_spiffs.h"
#include "mqtt_client.h"
#include "project_specific.h"
#include "common_defines.h"
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
#include "pump_files.h"


#if ACTIVE_CONTROLLER == PUMP_CONTROLLER ||	ACTIVE_CONTROLLER == WP_CONTROLLER
static SemaphoreHandle_t pumpop_mutex;

int pump_min_lim, pump_max_lim, pump_pressure_kpa, pump_debit;
static volatile int pump_state, pump_status, pump_current, kpa0_offset, pump_current_limit, psensor_mv, void_run_count;
static TaskHandle_t pump_task_handle;

static time_t start_overp_time;
int overp_time_limit = 10;

static gptimer_handle_t qmeter_timer;
static int qmeter_pc, qmeter_pc_sec;
static uint64_t qmeter_ts, last_qmeter_ts;
RTC_NOINIT_ATTR  uint64_t total_qwater;
static int qcal_a, qcal_b;
static int saved_t_water;

static const char *TAG = "PUMP OP";
static QueueHandle_t pump_cmd_queue = NULL;

static uint32_t testModeCurrent, testModePress;
uint32_t loop;
static int read_qcal();

		/**
 * @brief pump command and parameters
 */
static struct
	{
    struct arg_str *op;
    struct arg_int *minP;
    struct arg_int *maxP;
    struct arg_int *faultC;
    struct arg_int *overpt;
	struct arg_int *vrc;
    struct arg_end *end;
	} pumpop_args;

void IRAM_ATTR qmeter_gpio_handler(void* arg)
	{
	qmeter_pc++;
	}

static bool IRAM_ATTR qmeter_timer_callback(gptimer_handle_t qm_timer, const gptimer_alarm_event_data_t *edata, void *args)
	{
    BaseType_t high_task_awoken = pdFALSE;
    qmeter_pc_sec = qmeter_pc;
    qmeter_pc = 0;
    qmeter_ts = esp_timer_get_time() / 1000000;
    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
	}

static void config_qmeter_timer()
	{
	qmeter_timer = NULL;
	gptimer_config_t gptconf = 	{
								.clk_src = GPTIMER_CLK_SRC_DEFAULT,
								.direction = GPTIMER_COUNT_UP,
								.resolution_hz = QMETER_FREQ_RES,					//1 msec resolution

								};
	gptimer_alarm_config_t al_config = 	{
										.reload_count = 0,
										.alarm_count = QMETER_MEAS_TIME,
										.flags.auto_reload_on_alarm = true,
										};
	gptimer_event_callbacks_t cbs = {.on_alarm = &qmeter_timer_callback,}; // register user callback
	ESP_ERROR_CHECK(gptimer_new_timer(&gptconf, &qmeter_timer));
	ESP_ERROR_CHECK(gptimer_set_alarm_action(qmeter_timer, &al_config));
	ESP_ERROR_CHECK(gptimer_register_event_callbacks(qmeter_timer, &cbs, NULL));
	ESP_ERROR_CHECK(gptimer_enable(qmeter_timer));
	gptimer_start(qmeter_timer);
	}

#ifdef ADC_AD7811
static int get_pump_adc_values(int *psensor_mv)
	{
	int ret = ESP_FAIL, i;
	int nr_samp = NR_SAMPLES_PC;
	int16_t *c_data, *c_data_filt, c_temp[NR_SAMPLES_PC], p_data[NR_SAMPLES_PS];
	int16_t k_min, k_max, id_min[10], id_max[10], max_val = 20;
	int16_t max_mv[20], min_mv[20];
	if(testModeCurrent > 0 && pump_status == PUMP_ONLINE)
		{
		pump_current = testModeCurrent;
		*psensor_mv = testModePress;
		return ESP_OK;
		}
	*psensor_mv = 0;
	pump_current = 0;
	//get pump current data
	if((ret = adc_get_data_7911(CURRENT_CHN, c_temp, nr_samp)) == ESP_OK)
		{
		//for(i = 0; i < NR_SAMPLES_PC; i++)
		//	ESP_LOGI(TAG, "%d %d", i, c_temp[i]);
		//get pressure sensor data
		nr_samp = NR_SAMPLES_PS;
		if((ret = adc_get_data_7911(SENSOR_CHN, p_data, nr_samp)) == ESP_OK)
			{
			//pressure sensor mv = mean of data
			*psensor_mv = 0;
			for(i = 0; i < NR_SAMPLES_PS; i++)
				{
				//ESP_LOGI(TAG, "%d %d", i, p_data[i]);
				*psensor_mv += p_data[i];
				}
			*psensor_mv /= NR_SAMPLES_PS;
			//smooth current data: 5pt simple average
			c_data = calloc(NR_SAMPLES_PC, sizeof(int16_t));
			c_data_filt = calloc(NR_SAMPLES_PC, sizeof(int16_t));
			for(i = 2; i < NR_SAMPLES_PC - 2; i++)
				{
				c_data[i] = (c_temp[i - 2] + c_temp[i -1] + c_temp[i] + c_temp[i + 1] + c_temp[i + 2]) / 5;
				}
			//1st derivative
			for(i = 3; i < NR_SAMPLES_PC - 2; i++)
				c_temp[i] = c_data[i] - c_data[i -1];
			//smooth 1st derivative: 5pt sample average
			for(i = 5; i < NR_SAMPLES_PC - 4; i++)
				c_data_filt[i] = (c_temp[i - 2] + c_temp[i -1] + c_temp[i] + c_temp[i + 1] + c_temp[i + 2]) / 5;
			//get the local extreme index
			for(i = 6, k_min = 0, k_max = 0; i < NR_SAMPLES_PC -4; i++)
				{
				if(c_data_filt[i] * c_data_filt[i - 1] < 0) // local extreme; 0 is also a local extreme but more difficult to localize
					{
					if(c_data_filt[i] < 0) //local max
						{
						if(k_max < max_val)
							{
							max_mv[k_max] = c_data[i];
							id_max[k_max++] = i;
							//ESP_LOGI(TAG, "max %d, %d", i, c_data[i]);
							}
						}
					else // local min
						{
						if(k_min < max_val)
							{
							min_mv[k_min] = c_data[i];
							id_min[k_min++] = i;
							//ESP_LOGI(TAG, "min %d, %d", i, c_data[i]);
							}
						}
					}
				else if(c_data_filt[i] * c_data_filt[i - 1] == 0)
					{
					if(c_data_filt[i] == 0)
						{
						if(c_data_filt[i - 1] > 0) //local max
							{
							if(k_max < max_val)
								{
								max_mv[k_max] = c_data[i];
								id_max[k_max++] = i;
								//ESP_LOGI(TAG, "max %d, %d", i, c_data[i]);
								}
							}
						else // local min
							{
							if(k_min < max_val)
								{
								min_mv[k_min] = c_data[i];
								id_min[k_min++] = i;
								//ESP_LOGI(TAG, "min %d, %d", i, c_data[i]);
								}
							}
						}
					// need to skip following 0s
					while(c_data_filt[i] * c_data_filt[i - 1] == 0 && i < NR_SAMPLES_PC -4)i++;
					}
				}
			ret = ESP_OK;
			if(k_min >= 2 && k_max >= 2)
				{
				//space apart between 2 consecutive extremes
				//50Hz signal sampled @SAMPLE_PERIOD (usec)
				int sp = 1000000 / 50 / SAMPLE_PERIOD;
				int amin = 0, amax = 0, l;
				for(i = 0, l = 0; i < k_min - 1; i++)
					{
					//ESP_LOGI(TAG, "min %d, %d, %d, %d, %d, %d", i, id_min[i], min_mv[i], l, amin, abs(id_min[i + 1] - id_min[i] - sp));
					if(abs(id_min[i + 1] - id_min[i] - sp) <= 1)
						{
						amin += min_mv[i];
						l++;
						}
					}
				if(l)
					amin /= l;
				for(i = 0, l = 0; i < k_max - 1; i++)
					{
					//ESP_LOGI(TAG, "max %d, %d, %d, %d, %d", i, id_max[i], max_mv[i], l, amax);
					if(abs(id_max[i + 1] - id_max[i] - sp) <= 1)
						{
						amax += max_mv[i];
						l++;
						}
					}
				if(l)
					amax /= l;
				if(amin && amax)
					{
					pump_current = (amax - amin) / 2;
					//acs712 20A --> sensitivity = 100mV / A
					pump_current = (pump_current * 10000) / 1414;
					}
				else
					pump_current = 0;
				//ESP_LOGI(TAG, "pump_current: %d  p_sensor: %d / %d %d %d", pump_current, *psensor_mv, l, amin, amax);
				}
			else
				{
				pump_current = 0;
				//ESP_LOGI(TAG, "pump_current error %d %d", min_idx, max_idx);
				}
			free(c_data);
			free(c_data_filt);
			}
		}
	return ret;
	}
#endif

static void config_pump_gpio(void)
	{
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PUMP_ONOFF_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    /*
     * GPIO + relay (relay datasheet -> 40mA@5V)
     * GPIO_DRIVE_CAP_3 dI ~120 mA ->
     * GPIO_DRIVE_CAP_0 dI ~45mA
     */
    //gpio_set_drive_capability(PUMP_ONOFF_PIN, GPIO_DRIVE_CAP_0);
    gpio_config(&io_conf);
    gpio_set_level(PUMP_ONOFF_PIN, PIN_OFF);

	io_conf.intr_type = GPIO_INTR_POSEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_QMETER);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    gpio_isr_handler_add(PIN_NUM_QMETER, qmeter_gpio_handler, (void*) PIN_NUM_QMETER);

#ifdef LEDS
	/*
	 * output GPIOs
	 */
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PUMP_ONOFF_LED | 1ULL << PUMP_ONLINE_LED | 1ULL << PUMP_FAULT_LED);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    gpio_config(&io_conf);
    gpio_set_level(PUMP_ONOFF_LED, PIN_OFF);
    gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
    gpio_set_level(PUMP_FAULT_LED, PIN_OFF);


    /*
     * input GPIO
     */
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
	io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PUMP_ONLINE_CMD);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //gpio_install_isr_service(0);
    gpio_isr_handler_add(PUMP_ONLINE_CMD, pump_onoff_isr_handler, (void*) PUMP_ONLINE_CMD);
#endif
	}

int get_pump_state(void)
	{
	esp_err_t ret = ESP_FAIL;
    pump_limits_t plimits;
    int v_offset;
    int ps;
    char msg[80];
    if(rw_poffset(PARAM_READ, &v_offset) == ESP_OK)
    //if(rw_params(PARAM_READ, PARAM_V_OFFSET, &offset) == ESP_OK)
    	{
    	kpa0_offset = v_offset;
    	}
    else
    	{
    	kpa0_offset = 0;
    	}
    read_qcal();
    rw_plimits(PARAM_READ, &plimits);
   	//rw_params(PARAM_READ, PARAM_LIMITS, &plimits);
   	pump_min_lim = plimits.min_val;
	pump_max_lim = plimits.max_val;
	pump_current_limit = plimits.faultc;
	overp_time_limit = plimits.overp_lim;
	void_run_count = plimits.void_run_count;
	rw_poperational(PARAM_READ, &ps);
	//rw_params(PARAM_READ, PARAM_OPERATIONAL, &ps);
	pump_status = ps;
	if(!pump_task_handle) // else values are updated every 2sec by pump_mon_task
		{
		int local_ps_mv;
		ret = get_pump_adc_values(&local_ps_mv);
		if(ret == ESP_OK)
			{
			psensor_mv = local_ps_mv;
			pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
			if(pump_pressure_kpa < 0)
				pump_pressure_kpa = 0;
			}
		else
			{
			pump_pressure_kpa = -1;
			psensor_mv = -1;
			pump_current = -1;
			}
		}
	//else
	//	{
	//	//just wait pump mon task to compute the values
	//	if(xSemaphoreTake(pumpval_mutex, ( TickType_t ) 100 )) // 1 sec wait
	//		{
	//		xSemaphoreGive(pumpval_mutex);
	//		pdsem = 2;
	//		}
	//	}
	char opstate[20];
	if(pump_status == PUMP_ONLINE)
		{
		strcpy(opstate, "ONLINE");
#ifdef LEDS
		gpio_set_level(PUMP_ONLINE_LED, PIN_ON);
#endif
		}
	else if(pump_status == PUMP_OFFLINE)
		{
		strcpy(opstate, "OFFLINE");
#ifdef LEDS
		gpio_set_level(PUMP_ONLINE_LED, PIN_OFF);
#endif
		}
	else
		strcpy(opstate, "UNKNOWN");
	ESP_LOGI(TAG, "\n \
running state\t\t= %5d\n \
Operational state\t\t= %s\n \
pressure (kPa/mV)        = %5d/%5d\n \
min pressure (kPa)       = %5d\n \
max pressure (kPa)       = %5d\n \
fault current limit (mA) = %5d\n \
0 kPa offset (mV)        = %5d\n \
current (mA)             = %5d\n \
timeout P max (sec)      = %5d\n \
short run count          = %5d\n \
debit                    = %5d\n \
toal water               = %llu\n"
			,
			pump_state, opstate, pump_pressure_kpa, psensor_mv, pump_min_lim, pump_max_lim, pump_current_limit, kpa0_offset, pump_current, overp_time_limit, void_run_count, pump_debit, total_qwater);
	sprintf(msg, "%s\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%d\1%llu",
			PUMP_STATE, pump_state, pump_status, pump_current, pump_pressure_kpa, kpa0_offset, pump_min_lim, pump_max_lim, pump_current_limit, overp_time_limit, void_run_count, pump_debit, total_qwater);
	publish_topic(TOPIC_STATE, msg, 0, 0);
	msg_t msg_ui;
	msg_ui.source = PUMP_VAL_CHANGE;
	xQueueSend(ui_cmd_q, &msg_ui, 0);
    return ret;
    }

void get_pump_values(int *p_state, int *p_status, int *p_current, int *p_current_lim, int *p_min_pres, int *p_max_pres, int *p_press, int *p_debit)
	{
	*p_state = pump_state;
	*p_status = pump_status;
	*p_current = pump_current;
	*p_current_lim = pump_current_limit;
	*p_min_pres = pump_min_lim;
	*p_max_pres = pump_max_lim;
	*p_press = pump_pressure_kpa;
	*p_debit = pump_debit;
	}
int get_pump_state_value()
	{
	return pump_state;
	}
int start_pump(int from)
	{
	esp_err_t ret = ESP_OK;
    if(pump_status == PUMP_ONLINE && from == 0)
    	{
    	ESP_LOGI(TAG, "Pump ONLINE. Start/Stop controlled by monitor task");
    	ret = ESP_FAIL;
    	}
    else
    	{
		gpio_set_level(PUMP_ONOFF_PIN, PIN_ON);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		int local_ps_mv;
		ret = get_pump_adc_values(&local_ps_mv);
		if(ret == ESP_OK)
			{
			psensor_mv = local_ps_mv;
			if(pump_current > PUMP_CURRENT_OFF && pump_current <= pump_current_limit)
				{
				pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
				pump_state = PUMP_ON;
				ESP_LOGI(TAG, "Pump ON   start %d", pump_pressure_kpa);
				}
			else if(pump_current > pump_current_limit)
				{
				ESP_LOGI(TAG, "Pump overcurrent %d / %d", pump_current, pump_current_limit);
				gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
				pump_state = PUMP_ON;
				pump_status = PUMP_FAULT;
				int local_ps = pump_status;
				rw_poperational(PARAM_WRITE, &local_ps);
				//rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps);
				ret = ESP_FAIL;
				}
			else
				{
				ESP_LOGI(TAG, "Pump doesn't start");
				gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
				pump_state = PUMP_OFF;
				ret = ESP_FAIL;
				}
			//get_pump_state();
			}
		else
			ESP_LOGI(TAG, "get_pump_adc_values() returned error");
		}
	return ret;
	}

int stop_pump(int from)
	{
	esp_err_t ret = ESP_OK;
	if(pump_status == PUMP_ONLINE && from == 0)
    	{
    	ESP_LOGI(TAG, "Pump ONLINE. Start/Stop controlled by monitor task");
    	ret = ESP_FAIL;
    	}
	else
    	{
		gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
		vTaskDelay(500 / portTICK_PERIOD_MS);
		int local_ps_mv;
		ret = get_pump_adc_values(&local_ps_mv);
		if(ret == ESP_OK)
			{
			psensor_mv = local_ps_mv;
			pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
			if(pump_pressure_kpa < 0)
				pump_pressure_kpa = 0;
			if(pump_current < PUMP_CURRENT_OFF)
				{
				pump_state = PUMP_OFF;
				ESP_LOGI(TAG, "Pump OFF  stop %d", pump_pressure_kpa);
				}
			else if(pump_current > pump_current_limit)
				{
				ESP_LOGI(TAG, "Pump overcurrent %d / %d", pump_current, pump_current_limit);
				gpio_set_level(PUMP_ONOFF_PIN, PUMP_OFF);
				pump_state = PUMP_ON;
				pump_status = PUMP_FAULT;
				int local_ps = pump_status;
				rw_poperational(PARAM_WRITE, &local_ps);
				//rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps);
				ret = ESP_FAIL;
				}
			else
				{
				ESP_LOGI(TAG, "Pump doesn't stop");
				pump_state = PUMP_ON;
				ret = ESP_FAIL;
				}
			}
		else
			ESP_LOGI(TAG, "get_pump_adc_values() returned error");
		//get_pump_state();
		}
	return ret;
	}
int pump_operational(int po)
	{
	esp_err_t ret = ESP_OK;
	loop = 0;
	if(pump_status == po)
		ESP_LOGI(TAG, "Pump already in desired mode");
	else
		{
		pump_status = po;
		int local_ps = pump_status;
		if(rw_poperational(PARAM_WRITE, &local_ps) != ESP_OK)
//		if(rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps) != ESP_OK)
			ESP_LOGI(TAG, "Operational status could not be updated: current value: %d", pump_status);
		}
	return ret;
	}
int set_pump_0_offset()
	{
	esp_err_t ret = ESP_FAIL;
    int old_value;
    int v_offset;

	//printf("\nSet 0 offset for pressure sensor");
	//printf("\nSensor should be already at 0 kPa pressure\n");
	int local_ps_mv = 0;
	ret = get_pump_adc_values(&local_ps_mv);
    if(ret == ESP_OK)
    	{
    	psensor_mv = local_ps_mv;
    	ESP_LOGI(TAG, " %d /", psensor_mv);
    	v_offset = 0;
    	ret = rw_poffset(PARAM_READ, &v_offset);
		//ret = rw_params(PARAM_READ, PARAM_V_OFFSET, &v_offset);
		if(ret == ESP_OK)
			{
			old_value = v_offset;
			v_offset = psensor_mv;
			ret = rw_poffset(PARAM_WRITE, &v_offset);
			//ret = rw_params(PARAM_WRITE, PARAM_V_OFFSET, &v_offset);
			if(ret == ESP_OK)
				{
				v_offset = 0;
				ret = rw_poffset(PARAM_READ, &v_offset);
				//ret = rw_params(PARAM_READ, PARAM_V_OFFSET, &v_offset);
				if(ret != ESP_OK)
					{
					ESP_LOGI(TAG, "Error reading 0 offset value: %d / %s", ret, esp_err_to_name(ret));
					}
				else
					ESP_LOGI(TAG, "0 offset value updated; new value = %d / old value = %d", v_offset, old_value);
				}
			else
				ESP_LOGI(TAG, "Error writing 0 offset value: %d / %s", ret, esp_err_to_name(ret));
			}
		else
			{
			ESP_LOGI(TAG, "Error getting current offset value");
			}
    	}
	return ret;
	}

int do_pumpop(int argc, char **argv)
	{
	uint32_t minp, maxp, fc, overpt, vrc;
	int nerrors = arg_parse(argc, argv, (void **)&pumpop_args);
    if (nerrors != 0)
    	{
        arg_print_errors(stderr, pumpop_args.end, argv[0]);
        return 1;
    	}
    if(strcmp(argv[0], "pump"))
    	return 0;
    /*
    if(strcmp(pumpop_args.op->sval[0], "start") == 0)
    	{
		start_pump(0);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "stop") == 0)
    	{
		stop_pump(0);
    	}
    	*/
    if(strcmp(pumpop_args.op->sval[0], "state") == 0)
    	{
    	get_pump_state();
    	}
    else if(strcmp(pumpop_args.op->sval[0], "set0") == 0)
    	{
    	set_pump_0_offset();
    	}
    else if(strcmp(pumpop_args.op->sval[0], "set_limits") == 0)
    	{
    	minp = pump_min_lim;
    	maxp = pump_max_lim;
    	fc = pump_current_limit;
    	overpt = overp_time_limit;
		vrc = void_run_count;
    	if(pumpop_args.minP->count)
    		{
    		minp = pumpop_args.minP->ival[0];
    		if(pumpop_args.maxP->count)
    			{
    			maxp = pumpop_args.maxP->ival[0];
    			if(pumpop_args.faultC->count)
    				{
    				fc = pumpop_args.faultC->ival[0];
   					if(pumpop_args.overpt->count)
						{
   						overpt = pumpop_args.overpt->ival[0];
						if(pumpop_args.vrc->count)
   							vrc = pumpop_args.vrc->ival[0];
						}
    				}
    			}
    		}
    	if(minp > 0 && maxp > 0 && maxp > minp)
    		{
    		pump_limits_t plimits;
    		plimits.max_val = maxp;
    		plimits.min_val = minp;
    		plimits.faultc = fc;
    		plimits.overp_lim = overpt;
			plimits.void_run_count = vrc;
			rw_plimits(PARAM_WRITE, &plimits);
    		//rw_params(PARAM_WRITE, PARAM_LIMITS, &plimits);
    		get_pump_state();
    		}
    	else
    		printf("pumpop: [%s]: invalid parameters\n", pumpop_args.op->sval[0]);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "offline") == 0)
    	{
    	return pump_operational(PUMP_OFFLINE);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "online") == 0)
    	{
    	pump_operational(PUMP_ONLINE);
    	}
    else if(strcmp(pumpop_args.op->sval[0], "test") == 0)
    	{
    	if(pumpop_args.minP->count)
    		testModeCurrent = pumpop_args.minP->ival[0];
    	if(pumpop_args.maxP->count)
    		testModePress = pumpop_args.maxP->ival[0];
    	}
    else
		{
		printf("pumpop: [%s] unknown option\n", pumpop_args.op->sval[0]);
		}
	return 0;
	}

void register_pumpop()
	{
	pump_cmd_queue = xQueueCreate(10, sizeof(msg_t));
	config_pump_gpio();
	//config_cmd_timer();

	testModeCurrent = 0;

	pump_task_handle = NULL;
	pumpop_mutex = xSemaphoreCreateMutex();
	if(!pumpop_mutex)
		{
		ESP_LOGE(TAG, "cannot create pumpop_mutex");
		esp_restart();
		}

	pumpop_args.op = arg_str1(NULL, NULL, "<op>", "type of operation: offline | online | set0 --> set 0 kPa sensor offset | set_limits --> set the interval pump is running ");
	pumpop_args.minP = arg_int0(NULL, NULL, "<min pressure (kPa)>", "at this pressure pump will start");
	pumpop_args.maxP = arg_int0(NULL, NULL, "<max pressure (kPa)>", "at this pressure pump will stop");
	pumpop_args.faultC = arg_int0(NULL, NULL, "<max current (mA))>", "at this current pump will stop");
	pumpop_args.overpt = arg_int0(NULL, NULL, "<#>", "timeout on max pressure");
	pumpop_args.vrc = arg_int0(NULL, NULL, "<#>", "max # of short cycles");
	pumpop_args.end = arg_end(1);
    const esp_console_cmd_t pumpop_cmd =
    	{
        .command = "pump",
        .help = "pump online|offline|set0|set_limits <min_pressure kPa> <max_pressure (kPa)> <fault_current (mA) <max stdev> <over pressure time limit> <void run count>",
        .hint = NULL,
        .func = &do_pumpop,
        .argtable = &pumpop_args
    	};
    ESP_ERROR_CHECK(esp_console_cmd_register(&pumpop_cmd));
    saved_t_water = 0;
    int rr = esp_reset_reason();
    if(rr == ESP_RST_POWERON || rr == ESP_RST_BROWNOUT)
    	total_qwater = 0;
    ESP_LOGI(TAG, "reset reason: %d", rr);
    uint64_t tqw;
    if(rw_twater(PARAM_READ, &tqw) == ESP_OK)
    	{
		if(total_qwater < tqw)
			total_qwater = tqw;
		}
    //read_t_water();
    get_pump_state();
    qmeter_ts = last_qmeter_ts = 0;
    config_qmeter_timer();
    xTaskCreate(pump_mon_task, "pump_task", 8192, NULL, 5, &pump_task_handle);
	if(!pump_task_handle)
		{
		ESP_LOGE(TAG, "Unable to start pump monitor task");
		esp_restart();
		}
	}

void pump_mon_task(void *pvParameters)
	{
	//minmax_t min[10], max[10];
	int saved_pump_state = -1, saved_pump_status = -1, saved_pump_current = -1, saved_pump_pressure_kpa = -1, saved_pump_debit = -1;
	uint64_t saved_total_qwater = -1;
	char msg[80];
	msg_t msg_ui;
	uint32_t pcount = 20, void_run = 0;
	time_t running_time = 0, start_time = 0;
	while(1)
		{
		//memset(min, 0, sizeof(min));
		//memset(max, 0, sizeof(max));
		int local_ps_mv;
		if(get_pump_adc_values(&local_ps_mv) == ESP_OK)
			{
			psensor_mv = local_ps_mv;
			pump_pressure_kpa = ((psensor_mv - kpa0_offset) * 250) / 1000;
			if(pump_pressure_kpa < 0) pump_pressure_kpa = 0;
			if(pump_current > pump_current_limit) // error case --> stop the pump
				{
				pump_status = PUMP_FAULT;
				if(stop_pump(1) == ESP_OK)
					pump_status = PUMP_OFFLINE;
				int local_ps = pump_status;
				rw_poperational(PARAM_WRITE, &local_ps);
				//rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps);
				}
			else
				{
				if(pump_current > PUMP_CURRENT_OFF)
					{
					pump_state = PUMP_ON;
					if(pump_status == PUMP_FAULT || pump_status == PUMP_OFFLINE)
						{
						if(stop_pump(1) == ESP_OK)
							{
							pump_status = PUMP_OFFLINE;
							int local_ps = pump_status;
							rw_poperational(PARAM_WRITE, &local_ps);
							//rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps);
							}
						else
							{
							msg_ui.source = PUMP_OP_ERROR;
							xQueueSend(ui_cmd_q, &msg_ui, 0);
							}
						}
					}
				else
					{
					pump_state = PUMP_OFF;
					if(pump_status == PUMP_FAULT)
						{
						pump_status = PUMP_OFFLINE;
						int local_ps = pump_status;
						rw_poperational(PARAM_WRITE, &local_ps);
						//rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps);
						}
					}
				if(pump_status == PUMP_ONLINE)
					{
					if(pump_pressure_kpa >= pump_max_lim)
						{
						if(start_overp_time == 0)
							start_overp_time = time(NULL);
						else
							{
							if(time(NULL) - start_overp_time >= overp_time_limit)
								{
								if(pump_state == PUMP_ON)
									{
									ESP_LOGI(TAG, "Pump OFF  mon %d", pump_pressure_kpa);
									if(stop_pump(1) == ESP_OK)
										{
										if(running_time <= (overp_time_limit * 12)/10) // here is a 20% tolerance
											void_run++;
										else
											void_run = 0;
										if(pump_debit > 0)
											void_run = 0;
										if(void_run > void_run_count)
											{
											ESP_LOGI(TAG, "void run overflow %lu, pump set to offline mode", (unsigned long)void_run);
											void_run = 0;
											pump_status = PUMP_OFFLINE;
											int local_ps = pump_status;
											int ret = rw_poperational(PARAM_WRITE, &local_ps);
											//int ret = rw_params(PARAM_WRITE, PARAM_OPERATIONAL, &local_ps);
											if(ret != ESP_OK)
												ESP_LOGI(TAG, "Operational status could not be updated: current value: %d", pump_status);
											}
										}
									else
										{
										msg_ui.source = PUMP_OP_ERROR;
										xQueueSend(ui_cmd_q, &msg_ui, 0);
										}
									}
								}
							}
						}
					else if(pump_pressure_kpa < pump_max_lim)
						{
						if(pump_debit >= 2)
							{
							if(pump_state == PUMP_OFF)
								{
								ESP_LOGI(TAG, "Pump ON  mon %d", pump_pressure_kpa);
								if(start_pump(1) == ESP_OK)
									{
									start_overp_time = 0;
									start_time = time(NULL);
									}
								else
									{
									stop_pump(1);
									pump_operational(PUMP_OFFLINE);
									msg_ui.source = PUMP_OP_ERROR;
									xQueueSend(ui_cmd_q, &msg_ui, 0);
									}
								}
							}
						else
							{
							if(pump_pressure_kpa < pump_min_lim)
								{
								if(start_pump(1) == ESP_OK)
									{
									start_overp_time = 0;
									start_time = time(NULL);
									}
								else
									{
									stop_pump(1);
									pump_operational(PUMP_OFFLINE);
									msg_ui.source = PUMP_OP_ERROR;
									xQueueSend(ui_cmd_q, &msg_ui, 0);
									}
								}
							}
						}
					}
				if(pump_state == PUMP_ON)
					{
					running_time = time(NULL) - start_time;
					}
				}
			}
		if((loop % pcount) == 0)
			{

			if(pump_state != saved_pump_state ||
				pump_status != saved_pump_status ||
					pump_current != saved_pump_current ||
						pump_pressure_kpa != saved_pump_pressure_kpa ||
							saved_pump_debit != pump_debit ||
								saved_total_qwater != total_qwater)

				{
				sprintf(msg, "%s\1%d\1%d\1%d\1%d\1%d\1%llu", PUMP_STATE, pump_state, pump_status, pump_current, pump_pressure_kpa, pump_debit, total_qwater/1000);
				publish_topic(TOPIC_MONITOR, msg, 0, 0);
				//ESP_LOGI(TAG, "Pump state running:%d, pressure:%d(kPa), current:%d(mA), debit:%d, f(debit):%d, q_water:%llu, loop:%lu", pump_state, pump_pressure_kpa, pump_current, pump_debit, qmeter_pc_sec, total_qwater/1000, loop);
				saved_pump_state = pump_state;
				saved_pump_status = pump_status;
				saved_pump_current = pump_current;
				saved_pump_pressure_kpa = pump_pressure_kpa;
				saved_pump_debit = pump_debit;
				saved_total_qwater = total_qwater;
				msg_ui.source = PUMP_VAL_CHANGE;
				xQueueSend(ui_cmd_q, &msg_ui, 0);
				}
			}
		if(qmeter_ts - last_qmeter_ts >  0)
			{
			pump_debit = (qmeter_pc_sec * qcal_a + qcal_b) / 1000;
			if(qmeter_pc_sec < 2)
				pump_debit = 0;
			total_qwater = total_qwater + (qmeter_ts - last_qmeter_ts) * pump_debit * 1000 / 60; //cmc
			last_qmeter_ts = qmeter_ts;
			}
		// run loop every 2 sec if PUMP_OFFLINE
		// else loop every 100 msec
		if(pump_status != PUMP_ONLINE)
			{
			pcount = 1;
			vTaskDelay(2000 / portTICK_PERIOD_MS);
			//total_qwater++;
			}
		else
			{
			pcount = 10;
			vTaskDelay(100 / portTICK_PERIOD_MS);
			}
		struct tm tminfo;
		time_t ltime;
		ltime = time(NULL);
		localtime_r(&ltime, &tminfo);
		if(tminfo.tm_hour * 60 + tminfo.tm_min == SAVE_TWATER_TIME)
			{
			if(!saved_t_water)
				{
				if(rw_twater(PARAM_WRITE, &total_qwater) == ESP_OK)
				//if(save_t_water(total_qwater) == ESP_OK)
					saved_t_water = 1;
				}
			}
		else
			saved_t_water = 0;
		loop++;
		}
	}
static int read_qcal()
	{
	int ret = ESP_OK;
	struct stat st;
	char bufr[64];
	qcal_a = 0;
	qcal_b = 1;
	if (stat(BASE_PATH"/"QCAL_FILE, &st) != 0)
		{
		// file does no exists
		ESP_LOGI(TAG, "Calibration file for q-meter does not exist");
		ret = ESP_FAIL;
		}
	else
		{
		FILE *f = fopen(BASE_PATH"/"QCAL_FILE, "r");
		if (f != NULL)
			{
			fgets(bufr, 60, f);
			sscanf(bufr, "%d %d", &qcal_a, &qcal_b);
			fclose(f);
			ESP_LOGI(TAG, "qcal_a = %d, qcal_b = %d", qcal_a, qcal_b);
			}
		}

	return ret;
	}


#if 0
void process_adc_current(minmax_t *min, minmax_t *max)
	{
	int smin, smax, smed;
    int i, k;
    int dmin, dmax, med, delta;
	smin = smax = smed = 0;
	k = 0;
	if(testModeCurrent > 0)
		{
		pump_current = testModeCurrent;
		return;
		}
	for(i = 0; i < 10; i++)
		{
		if(min[i].index == 0 || max[i].index == 0)
			break;
		med = (max[i].mv + min[i].mv)/2;
		dmin = med - min[i].mv;
		dmax = max[i].mv - med;
		delta = max[i].mv - min[i].mv;
		/*
		 * check the period
		 * 50 Hz @2 kHz sampling rate it requires 20 samples between min and max
		 * accepted valid is 18 - 22
		 */
		if(abs((int)min[i].index - (int)max[i].index) >= 18 && abs((int)min[i].index - (int)max[i].index) <= 22)
			{
			if(delta >= 25 && dmin > 10 && dmax > 10) // valid measurement
				{
				smin += min[i].mv;
				smax += max[i].mv;
				smed += med;
				k++;
				}
			}
		}
	if(k)
		{
		smin /= k;
		smax /= k;
		smed /= k;
		if(smed < 2000 || smed > 3000)
			pump_current = 0;
		else
			{
			int cmin = (smed - smin) * 10000;
			int cmax = (smax - smed) * 10000;
			pump_current = (cmin + cmax) / 2 / 1414;
			//ESP_LOGI(TAG, "current measurements: %d   %d   %d", cmin, cmax, pump_current);
			}
		}
	else
		{
		pump_current = 0;
		}
	}
#endif
#endif
