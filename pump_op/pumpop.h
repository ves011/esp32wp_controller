/*
 * pumpop.h
 *
 *  Created on: Jan 28, 2023
 *      Author: viorel_serbu
 */

/**
 * @file pumpop.h
 * @brief header file definitions for control logic of submersible water pump
 */
#ifndef PUMP_OP_PUMPOP_H_
#define PUMP_OP_PUMPOP_H_

/** Pressure sensor ADC channel = SENSOR_PIN -1*/
#define	SENSOR_ADC_CHANNEL		SENSOR_PIN - 1
 /** ACS712 current sensor ADC channel = CURRENT_PIN - 1*/
#define CURRENT_ADC_CHANNEL		CURRENT_PIN - 1

	#define PUMP_ON					(1)
	#define PUMP_OFF				(0)

	#define PUMP_ONLINE				(2)
	#define PUMP_OFFLINE			(3)
	#define PUMP_FAULT				(4)
	#define PUMP_OVERRCURRENT		(5)
	#define PUMP_DOESNT_START		(6)
	#define PUMP_DOESNT_STOP		(7)
	
	#define MSG_QMETER_SEC			(1)
	#define MSG_PUMP_MON			(2)
	#define MSG_SET_ONLINE			(3)
	#define MSG_SET_OFFLINE			(4)

	#define MON_INTERVAL			200000	//200 msec timer tick
#define QMETER_FREQ_RES				1000000	// 1msec resolution
#define QMETER_MEAS_TIME			1000000	// 1 sec

/** How long(msec) the button has to be pressed  to send the command*/
#define PUSH_TIME_US			(3000000)

/** under this value the pump is OFF */
#define PUMP_CURRENT_OFF		300

 /** Name of the file storing pressure sensor output (mV) for 0kPA */
#define OFFSET_FILE				"psensor_voffset.txt"
 /** Name of the file storing operating limits for pump - see pump_limits definition */
#define LIMITS_FILE				"pump_limits.txt"
 /** Name of the file storing pump status: online | offline */
#define OPERATIONAL_FILE		"pump_status.txt"
/** file storing qmeter calibration a, b : q = (f + a) / b */
#define QCAL_FILE				"qcal.txt"

/** file storing total water consumed */
#define TWATER_FILE				"twater.txt"

#define DEFAULT_PRES_MIN_LIMIT		100
#define DEFAULT_PRES_MAX_LIMIT		340
#define DEFAULT_PUMP_CURRENT_LIMIT	5000


#define SAVE_TWATER_TIME			32 // = hour *60 + min --> 00:32

/*
typedef struct
	{
	int v_offset;
	} psensor_offset_t;
*/

extern QueueHandle_t pump_cmd_queue;
typedef struct
    {
    uint32_t mv;
    uint32_t index;
    } minmax_t;

/**
 * @brief pump limits structure passed to rw_params() function.
 */
typedef struct
	{
	uint32_t min_val;			/*!< min pressure limit										*/
	uint32_t faultc;			/*!< max acceptable current when pump running (mA)			*/
	} pump_limits_t;


/**
 * @brief dispatch pump commands based on arguments
 * @return 0 on success, 1 on error
 */
int do_pumpop(int argc, char **argv);

/**
 * @brief gets the pump state and publish response on state topic
 * @return ESP_OK or ESP_FAIL
 */
int get_pump_state(void);

/**
 * @brief start pump
 * @param from
 * 		if from == 1 function is called inside pump monitor loop
 * @return 	ESP_OK on success
 * 			ESP_FAIL on error
 */
int start_pump(int from);

/**
 * @brief stop pump
 * @param from
 * 		if from == 1 function is called inside pump monitor loop
 * @return 	ESP_OK on success
 * 			ESP_FAIL on error
 */
int stop_pump(int from);
void register_pumpop();
int set_pump_0_offset(void);
int pump_operational(int po);
void pump_mon_task(void *pvParameters);
void process_adc_current(minmax_t *min, minmax_t *max);
void get_pump_values(int *p_state, int *p_status, int *p_current, int *p_current_lim, int *p_min_pres, int *p_press, float *p_debit);
int get_pump_state_value();

#endif /* PUMP_OP_PUMPOP_H_ */
