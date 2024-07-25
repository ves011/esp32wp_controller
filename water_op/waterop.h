/*
 * water_op.h
 *
 *  Created on: Jun 8, 2023
 *      Author: viorel_serbu
 */

#ifndef WATER_OP_WATEROP_H_
#define WATER_OP_WATEROP_H_

#define DV0						0
#define DV1						1
#define DV2						2
#define DV3						3
#define DVCOUNT					4
#define WPDAY					2

/*
	#define PINEN_DV0				(6)
	#define PINEN_DV1				(7)
*/
	/* A1 ON 	/ B1 OFF 	--> open
	 * A1 OFF 	/ B1 ON 	--> close
	 * A1 OFF 	/ B1 OFF 	--> inactive
	 */
/*
	#define PINMOT_A1				(8)
	#define PINMOT_B1				(3)
	//#define PINMOT_A2				(12)
	//#define PINMOT_B2				(11)
	#define PINSENSE_MOT			(4)
*/
	#define DV_CURRENT_OFF_LIM			20
	#define DV_CURRENT_OFF_COUNT		3
	#define DV_CURRENT FAULT			200


#define WATER_OFF				0
#define WATER_ON				1

#define STDEV_MAX				10
#define DVSTATE_OK				1
#define DVSTATE_FAULT			2

#define DVOPEN					1
#define DVCLOSE					0
#define DVOP_INPROGRESS			2
#define DVOP_STEADY				3
#define MINPRES					100

/** Name of the file storing dv program */
#define PROGRAM_FILE		"dv_program.txt"
#define STATUS_FILE			"program_status.txt"

/** watering program state */
#define NOT_STARTED				0
#define IN_PROGRESS				1
#define COMPLETED				2
#define ABORTED					3
#define START_ERROR				4
#define STOP_ERROR				5
#define INVALID					6  // if start time is after stop time
#define PROG_SKIP				7

#define RETRY_OP_WATERING		5
//#define NO_PUMP_RESPONSE		6
#define PUMP_WRONG_STATE		7
//#define DV_ERROR				8
//#define PUMP_PRESSURE_LOW		9
#define START_WATERING_ERROR	10
#define STOP_WATERING_ERROR		11

#define RESET_PROGRAM_H			0
#define RESET_PROGRAM_M			22

#define FAULT_PUMP				10
#define DV_FAULT				11
//#define PUMP_NO_RESPONSE		12
#define DV_OPEN_FAIL			13
#define DV_CLOSE_FAIL			14
//#define GET_STATE_NOT_PERMITED	15
//#define GET_STATE_ERROR			16
//#define OP_ABORTED				17
#define PRESS2HIGH				18
#define PRESS2LOW				19
#define PRESS2LOWDVCLOSED		20


#define WATER_PUMP_DESC			"pump01"
#define PUMP_CMD_TOPIC			"pump01/cmd"

#define WATERING_STATE			"wstate"
#define WATERING_START			"wstart"
#define WATERING_STOP			"wstop"
#define PUMP_STATE				"pstate"
#define PROG_HISTORY			"phist"
#define ERR_START				"err_start"
#define ERR_STOP				"err_stop"

#define STATE_W					"state"
#define STATE_P					"program"
#define DVSTATE					"dvstate"
#define DVOP					"dvop"



typedef struct
	{
	uint8_t dvno;
	uint8_t pin_enable;
	uint8_t state;
	} dvconfig_t;

typedef struct
	{
	int no;
	int starth;
	int startm;
	int stoph;
	int stopm;
	struct tm eff_start;
	struct tm eff_stop;
	int cs;
	int fault;
	int qwater;
	struct tm last_start;
	struct tm last_stop;
	int last_cs;
	int last_fault;
	} w_prog_t;

typedef struct
		{
		struct
			{
			int dv;
			w_prog_t w_prog[WPDAY];
			} p[DVCOUNT];
		} dvprogram_t;
		/*
typedef struct
	{
	int year;
	int mon;
	int day;
	int hour;
	int min;
	int sec;
	int w_count;
	int cs;
	int fault;
	int qwater;
	}last_no_status_t;
typedef struct
	{
	int dv;
	last_no_status_t last_no[2];
	}last_status_t;

typedef struct
	{
	int dv;
	last_no_status_t last_no_b[2];
	last_no_status_t last_no_e[2];
	}last_status2_t;
*/
int do_dvop(int argc, char **argv);
void register_waterop(void);
void get_dv_current(int *dv_current, int *stdev);
void parse_devstr(int argc, char **argv);
//void get_water_values(last_status2_t *lst, dvprogram_t *dvprog, int *dvstate);
void get_water_values(dvprogram_t *dvprog, int *dvstate);
void get_water_dv_state(int *dvstate);
int open_dv(int dvnum);
int close_dv(int dvnum);
//void get_act_state(int dvnum);

#endif /* WATER_OP_WATEROP_H_ */
