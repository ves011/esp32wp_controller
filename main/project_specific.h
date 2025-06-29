/*
 * project_specific.h
 *
 *  Created on: Mar 20, 2023
 *      Author: viorel_serbu
 */

/**
 * @file project_specific.h
 * @brief defines the controller type: ACTIVE_CONTROLLER and ID: CTRL_DEV_ID
 */
#ifndef MAIN_PROJECT_SPECIFIC_H_
#define MAIN_PROJECT_SPECIFIC_H_

#define TEST_BUILD 0
#if(TEST_BUILD == 1)
	#define WITH_CONSOLE
	#define TEST1
	#define CTRL_DEV_ID					100
	#define LOG_SERVER_DEV				"proxy.gnet"
	#define LOG_PORT_DEV				8081
#else
	#define CTRL_DEV_ID					1
#endif

#define COMM_PROTO	MQTT_PROTO

#define DEV_NAME						"Pompa & Irigatie"
#define PROMPT_STR "WP_ctrl"

#define ACTIVE_CONTROLLER				(WP_CONTROLLER)
#define WIFI_STA_ON						1
#define MQTT_PUBLISH					(1)
//#define OTA_SUPPORT

//#define WP_HW_V1
#define WP_HW_V2

#define ADC_AD7811
//#define ADC_ESP32
#define ROT_ENCODER

#define WIFI_AP_ON	0
#define WIFI_STA_ON	1

/*
Message definitions for device monitor queue
*/
#define MSG_WIFI			1	// wifi connect (.val = 1)/disconnect (.val = 0) event 
#define MSG_TIMER_MON		1000


#endif /* MAIN_PROJECT_SPECIFIC_H_ */
