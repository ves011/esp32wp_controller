/*
 * pump_files.h
 *
 *  Created on: Nov 14, 2024
 *      Author: viorel_serbu
 */

#ifndef PUMP_OP_PUMP_FILES_H_
#define PUMP_OP_PUMP_FILES_H_

//int read_t_water();
//int save_t_water(uint64_t total_qw);

int rw_twater(int rw, float *total_qw);
int rw_poffset(int rw, int *v_offset);
int rw_plimits(int rw, pump_limits_t *plimits);
int rw_poperational(int rw, int *param_val);




#endif /* PUMP_OP_PUMP_FILES_H_ */
