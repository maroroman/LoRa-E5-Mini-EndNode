/*
 * bme280_support.h
 *
 *  Created on: Apr 21, 2024
 *      Author: Maro
 */

#ifndef INC_BME280_SUPPORT_H_
#define INC_BME280_SUPPORT_H_

s32 bme280_data_readout_template(int32_t *v_comp_temp_s32_pt, uint32_t *v_comp_press_u32_pt, uint32_t *v_comp_humidity_u32_pt);

#endif /* INC_BME280_SUPPORT_H_ */
