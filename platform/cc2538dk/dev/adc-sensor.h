/*
 * Copyright (c) 2013, ADVANSEE - http://www.advansee.com/
 * Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup cc2538-smartrf-sensors
 * @{
 *
 * \defgroup cc2538dk-als-sensor cc2538dk ALS Driver
 *
 * Driver for the SmartRF06EB ADC sensor
 * @{
 *
 * \file
 * Header file for the cc2538dk ADC Driver
 */
#ifndef ADC_SENSOR_H_
#define ADC_SENSOR_H_

#include "lib/sensors.h"

/*---------------------------------------------------------------------------*/
/** \name ALS sensor
 * @{
 */

#define ADC_SENSOR "ADC"

#define ADC_SENSOR_VDD_3        0 /**< On-chip VDD / 3 */
#define ADC_SENSOR_SENS4	1 /**< Onfarm Board SENS4 */
#define ADC_SENSOR_SENS3	2 /**< Onfarm Board SENS3 */
#define ADC_SENSOR_SENS2	3 /**< Onfarm Board SENS2 */
#define ADC_SENSOR_SENS1	4 /**< Onfarm Board SENS1 */

/** @} */

extern const struct sensors_sensor adc_sensor;

#endif /* ADC_SENSOR_H_ */

/**
 * @}
 * @}
 */
