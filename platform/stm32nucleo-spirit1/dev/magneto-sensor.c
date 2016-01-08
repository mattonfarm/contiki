<<<<<<< HEAD
/**
******************************************************************************
* @file    magneto-sensor.c
* @author  System LAB
* @version V1.0.0
* @date    17-June-2015
* @brief   Enable magneto sensor functionality 
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
=======
/*
 * Copyright (c) 2012, STMicroelectronics.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 */
/*---------------------------------------------------------------------------*/
>>>>>>> refs/remotes/contiki-os/master
/**
 * \addtogroup stm32nucleo-spirit1-magneto-sensor
 * @{
 *
 * \file
 * Driver for the stm32nucleo-spirit1 Magneto sensor (on expansion board)
 */
/*---------------------------------------------------------------------------*/
#ifdef X_NUCLEO_IKS01A1
/*---------------------------------------------------------------------------*/
#include "lib/sensors.h"
#include "magneto-sensor.h"
#include "st-lib.h"
/*---------------------------------------------------------------------------*/
static int _active = 1;
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
static void init(void)
=======
static void
init(void)
>>>>>>> refs/remotes/contiki-os/master
{
  BSP_MAGNETO_Init();
  _active = 1;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
static void activate(void)
=======
static void
activate(void)
>>>>>>> refs/remotes/contiki-os/master
{
  _active = 1;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
static void deactivate(void)
=======
static void
deactivate(void)
>>>>>>> refs/remotes/contiki-os/master
{
  _active = 0;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
static int active(void)
=======
static int
active(void)
>>>>>>> refs/remotes/contiki-os/master
{
  return _active;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
static int value(int type)
=======
static int
value(int type)
>>>>>>> refs/remotes/contiki-os/master
{
  int32_t ret_val = 0;
  volatile st_lib_axes_raw_typedef axes_raw_data;

<<<<<<< HEAD
  st_lib_bsp_magneto_m_get_axes_raw(&axes_raw_data);

  switch (type) {
    case X_AXIS:
        ret_val = axes_raw_data.AXIS_X ;
	break;
    case Y_AXIS:
        ret_val = axes_raw_data.AXIS_Y ;
	break;
    case Z_AXIS:
        ret_val = axes_raw_data.AXIS_Z ;
	break;
    default:
	break;
=======
  /* NOTE: this is a demo of mapping ST Nucleo sensors on Contiki sensor API.
   * For a real use case of sensors like acceleration, magneto and gyroscope,
   * it is better to directly call the ST lib to get the three value (X/Y/Z)
   * at once.
   */
  st_lib_bsp_magneto_m_get_axes_raw(&axes_raw_data);

  switch(type) {
  case X_AXIS:
    ret_val = axes_raw_data.AXIS_X;
    break;
  case Y_AXIS:
    ret_val = axes_raw_data.AXIS_Y;
    break;
  case Z_AXIS:
    ret_val = axes_raw_data.AXIS_Z;
    break;
  default:
    break;
>>>>>>> refs/remotes/contiki-os/master
  }

  return ret_val;
}
/*---------------------------------------------------------------------------*/
<<<<<<< HEAD
static int configure(int type, int value)
{
  switch(type) {
    case SENSORS_HW_INIT:
      init();
      return 1;
    case SENSORS_ACTIVE:
      if(value) {      
        activate();
      } else {
        deactivate();
      }
      return 1;
  }
 
  return 0;
}
/*---------------------------------------------------------------------------*/
static int status(int type)
{
  switch(type) {
    case SENSORS_READY:
      return active();
  }
  
=======
static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    init();
    return 1;
  case SENSORS_ACTIVE:
    if(value) {
      activate();
    } else {
      deactivate();
    }
    return 1;
  }

  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_READY:
    return active();
  }

>>>>>>> refs/remotes/contiki-os/master
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(magneto_sensor, MAGNETO_SENSOR, value, configure, status);
/*---------------------------------------------------------------------------*/
#endif /*X_NUCLEO_IKS01A1*/
/*---------------------------------------------------------------------------*/
/** @} */
