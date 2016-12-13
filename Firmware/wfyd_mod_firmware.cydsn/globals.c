// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         globals.c
*
* \brief        Global variables.
* \date         Dic. 1, 2015
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes

#include <globals.h>

//=============================================      global variables definition

struct st_ref   g_ref, g_refNew, g_refOld;  // motor variables
struct st_meas  g_meas, g_measOld;          // measurements
struct st_data  g_rx;                       // income data
struct st_mem   g_mem, c_mem;               // memory

// Timer value for debug field

uint32 timer_value;
uint32 timer_value0;

// Device Data

int32   dev_tension;                // Power supply tension
uint8   dev_pwm_limit;

uint8 calibration_flag;

// Bit Flag

CYBIT reset_last_value_flag;
CYBIT tension_valid;
CYBIT interrupt_flag;
CYBIT watchdog_flag;

// DMA Buffer

int16 ADC_buf[5]; 

// W-FYD Functions

uint16 pulse_counter;                       // pulsation counter
CYBIT motor_pulse_started = FALSE;          // pulsation started flag
float pulse_counter_float;                  // pulsation float counter
