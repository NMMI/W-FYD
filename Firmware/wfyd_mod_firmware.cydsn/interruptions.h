// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         Dic. 1, 2015
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//=====================================================        Interrupt Handler
    
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);
CY_ISR_PROTO(ISR_WATCHDOG_Handler);

//=====================================================     function declaration

void function_scheduler(void);

void encoder_reading(const uint8, const uint8);
void motor_control(const uint8);
void analog_read_end();

void calibration(void);

void pwm_limit_search();

void interrupt_manager();

void set_servo(int16);

void get_servo();

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */