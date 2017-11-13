// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017, Centro "E.Piaggio"
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         October 01, 2017
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED

//=================================================================     includes
#include <device.h>
#include "stdlib.h"
#include "math.h"
#include "commands.h"

//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION         "W-FYD MOD v6.1.0"

#define NUM_OF_MOTORS           2
#define NUM_OF_SENSORS          2
#define NUM_OF_ANALOG_INPUTS    5
#define NUM_OF_PARAMS           14

//==============================================================================
//                                                                       CONTROL
//==============================================================================

#define PWM_MAX_VALUE   100         // PWM is from 0 to 100, this value is used
                                    // to limit this value
#define PWM_DEAD        0           // deadband value, is directly added to the
                                    // value of PWM always limited to 100

// WARNING POS_INTEGRAL_SAT_LIMIT need to be lower than 2^17 or you have to modify
// the code in the motor control function
#define POS_INTEGRAL_SAT_LIMIT   100000  // Anti wind-up
#define CURR_INTEGRAL_SAT_LIMIT  100000  // Anti wind-up
#define CALIB_CURRENT            1000    // Max current for calibration (mA)
#define DEFAULT_CURRENT_LIMIT    1500    // Current limit when using CURR_AND_POS_CONTROL
    

//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

// Main frequency 1000 Hz

#define CALIBRATION_DIV         100     // 10 Hz

#define DIV_INIT_VALUE          1

//==============================================================================
//                                                                           DMA
//==============================================================================
    
#define DMA_BYTES_PER_BURST 2
#define DMA_REQUEST_PER_BURST 1
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)
    
//==============================================================================
//                                                                     INTERRUPT
//==============================================================================
#define    WAIT_START   0
#define    WAIT_ID      1
#define    WAIT_LENGTH  2
#define    RECEIVE      3
#define    UNLOAD       4
    
//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE           0
#define TRUE            1

#define DEFAULT_EEPROM_DISPLACEMENT 8   // in pages
    
#define MAX_WATCHDOG_TIMER 250          // num * 2 [cs]
    
#define DEFAULT_MAX_FORCE_PULSE 1900
#define DEFAULT_MIN_FORCE_PULSE 1350
#define MAX_FORCE 3520
#define MIN_FORCE -1583


//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references

struct st_ref {

    int32 pos[NUM_OF_MOTORS];       // motor reference position
    uint8 onoff;                    // enable flags

};

//=============================================================     measurements

struct st_meas {

    int32 pos[NUM_OF_SENSORS];      // sensor position
    int16 curr[NUM_OF_MOTORS];      // motor currents
    int8 rot[NUM_OF_SENSORS];       // sensor rotations
    int16 vel[NUM_OF_SENSORS];      // sensor velocity
    int16 acc[NUM_OF_SENSORS];      // sensor acceleration
    int16 ir;                       // sensor IR
    int16 servo;                    // servo indentation
    int16 force;                    // sensor force
    float duty_cycle_f;              // duty cycle relative to the force

};

//==============================================================     data packet

struct st_data {

    uint8   buffer[128];                    // CMD/DATA/CHECKSUM
    int16   length;                         // length
    int16   ind;                            // index
    uint8   ready;                          // Flag

};

//============================================     settings stored on the memory

struct st_mem {

    uint8   flag;                       // Device has been configured               1
    uint8   id;                         // device ID                                1

    int32   k_p;                        // Proportional constant                    4
    int32   k_i;                        // Derivative constant                      4
    int32   k_d;                        // Integrative constant                     4

    int16   current_limit;              // Limit for absorbed current               2

    uint8   activ;                      // Activation upon startup                  1

    uint8   res[NUM_OF_SENSORS];        // Angle resolution                         1 (3)
    int32   m_off[NUM_OF_SENSORS];      // Measurement offset                       4 (12)
    float   m_mult[NUM_OF_SENSORS];     // Measurement multiplier                   4 (12)  32
    uint8   pos_lim_flag;               // Position limit active/inactive           1
    int32   pos_lim_inf[NUM_OF_MOTORS]; // Inferior position limit for motors       4 (8)
    int32   pos_lim_sup[NUM_OF_MOTORS]; // Superior position limit for motors       4 (8)

    uint16  max_stiffness;              // Max stiffness value obtained
                                        // during calibration                       2       19
    uint8   baud_rate;                  // Baud Rate Setted                         1
    uint8   watchdog_period;            // Watchdog period setted, 255 = disable    1
    int32   max_step_neg;               // Maximum velocity for negative inputs     4       
    int32   max_step_pos;               // Maximum velocity for positive inputs     4

    /*=========================W-FYD RELATIVE PARAMETERS=============================*/
    int32   thr_max_force;              // Force upper bound pulsation mode         2640    (0-3520)
    int32   thr_max_pressure;           // Sistolic pressure
    
    // If these values are modified, the proportion has to be modified too
    int32   thr_min_pressure;           // Diastolic pressure
    int32   thr_min_force;              // Force lower bound pulsation mode         880     (0-3520)
    
    uint8   flag_pulse;                 // implementation of contact pulsation      1
    
    double  step_const;                 // ramp sample step
    int32   pulse_freq;                 // pulse frequency
    
    
    uint16 power_tension;               //                                          2
                                                                                    //TO UPDATE
};



//=================================================     calibration status

enum calibration_status {

    STOP        = 0,
    START       = 1,
    CONTINUE_1  = 2,
    CONTINUE_2  = 3,
    PAUSE_1     = 4,
    PAUSE_2     = 5

};

//====================================      external global variables declaration


extern struct st_ref    g_ref, g_refNew, g_refOld;  // motor variables
extern struct st_meas   g_meas, g_measOld;          // measurements
extern struct st_data   g_rx;                       // income data
extern struct st_mem    g_mem, c_mem;               // memory


extern uint32 timer_value;
extern uint32 timer_value0;

// Device Data

extern int32   dev_tension;                         // Power supply tension
extern uint8   dev_pwm_limit;

extern uint8 calibration_flag;

// Bit Flag

extern CYBIT reset_last_value_flag;
extern CYBIT tension_valid;                         // tension validation bit
extern CYBIT interrupt_flag;                        // interrupt flag enabler
extern CYBIT watchdog_flag;                         // watchdog flag enabler

// DMA Buffer

extern int16 ADC_buf[5];

// W-FYD Functions

extern uint16 pulse_counter;                        // pulsation counter
extern float pulse_counter_float;                   // pulsation float counter
extern CYBIT motor_pulse_started;                   // pulsation started flag

// -----------------------------------------------------------------------------

#endif

//[] END OF FILE