// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------
// File:        data_processing.c
//
// Description: Data processing functions.
// ----------------------------------------------------------------------------
//
// Project:             qbotFirmware
// Project Manager(s):  Fabio Bonomo and Felipe Belo
//
// Soft. Ver:           0.1b4
// Date:                Dic. 1, 2015
//
// This version changes:
//      - not reported
//
// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// ----------------------------------------------------------------------------
// Permission is granted to copy, modify and redistribute this file, provided
// header message is retained.
// ----------------------------------------------------------------------------
#ifndef COMMAND_PROCESSING_H_INCLUDED
#define COMMAND_PROCESSING_H_INCLUDED
// ----------------------------------------------------------------------------

//=================================================================     includes
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE

//==============================================================================
//                                                          function definitions
//==============================================================================

void	setZeros 			(void);
void	get_param_list		(uint16 index); 	
void    infoPrepare        	(unsigned char *);
void    infoGet            	(uint16);
void    commProcess        	();
void    commWrite          	(uint8*, const uint16, uint8);
void    commWrite_old_id    (uint8*, const uint16, uint8);
uint8   memStore           	(int);
void    sendAcknowledgment 	(const uint8);
void    memRecall          	(void);
uint8   memRestore         	(void);
uint8   memInit            	(void);

//==============================================================================
//                                            Service Routine interrupt function
//==============================================================================

void cmd_get_measurements();
void cmd_get_inputs();
void cmd_get_currents();
void cmd_get_curr_and_meas();
void cmd_set_inputs();
void cmd_get_velocities();
void cmd_activate();
void cmd_set_watchdog();
void cmd_get_activate();
void cmd_ping();
void cmd_store_params();
void cmd_set_baudrate();
void cmd_get_ir();
void cmd_get_servo();
void cmd_get_force();
void cmd_get_duty_cy_max();

#endif

/* [] END OF FILE */