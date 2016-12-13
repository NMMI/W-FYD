
// Copyright (c) 2016, Mattia Poggiani & Simone Fani.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// - Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \file        w_fyd_communications.h
 *
 * \brief       Library of functions for SERIAL PORT communication with a IMU board.
 *              Function Prototypes.
 *
 * \details
 *
 *  This library contains all necessary functions for communicating with a IMU board when
 *  using a USB to RS485 connector that provides a Virtual COM interface.
**/

 /**
* \mainpage     qbAPI Libraries
*
* \brief        Those functions allows to use the W-FYD through a serial port
*
* \version      6.0.0
*
* \author       Mattia Poggiani, Simone Fani
*
* \date         October 14, 2016
*
* \details      This is a set of functions that allows to use the W-FYD  
*               via a serial port.
*/

#ifndef W_FYD_SERIALPORT_H_INCLUDED
#define W_FYD_SERIALPORT_H_INCLUDED

#include "qbmove_communications.h"


int commGetIR(comm_settings *comm_settings_t, int id, short int measurements[1]);

void commSetServo(comm_settings *comm_settings_t, int id, short int inputs[1]);

int commGetServo(comm_settings *comm_settings_t, int id, short int measurements[1]);

int commGetForce(comm_settings *comm_settings_t, int id, short int measurements[1]);

int commGetDCyM(comm_settings *comm_settings_t, int id, short int measurements[1]);
			   


// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */
