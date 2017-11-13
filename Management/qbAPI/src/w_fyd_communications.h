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
 * \file        w_fyd_communications.h
 *
 * \brief       Library of functions for SERIAL PORT communication with WFYD.
 *              Function Prototypes.
 * \date         October 01, 2017
 * \author       _Centro "E.Piaggio"_
 * \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
 * \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
 * \details
 *
 *  This library contains all necessary functions for communicating with WFYD when
 *  using a USB to RS485 connector that provides a Virtual COM interface.
**/

 /**
* \mainpage     qbAPI Libraries
*
* \brief        Those functions allows to use WFYD haptic feedback device through a serial port
*
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*
* \date         October 01, 2017
*
* \details      This is a set of functions that allows to use the boards 
*               via a serial port.
*
*               Those APIs can be compiled for Unix systems like Linux and
*               Mac OS X and even for Windows. Refer to https://github.com/NMMI/qbAPI/tree/centropiaggio
*               for detailed instructions.
*
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
