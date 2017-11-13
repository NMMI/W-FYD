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
 * \file        w_fyd_commands.h
 *
 *  \brief      Definitions for W-FYD commands, parameters and packages.
 *
 *  \details
 *  This file is included in the W-FYD firmware, in its libraries and
 *  applications. It contains all definitions that are necessary for the
 *  contruction of communication packages.
 *
 *  It includes definitions for all of the device commands, parameters and also
 *  the size of answer packages.
 *
**/

#ifndef ADDITIONAL_COM_COMMANDS_DEFINITIONS_H_INCLUDED
#define ADDITIONAL_COM_COMMANDS_DEFINITIONS_H_INCLUDED

#include "commands.h"

#ifdef API_VERSION
	#undef API_VERSION
	#define API_VERSION "v6.1.0 mod. W-FYD"
#endif

//==============================================================================
//                                                                      COMMANDS
//==============================================================================


/** \name W-FYD Commands
 * \{
**/

enum w_fyd_command
{
	CMD_GET_IR              = 147,  ///< Command for asking ir readings (W-FYD)
    CMD_SET_SERVO           = 148,  ///< Command for setting servo position (W-FYD)
    CMD_GET_SERVO           = 149,  ///< Command for asking servo position (W-FYD)
    CMD_GET_FORCE           = 150,  ///< Command for asking force measures (W-FYD)
    CMD_GET_DUTY_CY_MAX     = 151   ///< Command for setting the max dc (W-FYD)
};

//==============================================================================
//                                                                    PARAMETERS
//==============================================================================
/** \name W-FYD Parameters */
/** \{ */

enum w_fyd_parameter
{
    PARAM_POWER_TENSION          = 27,  ///< Device power tension
    PARAM_PULSE_MODALITY         = 28,  ///< Pulse modality byte (W-FYD)
    PARAM_SUP_PRESSURE_BOUND     = 29,  ///< Systolic pressure value setting (W-FYD)
    PARAM_INF_PRESSURE_BOUND     = 30,  ///< Diastolic pressure value setting (W-FYD)
    PARAM_PULSE_FREQ             = 31   ///< Pulse frequency setting (W-FYD)
};
//** \} */

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */