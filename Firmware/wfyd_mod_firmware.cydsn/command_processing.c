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
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         Dic. 1, 2015
* \author       qbrobotics
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include <stdio.h>
#include <interruptions.h>
#include <utils.h>

#include "commands.h"

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(){
    
    uint8 CYDATA rx_cmd;
    int16 ind;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }

    switch(rx_cmd){
//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
            cmd_get_measurements();
            break;
            
//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            cmd_get_inputs();
            break;

//============================================================     CMD_SET_SERVO
        case CMD_SET_SERVO:
            ind = *((int16 *) &g_rx.buffer[1]); 
            set_servo(ind);
            break;
            
//===============================================================     CMD_GET_IR
        case CMD_GET_IR:
            cmd_get_ir();
            break;
            
//============================================================     CMD_GET_SERVO
        case CMD_GET_SERVO:
            cmd_get_servo();
            break;
            
//============================================================     CMD_GET_FORCE
        case CMD_GET_FORCE:
            cmd_get_force();
            break;
            
//======================================================     CMD_GET_DUTY_CY_MAX
        case CMD_GET_DUTY_CY_MAX:
            cmd_get_duty_cy_max();
            break;
            
//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            cmd_get_currents();
            break;

//====================================================     CMD_GET_CURR_AND_MEAS

        case CMD_GET_CURR_AND_MEAS:
            cmd_get_curr_and_meas();
            break;
            
//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            cmd_set_inputs();
            break;
            
//=======================================================     CMD_GET_VELOCITIES

        case CMD_GET_VELOCITIES:
            cmd_get_velocities();
            break;

//=============================================================     CMD_ACTIVATE
        case CMD_ACTIVATE:
            cmd_activate();
            break;
            
//=============================================================     CMD_WATCHDOG
            
        case CMD_SET_WATCHDOG:
            cmd_set_watchdog();
            break;
            
//=========================================================     CMD_GET_ACTIVATE
            
        case CMD_GET_ACTIVATE:
            cmd_get_activate();
            break;
            
//=========================================================     CMD_SET_BAUDRATE
            
        case CMD_SET_BAUDRATE:
            cmd_set_baudrate();
            break;  
            
//=============================================================     CMD_GET_INFO
            
        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]));
            break;

//============================================================     CMD_SET_PARAM
            
        case CMD_SET_ZEROS:
            setZeros();
            break;

//============================================================     CMD_GET_PARAM
            
        case CMD_GET_PARAM_LIST:
            get_param_list( *((uint16 *) &g_rx.buffer[1]));
            break;

//=================================================================     CMD_PING
            
        case CMD_PING:
            cmd_ping();
            break;

//=========================================================     CMD_STORE_PARAMS
            
        case CMD_STORE_PARAMS:
            cmd_store_params();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS
            
        case CMD_STORE_DEFAULT_PARAMS:
            if ( memStore(DEFAULT_EEPROM_DISPLACEMENT) ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            if ( memRestore() ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
            break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if ( memInit() ) {
                sendAcknowledgment(ACK_OK);
            } else {
                sendAcknowledgment(ACK_ERROR);
            }
            break;

//===========================================================     CMD_BOOTLOADER
        case CMD_BOOTLOADER:
            sendAcknowledgment(ACK_OK);
            CyDelay(1000);
            FTDI_ENABLE_REG_Write(0x00);
            CyDelay(1000);
            Bootloadable_Load();
            break;

//=========================================================== ALL OTHER COMMANDS
        default:
            break;

    }
}

//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type){
    unsigned char packet_string[1100];

//======================================     choose info type and prepare string

    switch (info_type) {
        case INFO_ALL:
            infoPrepare(packet_string);
            UART_RS485_PutString(packet_string);
            break;
        default:
            break;
    }
}

//==============================================================================
//                                                                     SET ZEROS
//==============================================================================

void setZeros()
{
    uint8 CYDATA i;        // iterator

    for(i = 0; i < NUM_OF_SENSORS; ++i) {
        g_mem.m_off[i] = - *((int16 *) &g_rx.buffer[1 + i * 2]);
        // Encoder in the front of the motor
        g_mem.m_off[i] = -g_mem.m_off[i];
        g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

        g_meas.rot[i] = 0;
    }
    reset_last_value_flag = 1;

    sendAcknowledgment(ACK_OK);
}

//==============================================================================
//                                                            GET PARAMETER LIST
//==============================================================================

void get_param_list(uint16 index)
{
    //Package to be sent variables
    uint8 packet_data[1351] = "";
    uint16 packet_lenght = 1351;

    //Auxiliary variables
    uint16 CYDATA i;
    uint8 string_lenght;
    int32 aux_int;

    //Parameters menu string definitions
    char id_str[15]             = "1 - Device ID:";
    char pos_pid_str[28]        = "2 - Position PID [P, I, D]:";
    char startup_str[28]        = "3 - Startup Activation:";
    char res_str[17]            = "4 - Resolutions:";
    char m_off_str[25]          = "5 - Measurement Offsets:";
    char mult_str[17]           = "6 - Multipliers:";
    char pos_lim_flag_str[28]   = "7 - Pos. limit active:";
    char pos_lim_str[28]        = "8 - Pos. limits [inf, sup]:";
    char max_step_str[26]       = "9 - Max steps [neg, pos]:";
    char curr_limit_str[20]     = "10 - Current limit:";
    char pulse_mod_str[28]      = "11 - Pulse modality:";
    char pressure_bounds_str[35]= "12 - Pressure bounds [max, min]:";
    char pulse_freq_str[22]     = "13 - Pulse frequency:";
    char pow_tension_str[22]    = "14 - Power tension:";
    
    //Parameters menus
    char yes_no_menu[42] = "0 -> Deactivate [NO]\n1 -> Activate [YES]\n";

    //Strings lenghts
    uint8 CYDATA id_str_len = strlen(id_str);
    uint8 CYDATA pos_pid_str_len = strlen(pos_pid_str);

    uint8 CYDATA res_str_len = strlen(res_str);
    uint8 CYDATA m_off_str_len = strlen(m_off_str);
    uint8 CYDATA mult_str_len = strlen(mult_str);

    uint8 CYDATA pos_lim_str_len = strlen(pos_lim_str);
    uint8 CYDATA curr_limit_str_len = strlen(curr_limit_str);
    
    uint8 CYDATA pow_tension_str_len = strlen(pow_tension_str);
    
    uint8 CYDATA yes_no_menu_len = strlen(yes_no_menu);

    packet_data[0] = CMD_GET_PARAM_LIST;
    packet_data[1] = NUM_OF_PARAMS;

    switch(index) {
        case 0:         //List of all parameters with relative types
            /*-----------------ID-----------------*/

            packet_data[2] = TYPE_UINT8;
            packet_data[3] = 1;
            packet_data[4] = c_mem.id;
            for(i = id_str_len; i != 0; i--)
                packet_data[5 + id_str_len - i] = id_str[id_str_len - i];

            /*-------------POSITION PID-----------*/

            packet_data[52] = TYPE_FLOAT;
            packet_data[53] = 3;
            *((float *) (packet_data + 54)) = (float) c_mem.k_p / 65536;
            *((float *) (packet_data + 58)) = (float) c_mem.k_i / 65536;
            *((float *) (packet_data + 62)) = (float) c_mem.k_d / 65536;
            for(i = pos_pid_str_len; i != 0; i--)
                packet_data[66 + pos_pid_str_len - i] = pos_pid_str[pos_pid_str_len - i];

            /*----------STARTUP ACTIVATION--------*/

            packet_data[102] = TYPE_FLAG;
            packet_data[103] = 1;
            packet_data[104] = c_mem.activ;
            if(c_mem.activ) {
                strcat(startup_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(startup_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[105 + string_lenght - i] = startup_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[105 + string_lenght]  = 3;
            
            /*-------------RESOLUTIONS------------*/
            
            packet_data[152] = TYPE_UINT8;
            packet_data[153] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                packet_data[i + 154] = c_mem.res[i];
            for(i = res_str_len; i != 0; i--)
                packet_data[157 + res_str_len - i] = res_str[res_str_len - i];
            
            /*----------MEASUREMENT OFFSET--------*/
            
            packet_data[202] = TYPE_INT16;
            packet_data[203] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++) 
                *((int16 *) ( packet_data + 204 + (i * 2) )) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);
            for(i = m_off_str_len; i != 0; i--)
                packet_data[210 + m_off_str_len - i] = m_off_str[m_off_str_len - i];
            
            /*------------MULTIPLIERS-------------*/
            
            packet_data[252] = TYPE_FLOAT;
            packet_data[253] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                *((float *) ( packet_data + 254 + (i * 4) )) = c_mem.m_mult[i];
            for(i = 0; i < strlen(mult_str); i++)
                packet_data[266 + i] = mult_str[i];

            /*-----------POS LIMIT FLAG-----------*/
            
            packet_data[302] = TYPE_FLAG;
            packet_data[303] = 1;
            packet_data[304] = c_mem.pos_lim_flag;
            if(c_mem.pos_lim_flag) {
                strcat(pos_lim_flag_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(pos_lim_flag_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[305 + string_lenght - i] = pos_lim_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[305 + string_lenght] = 3;
            
            /*-----------POSITION LIMITS----------*/
            
            packet_data[352] = TYPE_INT32;
            packet_data[353] = 4;
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                *((int32 *)( packet_data + 354 + (i * 2 * 4) )) = (c_mem.pos_lim_inf[i] >> c_mem.res[i]);
                *((int32 *)( packet_data + 354 + (i * 2 * 4) + 4)) = (c_mem.pos_lim_sup[i] >> c_mem.res[i]);
            }
            for(i = pos_lim_str_len; i != 0; i--)
                packet_data[370 + pos_lim_str_len - i] = pos_lim_str[pos_lim_str_len - i];

            /*--------------MAX STEPS-------------*/
            
            packet_data[402] = TYPE_INT32;
            packet_data[403] = 2;
            *((int32 *)(packet_data + 404)) = c_mem.max_step_neg;
            *((int32 *)(packet_data + 408)) = c_mem.max_step_pos;
            for(i = 0; i < strlen(max_step_str); i++)
                packet_data[412 + i] = max_step_str[i];

            /*------------CURRENT LIMIT-----------*/

            packet_data[452] = TYPE_INT16;
            packet_data[453] = 1;
            *((int16 *)(packet_data + 454)) = c_mem.current_limit;
            for(i = curr_limit_str_len; i != 0; i--)
                packet_data[456 + curr_limit_str_len - i] = curr_limit_str[curr_limit_str_len - i];

            /*-----------PULSE MODALITY-----------*/

            packet_data[502] = TYPE_FLAG;
            packet_data[503] = 1;
            packet_data[504] = c_mem.flag_pulse;
            if(c_mem.flag_pulse) {
                strcat(pulse_mod_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(pulse_mod_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[505 + string_lenght - i] = pulse_mod_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[505 + string_lenght]  = 3;

            /*-----------PRESSURE BOUNDS----------*/

            packet_data[552] = TYPE_INT32;
            packet_data[553] = 2;
            *((int32 *)(packet_data + 554)) = c_mem.thr_max_pressure;
            *((int32 *)(packet_data + 558)) = c_mem.thr_min_pressure;
            for(i = 0; i < strlen(pressure_bounds_str); i++)
                packet_data[562 + i] = pressure_bounds_str[i];

            /*----------PULSE FREQUENCY-----------*/

            packet_data[602] = TYPE_INT32;
            packet_data[603] = 1;
            *((int32 *)(packet_data + 604)) = c_mem.pulse_freq;
            for(i = 0; i < strlen(max_step_str); i++)
                packet_data[608 + i] = pulse_freq_str[i];

            /*------------POWER TENSION----------*/

            packet_data[652] = TYPE_UINT16;
            packet_data[653] = 1;
            *((uint16 *)(packet_data + 654)) = c_mem.power_tension;
            for(i = pow_tension_str_len; i!= 0; i--)
                packet_data[656 + pow_tension_str_len - i] = pow_tension_str[pow_tension_str_len - i];

            /*-----------PARAMETERS MENU----------*/
                
            for(i = yes_no_menu_len; i!= 0; i--)
                packet_data[702 + yes_no_menu_len - i] = yes_no_menu[yes_no_menu_len - i];

            packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght, FALSE);
            UART_RS485_ClearTxBuffer();
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;
        
//=======================================================     set_pid_parameters
        case 2:         //Position PID - float[3]
            g_mem.k_p = *((float *) &g_rx.buffer[3]) * 65536;
            g_mem.k_i = *((float *) &g_rx.buffer[3 + 4]) * 65536;
            g_mem.k_d = *((float *) &g_rx.buffer[3 + 8]) * 65536;
        break;

//===================================================     set_startup_activation        
        case 3:         //Startup flag - uint8
            if (g_rx.buffer[3])
                g_mem.activ = 0x03;
            else
                g_mem.activ = 0x00;
        break;
        
//===========================================================     set_resolution
        case 4:         //Resolution - uint8[3]
            for (i =0; i < NUM_OF_SENSORS; i++)
                g_mem.res[i] = g_rx.buffer[i+3];
        break;
        
//===============================================================     set_offset
        case 5:         //Measurement Offset - int32[3] 
            for(i = 0; i < NUM_OF_SENSORS; ++i) {
                g_mem.m_off[i] = *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            reset_last_value_flag = 1;
        break;
        
//===========================================================     set_multiplier
        case 6:         //Multipliers - float[3]
            for(i = 0; i < NUM_OF_SENSORS; ++i)
                g_mem.m_mult[i] = *((float *) &g_rx.buffer[3 + i * 4]);
        break;
        
//=====================================================     set_pos_limit_enable
        case 7:        //Position limit flag - uint8
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
        break;

//============================================================     set_pos_limit
        case 8:        //Position limits - int32[4]
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                g_mem.pos_lim_inf[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4)]);
                g_mem.pos_lim_sup[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4) + 4]);

                g_mem.pos_lim_inf[i] = g_mem.pos_lim_inf[i] << g_mem.res[i];
                g_mem.pos_lim_sup[i] = g_mem.pos_lim_sup[i] << g_mem.res[i];
            }
        break;

//==================================================     set_max_steps_per_cycle
        case 9:        //Max steps - int32[2]
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int <= 0)
                g_mem.max_step_neg = aux_int;

            aux_int = *((int32 *) &g_rx.buffer[3 + 4]);
            if (aux_int >= 0) 
                g_mem.max_step_pos = aux_int;
        break;
        
//========================================================     set_current_limit
        case 10:        //Current limit - int16
            g_mem.current_limit = *((int16*) &g_rx.buffer[3]);
        break;

//=======================================================     set_pulse_modality
        case 11:
            g_mem.flag_pulse = g_rx.buffer[3];
            
            if (g_mem.flag_pulse) {
                g_ref.onoff = 0x03;
                MOTOR_ON_OFF_Write(g_ref.onoff);
                g_ref.pos[0] = ((int16)-2276) << c_mem.res[0];
                g_ref.pos[1] = ((int16)2276) << c_mem.res[1];
            } else {
                g_ref.pos[0] = ((int16)0) << c_mem.res[0];
                g_ref.pos[1] = ((int16)0) << c_mem.res[1];
                g_ref.onoff = 0x00;
                MOTOR_ON_OFF_Write(g_ref.onoff);
            }
        break;

//======================================================     set_pressure_bounds
        case 12: 
            g_mem.thr_max_pressure = *((int32 *) &g_rx.buffer[3]);
            g_mem.thr_min_pressure = *((int32 *) &g_rx.buffer[3 + 4]);
        break;

//======================================================     set_pulse_frequency
        case 13:
            g_mem.pulse_freq = *((int32 *) &g_rx.buffer[3]);
        break;

//========================================================     set_power_tension
        case 14:
            g_mem.power_tension = *((uint16*) &g_rx.buffer[3]);
        break;
    }
}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int CYDATA i;

    unsigned char str[50];
    
    strcpy(info_string, "");
    strcat(info_string, "\r\n");
    strcat(info_string, "Firmware version: ");
    strcat(info_string, VERSION);
    strcat(info_string, ".\r\n\r\n");

    strcat(info_string, "DEVICE INFO\r\n");
    sprintf(str, "ID: %d\r\n", (int) c_mem.id);
    strcat(info_string, str);
    sprintf(str, "Number of sensors: %d\r\n", (int) NUM_OF_SENSORS);
    strcat(info_string, str);
    sprintf(str, "Supply tension: %d\r\n", c_mem.power_tension);
    strcat(info_string, str);
    sprintf(str, "PWM Limit: %d\r\n", (int) dev_pwm_limit);
    strcat(info_string, str);
    strcat(info_string, "\r\n");

    strcat(info_string, "MOTOR INFO\r\n");
    strcat(info_string, "Motor references: ");
    
    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "%d ", (int)(g_refOld.pos[i] >> c_mem.res[i]));
        strcat(info_string,str);
    }
    strcat(info_string,"\r\n");

    sprintf(str, "Motor enabled: ");
    
    if (g_refOld.onoff & 0x03) {
        strcat(str,"YES\r\n");
    } else {
        strcat(str,"NO\r\n");
    }
    strcat(info_string, str);


    strcat(info_string,"\r\nMEASUREMENTS INFO\r\n");
    strcat(info_string, "Sensor value:\r\n");
    for (i = 0; i < NUM_OF_SENSORS; i++) {
        sprintf(str,"%d -> %d", i+1,
            (int)(g_measOld.pos[i] >> c_mem.res[i]));
        strcat(info_string, str);
        strcat(info_string, "\r\n");
    }
    sprintf(str,"Voltage (mV): %ld", (int32) dev_tension );
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str,"Current 1 (mA): %ld", (int32) g_measOld.curr[0] );
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str,"Current 2 (mA): %ld", (int32) g_measOld.curr[1] );
    strcat(info_string, str);
    strcat(info_string,"\r\n");


    strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");

    strcat(info_string, "PID Controller:\r\n");
    sprintf(str,"P -> %f\r\n", ((double) c_mem.k_p / 65536));
    strcat(info_string, str);
    sprintf(str,"I -> %f\r\n", ((double) c_mem.k_i / 65536));
    strcat(info_string, str);
    sprintf(str,"D -> %f\r\n", ((double) c_mem.k_d / 65536));
    strcat(info_string, str);

    strcat(info_string,"\r\n");

    if (c_mem.activ == 0x03) {
        strcat(info_string, "Startup activation: YES\r\n");
    } else {
        strcat(info_string, "Startup activation: NO\r\n");
    }

    strcat(info_string, "Sensor resolution:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %d", (int) (i + 1),
            (int) c_mem.res[i]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
    }


    strcat(info_string, "Measurement Offset:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %ld", (int) (i + 1),
            (int32) c_mem.m_off[i] >> c_mem.res[i]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
    }

    strcat(info_string, "Measurement Multiplier:\r\n");
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        sprintf(str,"%d -> %f", (int)(i + 1),
            (double) c_mem.m_mult[i]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
    }

    sprintf(str, "Position limit active: %d", (int)g_mem.pos_lim_flag);
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        sprintf(str, "Position limit motor %d: inf -> %ld  ", (int)(i + 1),
                (int32)g_mem.pos_lim_inf[i] >> g_mem.res[i]);
        strcat(info_string, str);

        sprintf(str, "sup -> %ld\r\n",
                (int32)g_mem.pos_lim_sup[i] >> g_mem.res[i]);
        strcat(info_string, str);
    }

    sprintf(str, "Max stiffness: %d", (int)g_mem.max_stiffness >> g_mem.res[0]);
    strcat(info_string, str);
    strcat(info_string,"\r\n");

    sprintf(str, "Current limit: %d", (int)g_mem.current_limit);
    strcat(info_string, str);
    strcat(info_string,"\r\n");
    
    sprintf(str, "Force: %d", (int)g_meas.force);
    strcat(info_string, str);
    strcat(info_string,"\r\n");
    
    sprintf(str, "IR: %d", (int)g_meas.ir);
    strcat(info_string, str);
    strcat(info_string,"\r\n");
    
    sprintf(str, "debug: %ld", (uint32) timer_value0 - (uint32) timer_value);
    strcat(info_string, str);
    strcat(info_string, "\r\n");
}

//==============================================================================
//                                                     WRITE FUNCTIONS FOR RS485
//==============================================================================

void commWrite_old_id(uint8 *packet_data, uint16 packet_lenght, uint8 old_id)
{
    uint16 CYDATA index;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    //if(old_id)
        UART_RS485_PutChar(old_id);
    //else
        //UART_RS485_PutChar(g_mem.id);
        
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index) {
        UART_RS485_PutChar(packet_data[index]);
    }

    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}

void commWrite(uint8 *packet_data,const uint16 packet_lenght, uint8 next)
{
    uint16 CYDATA index;

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    
    // frame - ID
    if(next)
    // If next flag is set the message is sent to the device with ID with value
    // greater by one than the one that sends the message
        UART_RS485_PutChar((uint8) g_mem.id + 1);
    else
        UART_RS485_PutChar((uint8) g_mem.id);
    
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index)
        UART_RS485_PutChar(packet_data[index]);
    
    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    if(!next) {
        RS485_CTS_Write(1);
        RS485_CTS_Write(0);
    }
}

//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment(const uint8 value) {

    // Packet: header + crc
    
    uint8 CYDATA packet_data[2];

    // Header
    packet_data[0] = value;
    
    // Payload/CRC
    packet_data[1] = value;

    commWrite(packet_data, 2, FALSE);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================
/**
* This function stores current memory settings on the eeprom with the specified
* displacement
**/

uint8 memStore(int displacement) {

    uint8 writeStatus;
    int i;
    int pages;
    uint8 ret_val = 1;

    PWM_MOTORS_WriteCompare1(0);
    PWM_MOTORS_WriteCompare2(0);

    // Retrieve temperature for better writing performance
    EEPROM_UpdateTemperature();

    memcpy( &c_mem, &g_mem, sizeof(g_mem) );

    pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);

    for(i = 0; i < pages; ++i) {
        writeStatus = EEPROM_Write(&g_mem.flag + 16 * i, i + displacement);
        if(writeStatus != CYRET_SUCCESS) {
            ret_val = 0;
            break;
        }
    }

    memcpy( &g_mem, &c_mem, sizeof(g_mem) );

    return ret_val;
}


//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================
/**
* This function loads user settings from the eeprom.
**/

void memRecall(void) {

    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) 
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
   

    //check for initialization
    if (g_mem.flag == FALSE) 
        memRestore();
    else 
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================
/**
* This function loads default settings from the eeprom.
**/

uint8 memRestore(void) {

    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) 
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    

    //check for initialization
    if (g_mem.flag == FALSE) 
        return memInit();
     else 
        return memStore(0);
   
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================
/**
* This function initialize memory when eeprom is compromised.
**/

uint8 memInit(void) {

    uint8 CYDATA i;
    //initialize memory settings
    g_mem.id                =   1;
    g_mem.k_p               =   0.1 * 65536;
    g_mem.k_i               =   0 * 65536;
    g_mem.k_d               =   0.8 * 65536;
    
    g_mem.activ             =   1;
    g_mem.watchdog_period   =   0; //MAX_WATCHDOG_TIMER;

    g_mem.pos_lim_flag = 1;
    
    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        g_mem.m_mult[i] = 1;
        g_mem.res[i] = 3;
    }

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        g_mem.pos_lim_inf[i] = -15000 << g_mem.res[i];
        g_mem.pos_lim_sup[i] =  15000 << g_mem.res[i];
    }
    
    g_mem.thr_max_pressure = 120;
    g_mem.thr_max_force = DEFAULT_MAX_FORCE_PULSE;  // Correspond to 120mmHg
    g_mem.thr_min_pressure = 60;
    g_mem.thr_min_force = DEFAULT_MIN_FORCE_PULSE;  // correspond to 60mmHg
    g_mem.step_const = 0.00183;                     // Freq = 110bpm
    g_mem.pulse_freq = 110;
    g_mem.flag_pulse = 0;
    
    g_mem.power_tension = 6000;         //mV of needed supply power

    g_mem.m_off[0] = (int32)0 << g_mem.res[0];
    g_mem.m_off[1] = (int32)0 << g_mem.res[1];

    g_mem.max_stiffness = (int32)3000 << g_mem.res[0];

    g_mem.current_limit = DEFAULT_CURRENT_LIMIT;

    //set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;

    //write that configuration to EEPROM
    return ( memStore(0) && memStore(DEFAULT_EEPROM_DISPLACEMENT) );
}

//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================
/**
* Bunch of functions used on request from UART communication
**/

void cmd_get_measurements(){
   
    
    uint8 CYDATA index;
   
    // Packet: header + measure(int16) + crc
    
    uint8 packet_data[6];

    //Header package
    packet_data[0] = CMD_GET_MEASUREMENTS;   
    
    for (index = NUM_OF_SENSORS; index--;) 
        *((int16 *) &packet_data[(index << 1) + 1]) = (int16)(g_measOld.pos[index] >> g_mem.res[index]);
            
    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum (packet_data, 5);
    commWrite(packet_data, 6, FALSE);
    
}

void cmd_get_inputs(){

    // Packet: header + motor_measure(int16) + crc

    uint8 packet_data[6]; 
    
    //Header package

    packet_data[0] = CMD_GET_INPUTS;
    
    *((int16 *) &packet_data[1]) = (int16) (g_refOld.pos[0]  >> g_mem.res[0]);
    *((int16 *) &packet_data[3]) = (int16) (g_refOld.pos[1]  >> g_mem.res[1]);
    
    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum(packet_data, 5);

    commWrite(packet_data, 6, FALSE);

}

void cmd_get_currents(){
    
    // Packet: header + motor_measure(int16) + crc
    
    uint8 packet_data[6]; 
    
    //Header package

    packet_data[0] = CMD_GET_CURRENTS;

    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr[0];
    *((int16 *) &packet_data[3]) = (int16) g_measOld.curr[1];

    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum (packet_data, 5);

    commWrite(packet_data, 6, FALSE);
}

void cmd_get_curr_and_meas(){
    
    uint8 CYDATA index;
   
    //Packet: header + curr_meas(int16) + pos_meas(int16) + CRC
    
    uint8 packet_data[10];

    //Header package
    packet_data[0] = CMD_GET_CURR_AND_MEAS;
    
    // Currents
    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr[0];
    *((int16 *) &packet_data[3]) = (int16) g_measOld.curr[1];

    // Positions
    for (index = NUM_OF_SENSORS; index--;) 
        *((int16 *) &packet_data[(index << 2) + 5]) = (int16) (g_measOld.pos[index] >> g_mem.res[index]);
        
    // Calculate Checksum and send message to UART 
        
    #if (NUM_OF_SENSORS == 4)
        packet_data[13] = LCRChecksum (packet_data, 13);
        commWrite(packet_data, 14, FALSE);
    #endif
    #if  (NUM_OF_SENSORS == 3)
        packet_data[11] = LCRChecksum (packet_data, 11);
        commWrite(packet_data, 12, FALSE);
    #endif
}

void cmd_set_inputs(){
    
    // Store position setted in right variables
    g_refNew.pos[0] = *((int16 *) &g_rx.buffer[1]);   // motor 1
    g_refNew.pos[0] = g_refNew.pos[0] << g_mem.res[0];

    g_refNew.pos[1] = *((int16 *) &g_rx.buffer[3]);   // motor 2
    g_refNew.pos[1] = g_refNew.pos[1] << g_mem.res[1];

    // Check Position Limit cmd
    if (c_mem.pos_lim_flag) {                      
        
        if (g_refNew.pos[0] < c_mem.pos_lim_inf[0]) 
            g_refNew.pos[0] = c_mem.pos_lim_inf[0];
        if (g_refNew.pos[1] < c_mem.pos_lim_inf[1]) 
            g_refNew.pos[1] = c_mem.pos_lim_inf[1];

        if (g_refNew.pos[0] > c_mem.pos_lim_sup[0]) 
            g_refNew.pos[0] = c_mem.pos_lim_sup[0];
        if (g_refNew.pos[1] > c_mem.pos_lim_sup[1]) 
            g_refNew.pos[1] = c_mem.pos_lim_sup[1];
    }
}

void cmd_get_velocities(){
    
    uint8 CYDATA index;
    
    // Packet: header + measure(int16) + crc

    uint8 packet_data[6];
    
    //Header package
  
    packet_data[0] = CMD_GET_VELOCITIES;   
   
    for (index = NUM_OF_SENSORS; index--;)
        *((int16 *) &packet_data[(index << 2) + 1]) = (int16)(g_measOld.vel[index]);

    // Calculate Checksum and send message to UART 
    packet_data[5] = LCRChecksum (packet_data, 5);
    commWrite(packet_data, 6, FALSE);
}

void cmd_activate(){
    
    // Store new value reads
    g_refNew.onoff = g_rx.buffer[1];
    
    // Check type of control mode enabled
    g_refNew.pos[0] = g_meas.pos[0];
    g_refNew.pos[1] = g_meas.pos[1];
    
    
    // Activate/Disactivate motors
    MOTOR_ON_OFF_Write(g_refNew.onoff);

}

void cmd_set_watchdog(){
      
    if (g_rx.buffer[1] <= 0){
        // Deactivate Watchdog
        WATCHDOG_ENABLER_Write(1); 
        g_mem.watchdog_period = 0;   
    }
    else{
        // Activate Watchdog        
        if (g_rx.buffer[1] > MAX_WATCHDOG_TIMER)
            g_rx.buffer[1] = MAX_WATCHDOG_TIMER;
            
        // Period * Time_CLK = WDT
        // Period = WTD / Time_CLK =     (WTD    )  / ( ( 1 / Freq_CLK ) )
        // Set request watchdog period - (WTD * 2)  * (250 / 1024        )
        g_mem.watchdog_period = (uint8) (((uint32) g_rx.buffer[1] * 2 * 250 ) >> 10);   
        WATCHDOG_COUNTER_WritePeriod(g_mem.watchdog_period); 
        WATCHDOG_ENABLER_Write(0); 
    }
}

void cmd_get_activate(){
    
    uint8 packet_data[3];

    // Header        
    packet_data[0] = CMD_GET_ACTIVATE;
    
    // Fill payload
    packet_data[1] = g_ref.onoff;
    
    // Calculate checksum
    packet_data[2] = LCRChecksum(packet_data, 2);
    
    // Send package to UART
    commWrite(packet_data, 3, FALSE);

}

void cmd_ping(){

    uint8 packet_data[2];

    // Header
    packet_data[0] = CMD_PING;
    
    // Load Payload
    packet_data[1] = CMD_PING;

    // Send Package to uart
    commWrite(packet_data, 2, FALSE);
}

void cmd_store_params(){
    
    uint8 CYDATA packet_lenght = 2;
    uint8 CYDATA packet_data[2];
    uint8 CYDATA old_id;
    
    if (c_mem.m_mult[0] != g_mem.m_mult[0]){
        // Old m_mult
        g_refNew.pos[0] /= c_mem.m_mult[0];
        // New m_mult
        g_refNew.pos[0] *= g_mem.m_mult[0];
    }
    
    if (c_mem.m_mult[1] != g_mem.m_mult[1]){
        // Old m_mult
        g_refNew.pos[1] /= c_mem.m_mult[1];
        // New m_mult
        g_refNew.pos[1] *= g_mem.m_mult[1];
    }
    
    if (c_mem.m_off[0] != g_mem.m_off[0])
        g_refNew.pos[0] += g_mem.m_off[0] - c_mem.m_off[0];

    if (c_mem.m_off[1] != g_mem.m_off[1])
        g_refNew.pos[1] += g_mem.m_off[1] - c_mem.m_off[1];
        
    // Check position Limits
    if (c_mem.pos_lim_flag) {                   // position limiting
        if (g_refNew.pos[0] < c_mem.pos_lim_inf[0]) g_refNew.pos[0] = c_mem.pos_lim_inf[0];
        if (g_refNew.pos[1] < c_mem.pos_lim_inf[1]) g_refNew.pos[1] = c_mem.pos_lim_inf[1];

        if (g_refNew.pos[0] > c_mem.pos_lim_sup[0]) g_refNew.pos[0] = c_mem.pos_lim_sup[0];
        if (g_refNew.pos[1] > c_mem.pos_lim_sup[1]) g_refNew.pos[1] = c_mem.pos_lim_sup[1];
    }
    
    // Store params 
    if (c_mem.id != g_mem.id) {     //If a new id is going to be set we will lose communication 
        old_id = c_mem.id;          //after the memstore(0) and the ACK won't be recognised
        if(memStore(0)) {
            packet_data[0] = ACK_OK;
            packet_data[1] = ACK_OK;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }    
        else{
            packet_data[0] = ACK_ERROR;
            packet_data[1] = ACK_ERROR;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }
    }    
    else {
        if ( memStore(0) ) {
            sendAcknowledgment(ACK_OK);
        } else {
            sendAcknowledgment(ACK_ERROR);
        }
    }
}

void cmd_set_baudrate(){
    
    // Set BaudRate
    c_mem.baud_rate = g_rx.buffer[1];
    
    switch(g_rx.buffer[1]){
        case 13:
            CLOCK_UART_SetDividerValue(13);
            break;
        default:
            CLOCK_UART_SetDividerValue(3);
    }
}

void cmd_get_ir(){
    // Packet: header + measure(int16) + crc
    uint8 CYDATA packet_lenght = 4;
    uint8 CYDATA packet_data[4];
    
    packet_data[0] = CMD_GET_IR;   //header
    *((int16 *) &packet_data[1]) = (int16) g_measOld.ir;
    packet_data[3] = LCRChecksum (packet_data, packet_lenght - 1);
    commWrite(packet_data, 4, FALSE);
}

void cmd_get_servo(){
    // Packet: header + measure(int16) + crc
    uint8 CYDATA packet_lenght = 4;
    uint8 CYDATA packet_data[4];
    
    get_servo();
    packet_data[0] = CMD_GET_SERVO;   //header
    *((int16 *) &packet_data[1]) = (int16) g_measOld.servo;
    packet_data[3] = LCRChecksum (packet_data, packet_lenght - 1);
    commWrite(packet_data, packet_lenght, FALSE);
}

void cmd_get_force(){
    // Packet: header + measure(int16) + crc
    uint8 CYDATA packet_lenght = 4;
    uint8 CYDATA packet_data[4];
    
    packet_data[0] = CMD_GET_FORCE;   //header
    *((int16 *) &packet_data[1]) = (int16) g_measOld.force;
    packet_data[3] = LCRChecksum (packet_data, packet_lenght - 1);
    commWrite(packet_data, packet_lenght, FALSE);
}

void cmd_get_duty_cy_max(){
    // Packet: header + measure(int16) + crc
    uint8 CYDATA packet_lenght = 4;
    uint8 CYDATA packet_data[4];
    
    packet_data[0] = CMD_GET_DUTY_CY_MAX;   //header
    *((int16 *) &packet_data[1]) = (int16)g_measOld.duty_cycle_f;
    packet_data[3] = LCRChecksum (packet_data, packet_lenght - 1);
    commWrite(packet_data, packet_lenght, FALSE);
}

/* [] END OF FILE */