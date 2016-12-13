//==================================================================     defines


//=================================================================     includes

#include "../../qbAPI/src/qbmove_communications.h"
#include "../../qbAPI/src/w_fyd_communications.h"
#include "definitions.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <getopt.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <signal.h>
#include <assert.h>

#if defined(_WIN32) || defined(_WIN64)
    #include <windows.h>
	#define sleep(x) Sleep(1000 * x)
#endif

//===============================================================     structures


static const struct option longOpts[] = {
    { "set_inputs", required_argument, NULL, 's' },
    { "get_measurements", no_argument, NULL, 'g' },
    { "activate", no_argument, NULL, 'a' },
    { "deactivate", no_argument, NULL, 'd' },
    { "ping", no_argument, NULL, 'p' },
    { "serial_port", no_argument, NULL, 't' },
    { "verbose", no_argument, NULL, 'v' },
    { "help", no_argument, NULL, 'h' },
    { "set_zeros", no_argument, NULL, 'z'},
    { "get_currents", no_argument, NULL, 'c'},
    { "bootloader", no_argument, NULL, 'b'},
    { "get_velocities", no_argument, NULL, 'i'},
    { "get_accelerations", no_argument, NULL, 'o'},
	{ "get_ir", no_argument, NULL, 'I'},
	{ "set_servo", required_argument, NULL, 'S'},
	{ "get_servo", no_argument, NULL, 'G'},
	{ "get_force", no_argument, NULL, 'F'},
	{ "get_duty", no_argument, NULL, 'D'},
	{ "sinusoid", no_argument, NULL, 'N'},
    { "baudrate", required_argument, NULL, 'B'},
    { "set_watchdog", required_argument, NULL, 'W'},
    { "polling", no_argument, NULL, 'P'},
    { NULL, no_argument, NULL, 0 }
};

static const char *optString = "s:adgptvh?f:ljqxzkycbe:uoiW:PB:IS:GFDN";

struct global_args {
    int device_id;
    int flag_set_inputs;            ///< ./qbmove -s option 
    int flag_get_measurements;      ///< ./qbmove -g option 
    int flag_activate;              ///< ./qbmove -a option 
    int flag_deactivate;            ///< ./qbmove -d option 
    int flag_ping;                  ///< ./qbmove -p option 
    int flag_serial_port;           ///< ./qbmove -t option 
    int flag_verbose;               ///< ./qbmove -v option 
    int flag_set_zeros;             ///< ./qbmove -z option 
    int flag_get_currents;          ///< ./qbmove -c option 
    int flag_bootloader_mode;       ///< ./qbmove -b option
    int flag_get_velocities;        ///< ./qbmove -i option 
    int flag_get_accelerations;     ///< ./qbmove -o option
	int flag_get_ir;				///< ./qbmove -I option
	int flag_set_servo;				///< ./qbmove -S option
	int flag_get_servo;				///< ./qbmove -G option
	int flag_get_force;				///< ./qbmove -F option
	int flag_get_duty;				///< ./qbmove -D option
	int flag_sinusoid;				///< ./qbmove -N option
    int flag_set_baudrate;          ///< ./qbmove -R option 
    int flag_set_watchdog;          ///< ./qbmove -W option 
    int flag_polling;               ///< ./qbmove -P option 
    int flag_baudrate;              ///< ./qbmove -B option 

    short int inputs[NUM_OF_MOTORS];
    short int measurements[4];
    short int velocities[4];
    short int accelerations[4];
    short int measurement_offset[4];
    short int currents[NUM_OF_MOTORS];
	
	short int measurement_ir[1];
    short int measurement_servo[1];
	short int inputservo[1];
    short int measurement_force[1];
    short int meas_duty_cy_max[1];

    short int BaudRate;
    int save_baurate;
    short int WDT;

    FILE* emg_file;
    FILE* log_file_fd;
} global_args;  //multiple boards on multiple usb

struct position {
    float prec;
    float act;
} p1, p2;


//==========================================================    global variables

uint8_t resolution[4];         // sensors resolution set on the board

int ret;                                    //utility variable to store return values
int aux_int;

comm_settings comm_settings_1;


//=====================================================     function declaration

int open_port();
int port_selection();
int polling();


/** Display program usage, and exit.
 */
void display_usage( void );

/** Parse csv input file with values to be sent to the motors
 */
float** file_parser(char*, int*, int*);

/** CTRL-c handler 1
 */
void int_handler(int sig);

/** CTRL-c handler 2
 */
void int_handler_2(int sig);

/** CTRL-c handler 3
 */
void int_handler_3(int sig);

/** Baudrate functions
 */
int baudrate_reader();
int baudrate_writer(const int);


//==============================================================================
//                                                                      sinusoid
//==============================================================================

double elapsed_time (SYSTEMTIME current, SYSTEMTIME reference){
    double min , sec , mis , hour, timetotal;
    hour = (double)current.wHour - (double)reference.wHour;
    min = (double)current.wMinute - (double)reference.wMinute;
    sec = (double)current.wSecond - (double)reference.wSecond;
    mis = (double)current.wMilliseconds - (double)reference.wMilliseconds;
    timetotal = hour * 60*60 + min * 60 + sec + ( mis / 1000 );
    return timetotal;
}
SYSTEMTIME time_total_start, partial_current ;
SYSTEMTIME start, stop;

//==============================================================================
//                                                                     main loop
//==============================================================================


/** main loop
 */
int main (int argc, char **argv)
{

    int  i = 0;             // global counters
    int  k = 0;

    char aux_string[10000]; // used to store PING reply
    int  aux[3];             // used to store input during set_inputs

    int  option;             // used for processing options
    int  longIndex = 0;

    int sensor_num = 0;

    char aux_char;
	
	short int ind_cmd;
    double elap_time;
    double offset = 4.0;
    double Amplitude = 3.0; //1.5;
    double frequency = 5.0;

    // initializations

    global_args.device_id               = 0;
    global_args.flag_serial_port        = 0;
    global_args.flag_ping               = 0;
    global_args.flag_verbose            = 0;
    global_args.flag_activate           = 0;
    global_args.flag_deactivate         = 0;
    global_args.flag_get_measurements   = 0;
	global_args.flag_get_currents		= 0;
    global_args.flag_set_inputs         = 0;
    global_args.flag_set_zeros          = 0;
    global_args.flag_bootloader_mode    = 0;
    global_args.flag_get_velocities     = 0;
    global_args.flag_get_accelerations  = 0;
    global_args.flag_set_baudrate       = 0;
    global_args.flag_set_watchdog       = 0;
    global_args.flag_polling            = 0;
    global_args.flag_baudrate           = 0;
	global_args.flag_get_ir				= 0;
	global_args.flag_set_servo			= 0;
	global_args.flag_get_servo			= 0;
	global_args.flag_get_force			= 0;
	global_args.flag_get_duty			= 0;
	global_args.flag_sinusoid			= 0;

    global_args.BaudRate                = baudrate_reader();

    //===================================================     processing options

    while ((option = getopt_long( argc, argv, optString, longOpts, &longIndex )) != -1)
    {
        switch (option)
        {
            case 's':
                sscanf(optarg,"%d,%d", &aux[0], &aux[1]);
                global_args.inputs[0] = (short int) aux[0];
                global_args.inputs[1] = (short int) aux[1];
                global_args.flag_set_inputs = 1;
                break;
            case 'g':
                global_args.flag_get_measurements = 1;
                break;
            case 'a':
                global_args.flag_activate = 1;
                break;
            case 'd':
                global_args.flag_deactivate = 1;
                break;
            case 't':
                port_selection();
                break;
            case 'p':
                global_args.flag_ping = 1;
                break;
            case 'v':
                global_args.flag_verbose = 1;
                break;
            case 'z':
                global_args.flag_set_zeros = 1;
                break;
            case 'c':
                global_args.flag_get_currents = 1;
                break;
            case 'b':
                global_args.flag_bootloader_mode = 1;
                break;
            case 'i':
                global_args.flag_get_velocities = 1;
                break;
            case 'o':
                global_args.flag_get_accelerations = 1;
                break;
			case 'I':
				global_args.flag_get_ir = 1;
				break;
			case 'S':
				sscanf(optarg,"%d", &aux[0]);
                global_args.inputservo[0] = (short int) aux[0];
				global_args.flag_set_servo = 1;
				break;
			case 'G':
				global_args.flag_get_servo = 1;
				break;
			case 'F':
				memset(&global_args.measurement_force, 1, sizeof(short int));
				global_args.flag_get_force = 1;
				break;
			case 'D':
				global_args.flag_get_duty = 1;
				break;
			case 'N':
				global_args.flag_sinusoid = 1;
				break;
            case 'R':
                sscanf(optarg,"%d", &aux[0]);
                global_args.BaudRate = (short int) aux[0];

                global_args.flag_set_baudrate = 1;
                break;
            case 'W':
                sscanf(optarg,"%d", &aux[0]);
                global_args.WDT = (short int) aux[0];
                global_args.flag_set_watchdog = 1;
                break;
            case 'P':
                global_args.flag_polling = 1;
                break;
            case 'B':
                sscanf(optarg,"%d", &aux[0]);
                global_args.flag_baudrate = 1;
                global_args.save_baurate = (int) aux[0];
                break;
            case 'h':
            case '?':
            default:
                display_usage();
                return 0;
                break;
        }
    }

    if((optind == 1) | (global_args.flag_verbose & (optind == 2)))
    {
        display_usage();
        return 0;
    }

    //==================================================================     polling
    
    if (global_args.flag_polling)
        assert(polling());
    else{     
        if (!open_port()) {
            assert(port_selection());
     
            if (global_args.flag_polling)
                assert(polling());
            else
                assert(open_port());
        }
    }

    //====================================================     getting device id

    if (argc - optind == 1)
    {
        sscanf(argv[optind++],"%d",&global_args.device_id);
        if(global_args.flag_verbose)
            printf("Device ID:%d\n", global_args.device_id);
    }
    else if(global_args.flag_verbose)
        puts("No device ID was chosen. Running in broadcasting mode.");

    //=================================================================     ping

    // If ping... then DOESN'T PROCESS OTHER COMMANDS

    if(global_args.flag_ping)
    {
        if(global_args.flag_verbose)
            puts("Pinging serial port.");

        if(global_args.device_id) {
            commGetInfo(&comm_settings_1, global_args.device_id, INFO_ALL, aux_string);
        } else {
            RS485GetInfo(&comm_settings_1,  aux_string);
        }

       puts(aux_string);

        if(global_args.flag_verbose)
            puts("Closing the application.");

        return 0;
    }


//===============================================================     set inputs

    if(global_args.flag_set_inputs)
    {
        if(global_args.flag_verbose)
            printf("Setting inputs to %d and %d.\n", global_args.inputs[0], global_args.inputs[1]);

        commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);
	}


//=========================================================     get measurements

    if(global_args.flag_get_measurements)
    {
        if(global_args.flag_verbose)
            puts("Getting measurements.");

        while(1) {
            sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);

            if(sensor_num < 0 || sensor_num > 4) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
                printf("measurements:     ");
                for (i = 0; i < sensor_num; i++) {
                    printf("%d  ", (int)global_args.measurements[i]);
                }
                printf("\n");

                usleep(100000);
            }
        }
    }


//===========================================================     get velocities

    if(global_args.flag_get_velocities)
    {
        if(global_args.flag_verbose)
            puts("Getting velocities.");


        while(1) {
            sensor_num = commGetVelocities(&comm_settings_1, global_args.device_id, global_args.measurements);

            if(sensor_num < 0 || sensor_num > 4) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
                printf("velocities:     ");
                for (i = 0; i < sensor_num; i++) {
                    printf("%d  ", (int)global_args.velocities[i]);
                }
                printf("\n");

                usleep(100000);
            }
        }
    }

//===========================================================     get accelerations

    if(global_args.flag_get_accelerations)
    {
        if(global_args.flag_verbose)
            puts("Getting accelerations.");

        while(1) {
            sensor_num = commGetAccelerations(&comm_settings_1, global_args.device_id, global_args.accelerations);

            if(sensor_num < 0 || sensor_num > 4) {
                printf("An error occurred or the device is not supported\n");
                break;
            }
            else {
                printf("accelerations:     ");
                for (i = 0; i < sensor_num; i++) {
                    printf("%hd  ", global_args.accelerations[i]);
                }
                printf("\n");

                usleep(100000);
            }
        }
    }
	
//================================================================     set servo        

    if(global_args.flag_set_servo)
    {
        if(global_args.flag_verbose)
            printf("Setting indentation to servo to %d.\n", global_args.inputservo[0]);
		commSetServo(&comm_settings_1, global_args.device_id, global_args.inputservo);
    }
	
//=================================================================     sinusoid

    if(global_args.flag_sinusoid)
    {
        if(global_args.flag_verbose)
            printf("Sinusoid experiment.\n");
        GetSystemTime(&time_total_start);
        while(1)
        {
            GetSystemTime( &start );
            GetSystemTime( &partial_current );
            elap_time = elapsed_time ( partial_current, time_total_start );
            ind_cmd = offset + Amplitude*sin(frequency * elap_time);
			if(ind_cmd < 2){
   			 ind_cmd = 2;
			}
			if(ind_cmd > 15){
   			ind_cmd = 15;
			}

			global_args.inputservo[0] = 80 + ind_cmd * 4;
            commSetServo(&comm_settings_1, global_args.device_id, global_args.inputservo);
    }
    fflush(stdout);
    usleep(100000);
}

//====================================================================     get IR

	if(global_args.flag_get_ir)
    {
        if(global_args.flag_verbose)
            puts("Getting indentation from IR.");

        while(1) {
            commGetIR(&comm_settings_1, global_args.device_id, global_args.measurement_ir);

            printf("measurements IR:     ");

            printf("%d  ", (int)global_args.measurement_ir[0]);


            printf("\n");
            fflush(stdout);
            usleep(100000);
        }
    }
	
//================================================================      get force

	if(global_args.flag_get_force)
    {
        if(global_args.flag_verbose)
            puts("Getting force.");

        while(1) {
            commGetForce(&comm_settings_1, global_args.device_id, global_args.measurement_force);

            printf("measurements force:     ");

            printf("%d  ", (int)global_args.measurement_force[0]);

            printf("\n");
            fflush(stdout);
            usleep(100000);
        }
    }
	
//==========================================================    get duty cycle max

	if(global_args.flag_get_duty)
    {
        if(global_args.flag_verbose)
            puts("Getting duty cycle max.");

        while(1) {
            commGetDCyM(&comm_settings_1, global_args.device_id, global_args.meas_duty_cy_max);

            printf("measurements force:     ");

            printf("%d  ", (int)global_args.meas_duty_cy_max[0]);

            printf("\n");
            fflush(stdout);
            usleep(100000);
        }
    }
	
//===============================================================      get servo

	if(global_args.flag_get_servo)
    {
        if(global_args.flag_verbose)
            puts("Getting indentation from servo.");

        while(1) {
            commGetServo(&comm_settings_1, global_args.device_id,
                    global_args.measurement_servo);


            printf("measurements indentation servo:     ");

            printf("%d  ", (int)global_args.measurement_servo[0]);

            printf("\n");
            fflush(stdout);
            usleep(100000);
        }
    }

//==========================================================     bootloader mode

    if(global_args.flag_bootloader_mode)
    {
        printf("Are you sure you want to enter in bootloader mode? y/[N]\n");
        scanf("%c", &aux_char);
        if(aux_char == 'y' || aux_char == 'Y') {
            printf("Entering bootloader mode\n");
            if(commBootloader(&comm_settings_1, global_args.device_id) > 0)
                printf("DONE\n");
            else
                printf("An error occurred.\nRetry.\n");
        }
    }

//==========================================================     get_currents

    if(global_args.flag_get_currents)
    {
        if(global_args.flag_verbose)
            puts("Getting currents.");


        while(1) {
            commGetCurrents(&comm_settings_1, global_args.device_id, global_args.currents);

            printf("Current 1: %hd\t Current 2: %hd\n", global_args.currents[0], global_args.currents[1]);
            fflush(stdout);
            usleep(100000);
        }
    }


//=================================================================     activate

    if(global_args.flag_activate)
    {
        if(global_args.flag_verbose)
            puts("Turning device on.\n");
        
        commActivate(&comm_settings_1, global_args.device_id, 1);
        usleep(1000);
        commGetActivate(&comm_settings_1, global_args.device_id, &aux_char);

        printf("%c %d\n", aux_char, (int)aux_char);
    }


//===============================================================     deactivate

    if(global_args.flag_deactivate)
    {
        if(global_args.flag_verbose)
           puts("Turning device off.\n");

        commActivate(&comm_settings_1, global_args.device_id, 0);
    }

//===============================================================     baudrate

    if(global_args.flag_baudrate)
    {
        if(global_args.flag_verbose)
           puts("Save BaudRate.\n");

        if ((global_args.save_baurate != 460800) && (global_args.save_baurate != 2000000)){
            puts("Set default BaudRate, 2000000.");
            global_args.save_baurate = 2000000;
        }

        
        baudrate_writer(global_args.save_baurate);
    }


//=================================================================     watchdog

    if(global_args.flag_set_watchdog)
    {
        if(global_args.flag_verbose)
            puts("Set Watchdog timer to \n");

        if (global_args.WDT > MAX_WATCHDOG_TIME){
            printf("Watchdog Saturated to %d.\n", MAX_WATCHDOG_TIME);
            global_args.WDT = MAX_WATCHDOG_TIME;
        }
        else
            if (global_args.flag_verbose && (global_args.WDT <= 0)){
                puts("Watchdog DISABLED.\n");
                global_args.WDT = 0;
            }

        commSetWatchDog(&comm_settings_1, global_args.device_id, global_args.WDT);

    }



    //============================================================     set zeros
    if(global_args.flag_set_zeros)
    {
        struct timeval t_prec, t_act;
        struct timezone foo;

        signal(SIGINT, int_handler_2);
        printf("Press CTRL-C to set Zero Position\n\n");
        printf("Press return to proceed\n");
        getchar();

        sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, global_args.measurements);

        // Deactivate device to avoid motor movements
        commActivate(&comm_settings_1, global_args.device_id, 0);

        // Reset all the offsets
        for (i = 0; i < sensor_num; i++) {
            global_args.measurement_offset[i] = 0;
        }

        commSetZeros(&comm_settings_1, global_args.device_id, 
                    global_args.measurement_offset, sensor_num);


        //Display current values until CTRL-C is pressed
        gettimeofday(&t_prec, &foo);
        gettimeofday(&t_act, &foo);
        while(1) {
            while (1) {
                gettimeofday(&t_act, &foo);
                if (timevaldiff(&t_prec, &t_act) >= 200000) {
                    break;
                }
            }
            commGetMeasurements(&comm_settings_1, global_args.device_id,
                    global_args.measurements);
            for (i = 0; i < sensor_num; i++) {
                printf("%d\t", global_args.measurements[i]);
            }
            printf("\n");

            gettimeofday(&t_prec, &foo);
        }
    }
    //============================================================     baudrate

    if (global_args.flag_set_baudrate){


        if (((int) global_args.BaudRate == BAUD_RATE_T_460800) || ((int) global_args.BaudRate == BAUD_RATE_T_2000000))
            commSetBaudRate(&comm_settings_1, global_args.device_id, global_args.BaudRate);
        else
            printf("BaudRate request not supported. \n 0 -> 2000000 \n 1 -> 460800\n");

    }



//==========================     closing serial port and closing the application


    closeRS485(&comm_settings_1);

    if(global_args.flag_verbose)
        puts("Closing the application.");

#ifdef PHIDGETS_BRIDGE
    CPhidget_close((CPhidgetHandle)bridge);
    CPhidget_delete((CPhidgetHandle)bridge);
#endif

    return 1;
}


//==============================================================================
//                                                                port_selection
//==============================================================================

int port_selection() {
    int i;
    int aux_int;
    int num_ports = 0;
    char my_port[255];
    char ports[10][255];
    FILE *file;

    while(1) {
        num_ports = RS485listPorts(ports);

        if(num_ports) {
            puts("\nChoose the serial port for your QB:\n");

            for(i = 0; i < num_ports; ++i) {
                printf("[%d] - %s\n\n", i+1, ports[i]);
            }
            printf("Serial port: ");
            scanf("%d", &aux_int);
            getchar();

            if( aux_int && (aux_int <= num_ports) ) {
                strcpy(my_port, ports[aux_int - 1]);
            } else {
                puts("Choice not available");
                continue;
            }

            file = fopen(QBMOVE_FILE, "w+");
            if (file == NULL) {
                printf("Cannot open qbmove.conf\n");
            }
            fprintf(file,"serialport %s\n", my_port);
            fclose(file);
            return 1;

        } else {
            puts("No serial port available.");
            return 0;
        }
    }
}

//==============================================================================
//                                                               baudrate_writer
//==============================================================================

int baudrate_writer(const int baudrate) {

    FILE *file;

    file = fopen(QBMOVE_FILE_BR, "w+");
    
    if (file == NULL) 
        printf("Cannot open qbmove.conf\n");

    fprintf(file,"baudrate %d\n", baudrate);
    fclose(file);
    
    return 1;

}


//==============================================================================
//                                                               baudrate_reader
//==============================================================================

int baudrate_reader(){

    int br = 0;
    FILE* file;

    if(global_args.flag_verbose)
        puts("Reading BaudRate configuration files.");

    file = fopen(QBMOVE_FILE_BR, "r");

    if (file == NULL) {
        printf("Error operning file %s\n", QBMOVE_FILE_BR);
        return 0;
    }

    fscanf(file, "baudrate %d\n", &br);

    fclose(file);


    if (br == 460800)
        return BAUD_RATE_T_460800;
    else
        return BAUD_RATE_T_2000000;
}


//==============================================================================
//                                                                     open_port
//==============================================================================

int open_port() {
    FILE *file;
    char port[255];

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL) {
        printf("Error opening file %s\n", QBMOVE_FILE);
        return 0;
    }

    fscanf(file, "serialport %s\n", port);

    fclose(file);


    #if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux

        if (global_args.BaudRate == BAUD_RATE_T_460800)
            openRS485(&comm_settings_1, port , B460800);
        else
            openRS485(&comm_settings_1, port , B2000000);
    #else
        if (global_args.BaudRate == BAUD_RATE_T_460800)
           openRS485(&comm_settings_1, port , 460800);
        else
            openRS485(&comm_settings_1, port , 2000000);
    #endif

    if(comm_settings_1.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(100000);

    return 1;
}


//==============================================================================
//                                                                       polling
//==============================================================================

int polling() {
    FILE *file;
    char port[255];
    bool device_number = false;
    short int meas[3];
    int i, id;
    int sensor_num = 0;

    file = fopen(QBMOVE_FILE, "r");

    if (file == NULL) {
        printf("Error opening file %s\n", QBMOVE_FILE);
        return 0;
    }

    fscanf(file, "serialport %s\n", port);

    fclose(file);

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    openRS485(&comm_settings_1, port , B460800);
#else
    openRS485(&comm_settings_1, port , 460800);
#endif

        // Scan for 460800 Baud Rate devices
        printf("Devices Connect: BaudRate = 460800\n");
        printf("ID\tPos1\tPos2\tPosL\n");
        printf("=============================\n");

        for (id = 1; id < 128; ++id){
            sensor_num = commGetMeasurements(&comm_settings_1, id, meas);
            if (sensor_num > 0){

                printf("%d\t", id);
                for (i = 0; i < sensor_num; ++i) 
                    printf("%d\t", (int) meas[i]);

                printf("\n");

                device_number = true;
            }
        }
        if (device_number)
            printf("-----------------------------\n");
        else
            printf("NO DEVICE FOUND!\n\n");

    closeRS485(&comm_settings_1);

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    openRS485(&comm_settings_1, port , B2000000);
#else
    openRS485(&comm_settings_1, port , 2000000);
#endif

        device_number = false;

        // Scan for 2000000 Baud Rate devices
        printf("Devices Connect: BaudRate = 2000000\n");
        printf("ID\tPos1\tPos2\tPosL\n");
        printf("=============================\n");

        for (id = 1; id < 128; ++id){
            sensor_num = commGetMeasurements(&comm_settings_1, id, meas);
            if (sensor_num > 0){

                printf("%d\t", id);
                for (i = 0; i < sensor_num; ++i) 
                    printf("%d\t", (int) meas[i]);

                printf("\n");

                device_number = true;
            }
        }
        if (device_number)
            printf("-----------------------------\n");
        else
            printf("NO DEVICE FOUND!\n\n");
    
    closeRS485(&comm_settings_1);

    if(comm_settings_1.file_handle == INVALID_HANDLE_VALUE)
    {
        puts("Couldn't connect to the serial port.");
        return 0;
    }
    usleep(100000);

    return 1;
}


//==============================================================================
//                                                                   file_parser
//==============================================================================

/** Parse CSV file and return a pointer to a matrix of float dinamically
 *  allocated.  Remember to use free(pointer) in the caller
 */

float** file_parser( char* filename, int* deltat, int* num_values )
{
    FILE* filep;
    float** array = NULL;
    int i;
    filep = fopen(filename, "r");
    if (filep == NULL) perror ("Error opening file");
    else {
        //read first line
        fscanf(filep, "%d,%d", deltat, num_values);

        //alloc memory for the arrays
        array = (float**)malloc(2*sizeof(float*));
        array[0] = (float*)malloc(*num_values*sizeof(float));
        array[1] = (float*)malloc(*num_values*sizeof(float));

        //read num_values line of file and store them in array
        for(i=0; i<*num_values; i++) {
            fscanf(filep, "%f,%f", &array[0][i], &array[1][i]);
        }
    fclose(filep);
    }
    return array;
}

//==============================================================================
//                                                          CTRL-C interruptions
//==============================================================================

/** handle CTRL-C interruption 1
*/
void int_handler(int sig) {
    printf("\nForced quit!!!\n");

    // set motors to 0,0
    global_args.inputs[0] = 0;
    global_args.inputs[1] = 0;
    commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);


    exit(1);
}

/** handle CTRL-C interruption 2
*/
void int_handler_2(int sig) {
    int i;
    short int temp_meas[4];
    int sensor_num  = 0;

    sensor_num = commGetMeasurements(&comm_settings_1, global_args.device_id, temp_meas);

    if(sensor_num > 0 && sensor_num < 4) {
        printf("\n\nSetting zero position\n");

        //Set the offsets equal to minus current positions
        for (i = 0; i < sensor_num; i++) {
            global_args.measurement_offset[i] = -global_args.measurements[i];
        }

        if(commSetZeros(&comm_settings_1, global_args.device_id, global_args.measurement_offset, sensor_num)) {
            printf("\nAn error occurred while setting measurements offsets. Retry.\n");
            exit(1);
        }

        if (commStoreParams(&comm_settings_1, global_args.device_id)) {
            printf("Error saving params\n");
            exit(1);
        }

        sleep(1);

        // set motors to 0,0
        global_args.inputs[0] = 0;
        global_args.inputs[1] = 0;
        commSetInputs(&comm_settings_1, global_args.device_id, global_args.inputs);
    }
    else
        printf("Number of sensors not supported_ext\n");

    exit(1);
}

/** Handles the ctrl+c interruption to save the emg sensors measurements into a file
*/

void int_handler_3(int sig) {
    printf("Closing file and quitting application...\n");

    fclose(global_args.emg_file);

    printf("DONE\n");

    exit(1);
}

//==============================================================================
//                                                                 display usage
//==============================================================================

/** Display program usage, and exit.
*/

void display_usage( void )
{
    puts("================================================================================");
    printf("qbadmin version: %s\n", QBADMIN_VERSION);
    puts("================================================================================");
    puts("usage: qbadmin [id] [OPTIONS]" );
    puts("--------------------------------------------------------------------------------");
    puts("Options:");
    puts("");
    puts("================================================================================");
    puts("General commands");
    puts("================================================================================");
    puts(" -h, --help                       Shows this information.");
    puts(" -p, --ping                       Get info on the device.");
    puts(" -t, --serial_port                Set up serial port.");
    puts(" -W, --set_watchdog               Set up Watchdog ");
    puts("                                  [0 - 500] with step rate of 2 [cs]).");
    puts(" -P, --polling                    Call a polling search.");
    puts(" -B, --baudrate <value>           Set qbmove Baudrate communication "); 
    puts("                                  [460800 or 2000000].");
    puts(" -b, --bootloader                 Enter bootloader mode to update firmware.");
    puts(" -v, --verbose                    Verbose mode.");
    puts(" -s, --set_inputs <value,value>   Send reference inputs to the board.");
    puts(" -g, --get_measurements           Get measurements from the board.");
    puts(" -i, --get_velocities             Get velocities from the board.");
    puts(" -o, --get_accelerations          Get accelerations for the board");
    puts(" -c, --get_currents               Get motor currents");
    puts(" -a, --activate                   Activate the QB Move.");
    puts(" -d, --deactivate                 Deactivate the QB Move.");
    puts(" -z, --set_zeros                  Set zero position for all sensors");
    puts("");
	puts("================================================================================");
    puts("W-FYD commands");
    puts("================================================================================");
	puts(" -I, --ir                         Get measure of IR.");
    puts(" -S, --set_servo <value>          Send indentation to servo");
	puts(" -G, --get_force                  Get force measurements");
    puts(" -D, --get_duty_cy_max            Get duty cycle max value");
    puts("--------------------------------------------------------------------------------");
    puts("Examples:");
    puts("");
    puts("  qbadmin -p                      Get info on whatever device is connected.");
    puts("  qbadmin -t                      Set up serial port.");
    puts("  qbadmin 65 -s 10,10             Set inputs of device 65 to 10 and 10.");
    puts("  qbadmin 65 -g                   Get measurements from device 65.");
    puts("  qbadmin 65 -g -s 10,10          Set inputs of device 65 to 10");
    puts("                                  and 10, and get measurements.");
    puts("  qbadmin 65 -a                   Turn device 65 on.");
    puts("  qbadmin 65 -a                   Turn device 65 off.");
    puts("  qbadmin 65 -f filename          Pilot device 65 using file 'filename'");
    puts("================================================================================");
    /* ... */
    exit( EXIT_FAILURE );
}
/* [] END OF FILE */
