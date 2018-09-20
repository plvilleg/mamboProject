//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Programa principal para la obtención de los puntos GPS de la ubicación del submarino
//Versión 1.0
//
//
//./TestGPS_point -p /dev/ttyUSB0 0.001 0.001 0.001 
//
//
//
//
//
//
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////
// Standards libraries
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <time.h>
#include <signal.h>
//
//////////////////////////////

//////////////////////////////
//Piksi libraries
#include <libserialport.h> // Serialport library
extern "C" {
#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libsbp/navigation.h>
#include <libsbp/system.h>
}
//
//////////////////////////////

//////////////////////////////
// Special libraries 
#include "config.h"
#include "COMPASS.h"
#include "RELATIVE_DISTANCE.h"
//
//////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// Execution command
// g++ testCompass.cpp compass.cpp HMC5843.cpp ADXL345.cpp -o testcompass -O2 -lwiringPi -larmadillo
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// Global variables

volatile sig_atomic_t flag = 0; // interruption variable

const float PI = (atan(1)*4); // Declaration of PI variable

COMPASS comp; 		// Declaration of a object of class COMPASS
RelativeDistace dist;	// Declaration of a object of class RelativeDistace

fstream dataLogFile;	// Declaration of file variable
fstream eventlogFile;

// Serial variables
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

time_t seconds;
struct tm *timestamp;

char buffer3[80], buffer4[300]; // Buffers
char eventTime[50];

//  Default log dir
char defaultdataDIR[80] = "/home/pi/mamboProject/logs/data/";
char defaulteventDIR[80] = "/home/pi/mamboProject/logs/events/";


///////////////////////////////////////////
// Piksi variables

sbp_state_t sbp_state;	// State of the SBP parser.

// SBP structs that messages from Piksi will feed. 
msg_pos_llh_dep_a_t	pos_llh;
msg_gps_time_dep_a_t	gps_time;

/* SBP callback node must be statically allocated. Each message ID / callback pair
must have a unique sbp_msg_callbacks_node_t associated with it*/

static sbp_msg_callbacks_node_t pos_llh_node;
static sbp_msg_callbacks_node_t gps_time_node;
//
///////////////////////////////////////////

//
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// Piksi functions

/* Callback funtions to interpret SBP messages.
*  Every message ID has a callback associated with it to
*  receive and interpret the message payload
*/
void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	pos_llh = *(msg_pos_llh_dep_a_t *)msg; // Parse the message to msg_pos_llh_dep_a_t type
}


void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	gps_time = *(msg_gps_time_dep_a_t *)msg; // Parse the message to msg_gps_time_dep_a_t type
}


/////////////////////////////////////////////
//  This function configure the piksi
void sbp_setup(void)
{
	int ret = -5;
	
	/* SBP parser state must be initialized before sbp_process is called*/
	sbp_state_init(&sbp_state);

	/* Register a node and callback, and associate them with a specific message ID*/
	
	ret = sbp_register_callback(&sbp_state, 0x0201, &sbp_pos_llh_callback, NULL, &pos_llh_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x0100, &sbp_gps_time_callback, NULL, &gps_time_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	printf("SBP_SETUP_OK\n\n");
}

/////////////////////////////////////////////
// This function read the serial port that is connected to the piksi
u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
	(void)context;
	u32 result;
	result = sp_blocking_read(piksi_port, buff, n, 0);
	return result;
}
//
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// Port configuration

/////////////////////////////////////////////
// Help message
void usage(char *prog_name){
	fprintf(stderr, "usage: %s [-p serial port] ta tb tc fileName.csv\n", prog_name);
}
//
/////////////////////////////////////////////

/////////////////////////////////////////////
// This function setup the serial port
void setup_port()
{
	int result;
	
	result = sp_set_baudrate(piksi_port, 1000000);
	if(result != SP_OK){
		fprintf(stderr, "Cannot set port baud rate!\n");
		exit(EXIT_FAILURE);
	}

	result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
	if(result != SP_OK){
		fprintf(stderr, "Cannot set flow control!\n");
		exit(EXIT_FAILURE);
	}

	result = sp_set_bits(piksi_port, 8);
	if(result != SP_OK){
		fprintf(stderr, "Cannot set data bits!\n");
		exit(EXIT_FAILURE);
	}

	result = sp_set_parity(piksi_port, SP_PARITY_NONE);
	if(result != SP_OK){
		fprintf(stderr, "Cannot set parity!\n");
		exit(EXIT_FAILURE);
	}

	result =sp_set_stopbits(piksi_port, 1);
	if(result != SP_OK){
		fprintf(stderr, "Cannot set stop bits!\n");
		exit(EXIT_FAILURE);
	}
}
//
/////////////////////////////////////////////


//
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  This function converts deggree to radians
double toRadians(double degrees){
	return degrees * PI/180;
}
//
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
//  This function converts radians to degree
double toDegrees(double radians){
	return radians * 180/PI;
}
//
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Helper function of a crtl-c interruption
void exitFunction(int sig){
	flag = 1;
}
//
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// Create log file
bool createLogFiles(void){
	
	printf("Creating files logs..!\n");	
	
	time(&seconds);
	timestamp = localtime(&seconds);

	strftime(buffer3,sizeof(buffer3)-1,"%Y-%m-%d_%H%M%S",timestamp);
	strcat(buffer3,"_dataLog.cvs");
	strcat(defaultdataDIR,buffer3);
	dataLogFile.open(defaultdataDIR, ios::out); 

	memset(buffer3,'\0',80);
	strftime(buffer3,sizeof(buffer3)-1,"%Y-%m-%d_%H%M%S",timestamp);
	strcat(buffer3,"_eventLog.cvs");
	strcat(defaulteventDIR,buffer3);
	eventlogFile.open(defaulteventDIR, ios::out); 
	
	if(dataLogFile.is_open() && eventlogFile.is_open()){
		dataLogFile <<"Time" <<","<< "BUOY_latitude"<<"," <<"BUOY_longitude" <<"," <<"ROV_latitude" <<"," <<"ROV_longitude" <<"," <<"depth" <<"," <<"fixed_state" <<endl;
		eventlogFile <<"Time" <<","<< "event"<<endl;
		printf("File log successful created..!\n");
		return true;
	}
	else {
		return false;
	}



}	
//
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
//  Update eventLog
bool updateEvent(char *event){

	time(&seconds);
	timestamp = localtime(&seconds);

	strftime(eventTime,sizeof(eventTime)-1,"%Y%m%d_%H%M%S",timestamp);
	//sprintf(buffer4,"%2.8f,%2.8f,%2.8f,%2.8f,%2.3f,%d",lat_O_deg, lon_O_deg, lat_D_deg, lon_D_deg, depth, fixMode);
	
	eventlogFile << eventTime <<","<< event << endl;
}

//
/////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////
// This is the main function
int main(int argc, char **argv)
{

/////////////////////////////////////////
//  Local variables

	if(argc <= 1) { // If not passed argument that arise a failure
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	if(!createLogFiles()){
		printf("Files cannot be created, check the path..!\n");
		exit(EXIT_FAILURE);
	}

	updateEvent("Files succefull created.");

	int opt; // Option arg
	int result = 0; // State of configuration
	int ret=0; // State of Piksi messages
	
	double lat_O_deg, lon_O_deg, lat_O_rad, lon_O_rad, lat_D_rad, lon_D_rad, lat_D_deg, lon_D_deg, theta, gamma; // Geograpihc variables
	double distance, bearing, x, y; // Point variables
	double Ta, Tb, Tc, depth; // Values adquire from submarine
	int8_t fixMode;  //State of GPS pression	

		
	std::string timeA, timeB, timeC; // Variables to store the times
	std::string::size_type sz; // Get the size of a String

	// Store times
	timeA = argv[3];
	timeB = argv[4];
	timeC = argv[5];

	// Transform time from string to float
	Ta = std::stod(timeA,&sz);
	Tb = std::stod(timeB,&sz);
	Tc = std::stod(timeC,&sz);

	// This value must be adquire from the submarine
	depth = 5.0;

	
	comp.init();	// Init the compass
	dist.init();	// Init method that calculate distance

	// Set the position of the speakers
	dist.setSpeaker_1(c1X,c1Y); 
	dist.setSpeaker_2(c2X,c2X);
	dist.setSpeaker_3(c3X,c3X);
	
//
/////////////////////////////////////////


/////////////////////////////////////////
// Initial setup

	printf("Configuring Buoy..!\n");
	
	signal(SIGINT, exitFunction); // Auxiliar function for interruptions
	
	while((opt = getopt(argc, argv, "p:")) != -1) { // Get arguments for configure the serial port
		switch(opt){
			case 'p':
				serial_port_name = (char *)calloc(strlen(optarg) + 1, sizeof(char));
				if(!serial_port_name){
					fprintf(stderr, "Cannot allocate memory!\n");
				}
				strcpy(serial_port_name, optarg);
				break;
			case 'h':
				usage(argv[0]);
				exit(EXIT_FAILURE);
		}
	}

////////////////////////////
// Configure the serial port
	if(!serial_port_name){ // Check the serial port name variable is set
		fprintf(stderr, "Please supply the serial port path where the piksi is connected!\n");
		exit(EXIT_FAILURE);
	}

	result = sp_get_port_by_name(serial_port_name, &piksi_port);
	if(result != SP_OK){
		fprintf(stderr, "Cannot find provided serial port!\n");
		exit(EXIT_FAILURE);
	}

	result = sp_open(piksi_port, SP_MODE_READ);
	if(result != SP_OK){
		fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
		exit(EXIT_FAILURE);
	}

	setup_port();

	#if (DEBUG_MODE > 0)
		printf("Port succesfull configured..!\n");
	#endif
//
////////////////////////////
	

	sbp_setup(); // Configure Piksi GPS
	#if (DEBUG_MODE > 0)
		printf("Piksi succesfull configured..!\n");
	#endif

	printf("Configuration successfull..!\n");
	
//
/////////////////////////////////////////


/////////////////////////////////////////
//  Main  infinity loop

	while(true){	

		// Get GPS information 
		ret = sbp_process(&sbp_state, &piksi_port_read);
	
		if(ret != SBP_OK_CALLBACK_EXECUTED){
			#if (DEBUG_MODE > 0)
				printf("sbp_process error\n");
				printf("ret: %d\n",ret);
			#endif
		}
	
		if(ret == SBP_OK_CALLBACK_EXECUTED)
		{	
			time(&seconds);
			timestamp = localtime(&seconds);

			strftime(buffer3,sizeof(buffer3)-1,"%Y%m%d_%H%M%S",timestamp);
	
			lat_O_deg = pos_llh.lat; //-2.142992;
			lon_O_deg = pos_llh.lon; //-79.967774;

			fixMode = pos_llh.flags & 0x07;
		
			printf("Latitude %2.6f\n", lat_O_deg);
			printf("Longitude %2.6f\n",lon_O_deg);
			
			
			// Process the submarine GPS point

			distance = dist.relative_distance(Ta, Tb, Tc, depth);

			distance /= 1000;

			printf("Distance %2.6f\n",distance);
		
			bearing = 45;//comp.get_Comp_heading();

			theta = toRadians(bearing);
		
			gamma = distance / earth_R;
			
			lat_O_rad = toRadians(lat_O_deg); 
			lon_O_rad = toRadians(lon_O_deg);

			
			lat_D_rad = asin( ( sin(lat_O_rad) * cos(gamma) ) + ( cos(lat_O_rad) * sin(gamma) * cos(theta) ) );

			x = cos(gamma) - (sin(lat_O_rad) * sin(lat_D_rad));
			y = sin(theta) * sin(gamma) * cos(lat_O_rad);

			lon_D_rad = lon_O_rad + atan2( y, x );
			
			
			lat_D_deg = toDegrees(lat_D_rad);
			
			lon_D_deg= fmod(toDegrees(lon_D_rad) + 540, 360)-180;

			printf("Latitude dest: %2.6f\n", lat_D_deg);
			printf("Longitude dest: %2.6f\n",lon_D_deg);

			sprintf(buffer4,"%2.8f,%2.8f,%2.8f,%2.8f,%2.3f,%d",lat_O_deg, lon_O_deg, lat_D_deg, lon_D_deg, depth, fixMode);
			
			dataLogFile << buffer3 <<","<< buffer4 <<endl;
			
			if(flag){
				printf("End program.! \n");
				dataLogFile.close();
				eventlogFile.close();

				result = sp_close(piksi_port);
				if(result != SP_OK){
					fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
				}
	
				sp_free_port(piksi_port);
	
				free(serial_port_name);
				exit (NULL);

			}
		
			usleep(1000000);
		
		}
		if(flag){
				printf("End program.! \n");
				dataLogFile.close();
				eventlogFile.close();

				result = sp_close(piksi_port);
				if(result != SP_OK){
					fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
				}
	
				sp_free_port(piksi_port);
	
				free(serial_port_name);
				exit (NULL);

			}
	}
//
/////////////////////////////////////////


	return 0;
}
//
/////////////////////////////////////////////////////////////////////////////////
