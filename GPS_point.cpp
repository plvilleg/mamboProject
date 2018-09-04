//http://www.dirsig.org/docs/new/coordinates.html

// Standards libraries
#include <stdio.h>
#include <cstdlib>
#include <cstring>
#include <string>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <time.h>

// Special libraries 
#include <armadillo>
#include "COMPASS.h"
#include "RELATIVE_DISTANCE.h"
#include <libserialport.h>
extern "C" {
#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libsbp/navigation.h>
#include <libsbp/system.h>
}

/////////////////////////////////////////////////////////////////////////////////
// Definitions

#define _USE_MATH_DEFINES
#define ARMA_DONT_PRINT_ERRORS
#define earth_R 6378.137

//
/////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////
// Variables
using namespace std;
using namespace arma;

const float PI = (atan(1)*4); 

COMPASS comp;
RelativeDistace distance;

std::fstream archivo1;
struct timespec t1, t2;

// Serial variables
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

///////////////////////////////////////////
// Piksi variables

sbp_state_t sbp_state;	// State of the SBP parser.

// SBP structs that messages from Piksi will feed. 
msg_pos_llh_t	pos_llh;
msg_gps_time_t	gps_time;

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
	pos_llh = *(msg_pos_llh_t *)msg;
}


void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	gps_time = *(msg_gps_time_t *)msg;
}


void sbp_setup(void)
{
	int ret = -5;
	
	/* SBP parser state must be initialized before sbp_process is called*/
	sbp_state_init(&sbp_state);

	/* Register a node and callback, and associate them with a specific message ID*/
	
	ret = sbp_register_callback(&sbp_state, 0x201, &sbp_pos_llh_callback, NULL, &pos_llh_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x100, &sbp_gps_time_callback, NULL, &gps_time_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	printf("SBP_SETUP_OK\n\n");
}

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

// Setup PORT
void usage(char *prog_name){
	fprintf(stderr, "usage: %s [-p serial port]\n", prog_name);
}

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
/////////////////////////////////////////////////////////////////////////////////




double toRadians(double degrees){
	return degrees * PI/180;
}


double toDegrees(double radians){
	return radians * 180/PI;
}


int main(int argc, char **argv)
{
	int opt; // Option arg
	int result = 0;
	int ret=0;

	time_t seconds;
	struct tm *time_gps;

	double lat_O_deg, lon_O_deg, lat_O_rad, lon_O_rad, lat_D_rad, lon_D_rad, lat_D_deg, lon_D_deg, theta, gamma;
	double distance, bearing, x, y; 
	double Ta, Tb, Tc, depth;
	
	char buffer3[80];

	char *fileName = NULL;;
 
	std::string timeA, timeB, timeC; 
	std::string::size_type sz;

	if(argc <= 1) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

	timeA = argv[3];
	timeB = argv[4];
	timeC = argv[5];

	Ta = std::stod(timeA,&sz);
	Tb = std::stod(timeB,&sz);
	Tc = std::stod(timeC,&sz);

	while((opt = getopt(argc, argv, "p:")) != -1) {
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

	if(!serial_port_name){
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

	sbp_setup();

	// Create log file	
		
	//if( (archivo1 = fopen("test.txt", "w+") ) == NULL){
	//	printf("ERROR: Cannot open the file");
	//	exit(EXIT_FAILURE);		
	//}

	printf("OK3\n");
	//fprintf(archivo1,"time,latitude,longitude,depth\n");

	archivo1.open(argv[6], ios::app);

	depth = 5.0;

	
	//printf("%s", argv[6]);
	//strcpy(fileName,argv[6]);
	//fileName = *argv[6];


	
	// Set the position of the speakers
	setSpeaker_1(0,1); 
	setSpeaker_2(0.988,0.1542);
	setSpeaker_3(-0.988,-0.1542);

	while(true){	

		// Get GPS information 
		ret = sbp_process(&sbp_state, &piksi_port_read);
	
		if(ret < 0)
			printf("sbp_process error\n");
	
		if(ret == SBP_OK_CALLBACK_EXECUTED)
		{	
			seconds = (time_t)(gps_time.tow/1000);
			time_gps = localtime(&seconds);
	
			strftime(buffer3,sizeof(buffer3)-1,"%Y%m%d_%H%M%S",time_gps);
	
			lat_O_deg =  -2.142992;//pos_llh.lat;
			lon_O_deg =  -79.967774;//pos_llh.lon;
		
			printf("Latitude %2.6f\n", lat_O_deg);
			printf("Longitude %2.6f\n",lon_O_deg);
			printf("Distance %2.6f\n",distance);

			// lat_O_deg, lon_O_deg, lat_O_rad, lon_O_rad, lat_D_rad, lon_D_rad, lat_D_deg, lon_D_deg, theta, gamma


			// Process the submarine GPS point

			distance = relative_distance(Ta, Tb, Tc, depth);
		
			bearing = real_Bearing();

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
			
			//fprintf(archivo1,"%s,%4.10lf,%4.10lf,%2.4f\n",buffer3,lat_dest,lon_dest,depth);
			archivo1 << buffer3 <<","<< lat_D_deg<<"," << lon_D_deg<<"," << depth << endl;
		
		}
	}
		archivo1.close();

		result = sp_close(piksi_port);
		if(result != SP_OK){
			fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
		}
	
		sp_free_port(piksi_port);
	
		free(serial_port_name);

	
	return 0;
}