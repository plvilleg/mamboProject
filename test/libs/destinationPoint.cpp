#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <cmath>
#include <unistd.h>
#include <fstream>
#include <time.h>
#include "COMPASS.h"
#include <libserialport.h>
#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libsbp/navigation.h>
#include <libsbp/system.h>
#include <armadillo>

#define _USE_MATH_DEFINES
#define ARMA_DONT_PRINT_ERRORS


#define W_speed 1496.73f // T = 25º , V = 1.402385(10^3)+5.038813(T)−5.799136(10^−2)(T^ 2)+3.287156(10^−4)(T^3)−1.398845 (10^−6) (T^4) +2.787860(10^−9)(T^5)
//http://www.dirsig.org/docs/new/coordinates.html
#define earth_R 6378.137

const float PI = (atan(1)*4); 

using namespace std;
using namespace arma;

COMPASS comp;
mat D(2,1);
float Pos_x1, Pos_y1, Pos_x2, Pos_y2, Pos_x3, Pos_y3;

FILE *archivo1;

struct timespec t1, t2;

// Serial variables
char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;

// Setup PIKSI
// Piksi variables

sbp_state_t sbp_state;	// State of the SBP parser.

/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_t	pos_llh;
msg_gps_time_t	gps_time;

/* SBP callback node must be statically allocated. Each message ID / callback pair
must have a unique sbp_msg_callbacks_node_t associated with it*/

static sbp_msg_callbacks_node_t pos_llh_node;
static sbp_msg_callbacks_node_t gps_time_node;

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


u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
	(void)context;
	u32 result;
	result = sp_blocking_read(piksi_port, buff, n, 0);
	return result;
}


void init(void){
	D.ones();
}

mat leastSqueare(float x1,float y1,float x2,float y2,float x3,float y3,float a,float b,float c)
{
	mat A(2,2); 
	mat B(2,1);
	mat R(2,1);

	A.ones();
	B.ones();

	A 	<< x3 - x1 << y3 - y1 << endr
		<< x3 - x2 << y3 - y2 << endr;

	A = A * 2;	

	B 	<< (pow(a,2.0) - pow(c,3.0)) - (pow(x1,2.0) - pow(x3,2.0)) - (pow(y1,2.0) - pow(y3,2.0)) <<endr
		<< (pow(b,2.0) - pow(c,2.0)) - (pow(x2,2.0) - pow(x3,2.0)) - (pow(y2,2.0) - pow(y3,2.0)) << endr;

	R = inv(A.t()*A)*A.t()*B;

	return R;
}

float relative_distance(float Ta, float Tb, float Tc, float depth){
	init();
	float dist_a = 0.0, dist_b = 0.0, dist_c = 0.0, rel_distA = 0.0, rel_distB = 0.0, rel_distC = 0.0, XAxis, YAxis;

	printf("OK3 \n");

	dist_a = W_speed * Ta;
	dist_b = W_speed * Tb;
	dist_c = W_speed * Tc;

	rel_distA = sqrt(pow(dist_a,2) + pow(depth,2));
	rel_distB = sqrt(pow(dist_b,2) + pow(depth,2));
	rel_distC = sqrt(pow(dist_c,2) + pow(depth,2));

	printf("OK4 \n");
	
	D = leastSqueare(Pos_x1, Pos_y1, Pos_x2, Pos_y2, Pos_x3, Pos_y3, rel_distA, rel_distB, rel_distC);

	XAxis = D(0,0);
	YAxis = D(1,0);

	return sqrt(pow(XAxis,2) + pow(YAxis,2));
}


float real_Bearing(){

	float bearing;
	float XAxis_o, YAxis_o, XAxis_r, YAxis_r, phi, gamma;
	comp.init();
	bearing = comp.get_Bearing();

	XAxis_o = D(0,0);
	YAxis_o =D(1,0);

	phi = bearing - 90;

	XAxis_r = XAxis_o*cos(phi) - YAxis_o*sin(phi);
	YAxis_r = XAxis_o*sin(phi) + YAxis_o*cos(phi);

	gamma = atan2(YAxis_r,XAxis_r) * (180.0/PI);	

	if(XAxis_r > 0 && YAxis_r > 0)
		bearing = 90 - gamma;
	if(XAxis_r < 0 && YAxis_r > 0)
		bearing = 180 - gamma;
	if(XAxis_r < 0 && YAxis_r < 0)
		bearing = 180 + gamma;
	if(XAxis_r > 0 && YAxis_r < 0)
		bearing = 360 - gamma;

	return bearing;
}


void setSpeaker_1(float x1, float y1){
	Pos_x1 = x1;
	Pos_y1 = y1;
}
	

void setSpeaker_2(float x2, float y2){
	Pos_x2 = x2;
	Pos_y2 = y2;
}


void setSpeaker_3(float x3, float y3){
	Pos_x3 = x3;
	Pos_y3 = y3;
}


int main(int argc, char **argv)
{
	int opt; // Option arg
	int result = 0;

	time_t seconds;
	struct tm * time_gps;

	double lat_origin, lon_origin, lat_dest, lon_dest, gamma;
	double distance, bearing; 
	double Ta, Tb, Tc, depth;
	
	char buffer3[80];

	char *fileName;
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

	depth = 1.0;

	strcpy(fileName,argv[6]);
	//fileName = argv[6];

	// Set the position of the speakers
	setSpeaker_1(0,1); 
	setSpeaker_2(0.988,0.1542);
	setSpeaker_3(-0.988,-0.1542);

	// Get GPS information 
	int ret = sbp_process(&sbp_state, &piksi_port_read);

	if(ret < 0)
		printf("sbp_process error\n");

	if(ret == SBP_OK_CALLBACK_EXECUTED)
	{	
		seconds = (time_t)(gps_time.tow/1000);
		time_gps = localtime(&seconds);

		strftime(buffer3,sizeof(buffer3)-1,"%Y%m%d_%H%M%S",time_gps);

		lat_origin =  pos_llh.lat;
		lon_origin =  pos_llh.lon;
	
		// Process the submarine GPS point
		distance = relative_distance(Ta, Tb, Tc, depth);
	
		bearing = real_Bearing();
	
		gamma = distance / earth_R;
	
		lat_dest = asin( (sin(lat_origin)*cos(gamma)) + (cos(lat_origin) * sin(gamma) * cos(bearing)) );
		lon_dest = lon_origin + atan2( sin(bearing) * sin(gamma) * cos(lat_origin), cos(gamma) - (sin(lat_origin) * sin(lat_dest)) );
	
		// Create log file	
		if( (archivo1 = fopen(fileName,"r+") ) != NULL)
		{
			fprintf(archivo1,"%s,%4.10lf,%4.10lf,%2.4f\n",buffer3,lat_dest,lon_dest,depth);
	
		}else if( (archivo1 = fopen(fileName, "w+") ) != NULL){
	
			fprintf(archivo1,"time,latitude,longitude,depth\n");
			fprintf(archivo1,"%s,%4.10lf,%4.10lf,%2.4f\n",buffer3,lat_dest,lon_dest,depth);
	
	
		}else{
			printf("ERROR: Cannot open the file");
			exit(EXIT_FAILURE);		
		}
	
		result = sp_close(piksi_port);
		if(result != SP_OK){
			fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
		}
	
		sp_free_port(piksi_port);
	
		free(serial_port_name);

	}
	return 0;
}
