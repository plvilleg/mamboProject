#include "PIKSI.h"




/* Callback funtions to interpret SBP messages.
*  Every message ID has a callback associated with it to
*  receive and interpret the message payload
*/
void PIKSI::sbp_utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	utc_time = *(msg_utc_time_t *)msg;
}

void PIKSI::sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	pos_llh = *(msg_pos_llh_t *)msg;
}	

void PIKSI::sbp_baseline_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	baseline_ned = *(msg_baseline_ned_t *)msg;
}

void PIKSI::sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	vel_ned = *(msg_vel_ned_t *)msg;
}

void PIKSI::sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	dops = *(msg_dops_t *)msg;
}

void PIKSI::sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	gps_time = *(msg_gps_time_t *)msg;
}

void PIKSI::sbp_iar_state_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	iar_state = *(msg_iar_state_t *)msg;
}

void PIKSI::sbp_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	pos_ecef = *(msg_pos_ecef_t *)msg;
}

void PIKSI::sbp_uart_state_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
	uart_state = *(msg_uart_state_t *)msg;
}

void PIKSI::sbp_setup(void)
{
	int ret = -5;
	
	/* SBP parser state must be initialized before sbp_process is called*/
	sbp_state_init(&sbp_state);

	/* Register a node and callback, and associate them with a specific message ID*/

	ret = sbp_register_callback(&sbp_state, SBP_MSG_UTC_TIME, &sbp_utc_time_callback, NULL, &utc_time_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x100, &sbp_gps_time_callback, NULL, &gps_time_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}
	
	ret = sbp_register_callback(&sbp_state, 0x201, &sbp_pos_llh_callback, NULL, &pos_llh_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x203, &sbp_baseline_callback, NULL, &baseline_ned_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x205, &sbp_vel_ned_callback, NULL, &vel_ned_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x206, &sbp_dops_callback, NULL, &dops_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x0019, &sbp_iar_state_callback, NULL, &iar_state_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x0200, &sbp_pos_ecef_callback, NULL, &pos_ecef_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	ret = sbp_register_callback(&sbp_state, 0x0018, &sbp_uart_state_callback, NULL,&uart_state_node);
	if(ret != SBP_OK){
		printf("SBP_CALLBACK_ERROR");
		exit(EXIT_FAILURE);
	}

	printf("SBP_SETUP_OK\n\n");
}


// Setup PORT
void PIKSI::usage(char *prog_name){
	fprintf(stderr, "usage: %s [-p serial port]\n", prog_name);
}

void PIKSI::setup_port()
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


u32 PIKSI::piksi_port_read(u8 *buff, u32 n, void *context)
{
	(void)context;
	int i=0;
	u32 result;
	result = sp_blocking_read(piksi_port, buff, n, 0);

	return result;
}


// Setup file headers and names
void PIKSI::setup_files(struct tm * time_gps)
{	
	printf("Creating FILES\n\n");
	strftime(buffer1, sizeof(buffer1)-1, "baseline_log_%Y%m%d-%H%H%S.txt", time_gps);
	strftime(buffer2, sizeof(buffer2)-1, "position_log_%Y%m%d-%H%H%S.txt", time_gps);
	strftime(buffer3, sizeof(buffer3)-1, "velocity_log_%Y%m%d-%H%H%S.txt", time_gps);

	if( (archivo1 = fopen(buffer1, "w+") ) == NULL)
	{
		printf("ERROR: Cannot open the file");
		exit(EXIT_FAILURE);
	}

	if( (archivo2 = fopen(buffer2, "w+") ) == NULL)
	{
		printf("ERROR: Cannot open the file");
		exit(EXIT_FAILURE);
	}

	if( (archivo3 = fopen(buffer3, "w+") ) == NULL)
	{
		printf("ERROR: Cannot open the file");
		exit(EXIT_FAILURE);
	}

	fprintf(archivo1,"time,distance(meters),num_sats,flags\n");
	fprintf(archivo2,"time,latitude(degrees),longitudes(degrees),altitude(meters),n_sats,flags\n");
	fprintf(archivo3,"time,speed(m/s),num_sats\n");

}


int main(int argc , char **argv)
{
	int opt;
	int result = 0;
	
	time_t seconds;
	struct tm * time_gps;
	
	double dist2, dist;

	
	if(argc <= 1) {
		usage(argv[0]);
		exit(EXIT_FAILURE);
	}

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

	//printf("Starting..!\n\n");
	//while(1){
		int ret = sbp_process(&sbp_state, &piksi_port_read);

		if(ret < 0)
			printf("sbp_process error\n");

		if(ret == SBP_OK_CALLBACK_EXECUTED)
		{

			seconds = (time_t)(gps_time.tow/1000);
			time_gps = localtime(&seconds);

			if(flag_f == 1){
				setup_files(time_gps);
				flag_f = 0;
				printf("Started!\n\n");
			}

			strftime(buffer3,sizeof(buffer3)-1,"%Y%m%d_%H%M%S",time_gps);


		

			/* Print absolute position. */
			/*printf("Absolute Position:\n ");
			printf("\tLatitude\t: %4.10lf\n", pos_llh.lat);
			printf("\tLongitude\t: %4.10lf\n", pos_llh.lon);
			printf("\tHeight\t\t: %4.10lf\n", pos_llh.height);
			printf("\tSatellites\t:  %02d\n", pos_llh.n_sats);
			printf("\tState\t: %d\n",pos_llh.flags);
			printf("\n");*/


			dist2 =	pow(((double)baseline_ned.n)/1000, 2) + pow(((double)baseline_ned.e)/1000, 2) + pow(((double)baseline_ned.d)/1000, 2);
			dist = sqrt(dist2);

			fprintf(archivo1,"%s,%.2f,%2d,%2d\n",buffer3,dist,baseline_ned.n_sats,baseline_ned.flags);
			fprintf(archivo2,"%s,%4.10lf,%4.10lf,%4.10lf,%2d,%2d\n",buffer3,pos_llh.lat,pos_llh.lon,pos_llh.height,pos_llh.n_sats,pos_llh.flags);
			fprintf(archivo3,"%s,%6d,%2d\n",buffer3,vel_ned.d,vel_ned.n_sats);

		}



//	}

	result = sp_close(piksi_port);
	if(result != SP_OK){
		fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
	}

	sp_free_port(piksi_port);

	free(serial_port_name);

	return 0;
}
