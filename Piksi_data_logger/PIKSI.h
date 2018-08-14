#ifndef PIKSI_h
#define PIKSI_h

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/piksi.h>
#include <libsbp/navigation.h>
#include <libsbp/system.h>



class PIKSI
{
	// Log variables
	char buffer1[80];
	char buffer2[80];
	char buffer3[80];
	
	FILE *archivo1, *archivo2, *archivo3;
	struct timespec t1, t2;
	double tiempo;
	int flag_f = 1;
	
	// Serial variables
	char *serial_port_name = NULL;
	struct sp_port *piksi_port = NULL;
	
	
	// Setup PIKSI
	// Piksi variables
	
	sbp_state_t sbp_state;	// State of the SBP parser.
	
	
	/* SBP structs that messages from Piksi will feed. */
	msg_pos_llh_t	pos_llh;
	msg_baseline_ned_t	baseline_ned;
	msg_vel_ned_t	vel_ned;
	msg_dops_t	dops;
	msg_gps_time_t	gps_time;
	msg_utc_time_t utc_time;
	msg_iar_state_t iar_state;
	msg_pos_ecef_t pos_ecef;
	msg_uart_state_t uart_state;
	
	/* SBP callback node must be statically allocated. Each message ID / callback pair
	must have a unique sbp_msg_callbacks_node_t associated with it*/
	
	static sbp_msg_callbacks_node_t pos_llh_node;
	static sbp_msg_callbacks_node_t baseline_ned_node;
	static sbp_msg_callbacks_node_t vel_ned_node;
	static sbp_msg_callbacks_node_t dops_node;
	static sbp_msg_callbacks_node_t gps_time_node;
	static sbp_msg_callbacks_node_t utc_time_node;
	static sbp_msg_callbacks_node_t iar_state_node;
	static sbp_msg_callbacks_node_t pos_ecef_node;
	static sbp_msg_callbacks_node_t uart_state_node;

//public:




private:
	void sbp_utc_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_baseline_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_iar_state_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_uart_state_callback(u16 sender_id, u8 len, u8 msg[], void *context);
	void sbp_setup(void);
	void usage(char *prog_name);
	void setup_port();
	u32 piksi_port_read(u8 *buff, u32 n, void *context);
	void setup_files(struct tm * time_gps);
	


};

#endif