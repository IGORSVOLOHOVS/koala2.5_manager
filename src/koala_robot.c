//--------------------------------------------------------------------------------//
//-                   KOALA( Koala extension board			 )                       -//
//                                                                               -//
//-  Copyright (C) Julien Tharin, K-Team S.A. 2013                               -//
//-  This library is free software; you can redistribute it and/or               -//
//-  modify it under the terms of the GNU Lesser General Public                  -//
//-  License as published by the Free Software Foundation; either                -//
//-  version 2.1 of the License, or any later version.                           -//
//-                                                                              -//
//-  This library is distributed in the hope that it will be useful,             -//
//-  but WITHOUT ANY WARRANTY; without even the implied warranty of              -//
//-  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU           -//
//-  Lesser General Public License for more details.                             -//
//-                                                                              -//
//-  You should have received a copy of the GNU Lesser General Public            -//
//-  License along with this library; if not, write to the Free Software         -//
//-  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA   -//
//-                                                                              -//
//-                               __  __  ________                               -//
//- K-Team S.A.                  |  |/  /|__    __|___  _____  ___  ___          -//
//- Chemin des Plans-Praz 28,    |     / __ |  | _____|/  _  \|   \/   |         -//
//- 1337 Vallorbe                |  |  \    |  | ____|/  /_\  |        |         -//
//- Switzerland                  |__|\__\   |__|______|_/   \_|__|\/|__|         -//
//- jtharin@k-team.com   tel:+41 24 423 89 56 fax:+41 24 423 8960                -//
//-                                                                              -//
//--------------------------------------------------------------------------------//

////////////////////////////////////////////////////////////////////////////////
/*!   \file koala_robot.c
      \brief Functions for commanding the Koala robot
*/
////////////////////////////////////////////////////////////////////////////////



/* ---- Include Files ---------------------------------------------------- */
#include "koala.h"

#include <stdio.h>
#include <string.h>
#include <errno.h>

/* ---- Private Constants and Types -------------------------------------- */

koala_rs232_t *koala_rs232; // serial port device


/*---- private Functions --------------------------------------------------*/

/*! process integer parameters from a string into a integer array
 *
 * \param source array of char containing  input parameters sperated by DELIM
 * \param destination array of integer containing parameters
 * \param nb_arg number of parameters found
 *
 * \return A value:
 *       - <0 on error
 *       - 0 on success
 *
 */
int process_param(char *source,int *destination,int nb_arg)
{
  int narg=0;
  char *pch;
  
	pch = strtok (source,KOALA_DELIM);
	while (pch != NULL)
	{		
		if (sscanf(pch,"%d",&destination[narg])!=1)
		  return -1;
		
	  pch = strtok (NULL, KOALA_DELIM);
	  narg++;
	    
	  if (narg == nb_arg)
			break;
	}
	
	return 0;	
}	

/* ---- Exported Functions------------------------------------------------ */


const char  *KOALA_US_SENSOR_NAMES[]=   {"Left rear","Left front","Front left","Front","Front right","Right front","Right rear","Back right","Back left"};

/*! initializes.
 * This function needs to be called BEFORE any other functions.
 * But default it is already called inside koala_init!
 *
 * \param (none)
 *
 * \return A value:
 *       - <0 on error
 *       - 0 on success
 *
 */
int koala_robot_init( void )
{
  char version;
  int rc;

	// open serial port port for communicating with the robot
	if ((koala_rs232=koala_rs232_open(KOALA_SERIAL_PORT_NAME,KOALA_SERIAL_PORT_BAUDRATE))== NULL)
	{
		  //printf("\n  libkoala ERROR: could not open serial port!\n\n");
			return -1;
	}
			
	return 0;	
}


/*! release the robot.
 * This function needs to be called AFTER any other functions.
 *
 * \param (none)
 *
 * \return A value:
 *       - <0 on error
 *       - 0 on success
 *
 */
int koala_robot_release( void )
{
  koala_rs232_close(koala_rs232);
	return 0;
}

/*!
 *  gets a command frame from the robot.
 *
 *
 * Normally an end user don't want to use these function as they are
 * assumed as "low level functions".
 *
 * \param read_len is the size of the message received
 * \param command is a pointer to a buffer where the command frame will be stored in.
 *
 * \return an error code:
 *         - <0  standard error code. See errno.h
 *         - >=0 on success
 *
 */
int koala_getcommand(char *command,int read_len)
{
 int rc=0;

 rc=koala_rs232_read(koala_rs232, command ,read_len );

 return rc;
}

/*!
 *  gets a command line from the robot.
 *
 *
 * Normally an end user don't want to use these function as they are
 * assumed as "low level functions".
 *
 * \param command is a pointer to a buffer where the command frame will be stored in.
 *
 * \return an error code:
 *         - <0  standard error code. See errno.h
 *         - >=0 on success
 *
 */
int koala_getcommand_line(char *command)
{
 int rc;

 rc=koala_rs232_readLine(koala_rs232,command);
 
 return rc;
}



int koala_getcommand_line_nowait(char *command)

{
 int rc;

 rc=koala_rs232_readLine_nowait(koala_rs232,command);
 
 return rc;
}

/*!
 *  send a command frame from the robot.
 *
 *
 * Normally an end user don't want to use these function as they are
 * assumed as "low level functions".
 *
 * \param send_len is the size of the message received
 * \param command is a pointer to a buffer where the command frame to send
 *
 * \return an error code:
 *         - <0  standard error code. See errno.h
 *         - >=0 on success
 *
 */
int koala_sendcommand(char *command,int send_len)
{
 int rc=0;
 
 rc=koala_rs232_write(koala_rs232,command,send_len);

 return rc;
}

/*!
 *  Configure the auto monitoring mode. Enable a peripheral to return
 *  automatically without asking its value when refreshed.
 *
 * \param bit_config  binary OR bit mask configuration for auto monitoring mode (default: all OFF)
 *     see koala_robot.h for constant definition of each
 *		Bit 0:	US sensor
 *		Bit 1:	Motor Speed
 *		Bit 2:	Motor Position
 *		Bit 3:	Motor Current
 *		Bit 4:	Accelerometer value
 *		Bit 5:	Gyroscope value
 *		Bit 6:	GPS data
 *		Bit 7:	GPS NEMA Data (will return all GPS raw data)
  *		Bit 8:	Magnometer value
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_configure_auto_monitoring_mode(unsigned int bit_config)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"A,%d\r",bit_config);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'a')
	  return -1;
	  
	return 0;
}

/*!
 *  retrieves the current OS Firmware version/revision
 *
 * \param version version
 * \param revision revision
 *
 * \return -  >=0 OK
 *         - - <0 error
 */
int koala_get_firmware_version_revision(char *version,char *revision)
{
  int rc=0;
  char buffer[7];
  
  buffer[0]='B';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand(buffer,7);
  
  if (version == NULL)
  	return -1;
  
  if (revision == NULL)
  	return -2;	
  
  if (buffer[0]!='b')
  	return -3;
  
  if (rc!=7)
  	return -4;
  
  *version=buffer[2];
  
  *revision=atoi(buffer+4);

	return 0;
}

/*!
 *  Configure the auto monitoring mode. Enable a peripheral to return
 *  automatically without asking its value when refreshed.
 *
 * \param us_mask  binary OR bit mask configuration for use of Ultrasonic sensors (default: all active)
 *     see koala_robot.h for constant definition of each
 *		Bit 0:	Left rear
 *		Bit 1:	Left front
 *		Bit 2:	Front left
 *		Bit 3:	Front 
 *		Bit 4:	Front right
 *		Bit 5:	Right front
 *		Bit 6:	Right rear
 *		Bit 7:	Back right
 *		Bit 8:	Back left
 *
 * \param io_dir binary OR bit mask configuration for direction of the four IO (0..3).
 *    Each IO is configure with two following bits: (IO0 = bit0 & 1, IO1 = bit2&3,…).
 *    Default: 0 = all output
 *    00 = output
 *    01 = input
 *    10 = PWM servo (50Hz).
 *     see koala_robot.h for constant definition of each
 *
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_configure_us_io(int us_mask,unsigned char io_dir)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"C,%d,%d\r",us_mask,io_dir);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'c')
	  return -1;
	  
	return 0;
}


/*!
 *  Set the motor speed [pulses / KOALA_TIME_BTWN ms]. The PID controller will manage the speed in closed loop.
 *
 * \param left left speed
 * \param right right speed
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_motor_speed(int left, int right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"D,%d,%d\r",left,right);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'd')
	  return -1;
	  
	return 0;
}

/*!
 *  Read the motor actual speed [pulses / KOALA_TIME_BTWN ms]. 
 *
 * \param left left speed
 * \param right right speed
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_motor_speed(int *left, int *right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	buffer[0]='E';
	buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='e')
  	return -1;
  	
  if (sscanf(buffer,"%*c,%d,%d",left,right)!=2)
  	return -2;
	  
	return 0;
}

/*!
 *  Set the motor target position [pulses]. 
 *
 * \param left left target position
 * \param right right target position
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_motor_target_position(int left, int right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"F,%d,%d\r",left,right);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'f')
	  return -1;
	  
	return 0;
}

/*!
 *  Read the ultrasonic sensors 
 *
 * \param values_array sensors values array 
 				0:		Obstacle <25cm
				25-250:	Obstacle distance in cm
				1000:	No obstacle detected
				2000:	Sensor disable
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_us_sensors(int *values_array)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;
	int narg=0;
	char * pch;

	buffer[0]='G';
	buffer[1]='\r';

	koala_sendcommand(buffer,2);;
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='g')
  	return -1;
  
	if (process_param(buffer+2,values_array,KOALA_US_SENSORS_NUMBER)!=0)
	 return -2;
   
	return 0;
}


/*!

 *  Read the magnometer 
 *
 * \param values_array sensors values x,y,z	in [mGauss]
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_magnetometer(int *values_array)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;
	int narg=0;
	char * pch;

	buffer[0]='&';
	buffer[1]='\r';

	koala_sendcommand(buffer,2);;
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='@')
  	return -1;
  
	if (process_param(buffer+2,values_array,KOALA_MAGNE_VALUES_NUMBER)!=0)
	 return -2;
   
	return 0;
}

/*!
 *  Configure the PID controller value used for the speed control.
 *
 * \param kp P parameter
 * \param ki I parameter
 * \param kd D parameter
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_configure_pid(int kp, int ki,int kd)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"H,%d,%d,%d\r",kp,ki,kd);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'h')
	  return -1;
	  
	return 0;
}

/*!
 *  Set the position encoder value [pulses].
 *
 *  Reset the position encoder value of the motor. 
 *	If set during a position control move,
 *  the motors will be stopped to avoid an incorrect behaviour.
 *
 * \param left left encoder position
 * \param right right encoder position
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_position_encoders(int left, int right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"I,%d,%d\r",left,right);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'i')
	  return -1;
	  
	return 0;
}

/*!
 *  Configure the parameters used for the position control. 
 *
 * \param acc_inc     Increment of the speed added every “Acc_Div+1” control loop.
 * \param acc_div     Number of control loop before adding the Acc_Inc to the speed order
 * \param min_speed   Minimum speed order used during the position control
 * \param cst_speed Constant speed used during the position control after acceleration
 * \param pos_margin  Margin of the position control to detect when the robot reach its target.
 * \param max_current Maximum current for each motor. If above this value,
 *                    the controller will limit the motor command.
 *                    (unit is mA, 0 = disable (default)).
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_speed_profile(int acc_inc,int acc_div,int min_speed,int cst_speed,int pos_margin,int max_current)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"J,%d,%d,%d,%d,%d,%d\r",acc_inc,acc_div,min_speed,cst_speed,pos_margin,max_current);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'j')
	  return -1;
	  
	return 0;
}

/*!
 *  Set Motor Speed with acceleration ramp [pulses / KOALA_TIME_BTWN ms].
 *
 *  Set a speed order to reach with acceleration ramp.
 *  The parameters used for this mode are the same as the position control.
 *
 * \param left left speed
 * \param right right speed
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_motor_speed_accel(int left, int right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"K,%d,%d\r",left,right);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'k')
	  return -1;
	  
	return 0;
}

/*!
 *  Set speed in open loop control.
 *
 *  Set a PWM value for each motor. Value can be from -2000 to +2000 (corresponding to -100 to 100% of PWM).
 *
 * \param left left pwm
 * \param right right pwm
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_motor_speed_open_loop(int left, int right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"L,%d,%d\r",left,right);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 'l')
	  return -1;
	  
	return 0;
}

/*!
 *  Get Accelerometer value: read the last 10 value of the XYZ acceleration.
 *
 * \param values_array  in order x0,y0,z0,x1,y1,z1,...,x9,y9,z9; new values first
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_accelerometer(int *values_array)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0,i;

	buffer[0]='M';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='m')
  	return -1;
	
	if (process_param(buffer+2,values_array,KOALA_ACCEL_VALUES_NUMBER)!=0)
	  return -2;

	  
	return 0;
}

/*!
 *  Get Gyroscope value: read the last 10 value of the XYZ.
 *
 * \param values_array  in order x0,y0,z0,x1,y1,z1,...,x9,y9,z9; new values first
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_gyroscope(int *values_array)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0,i;

	buffer[0]='N';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='n')
  	return -1;
  		
  if (process_param(buffer+2,values_array,KOALA_GYRO_VALUES_NUMBER)!=0)
	  return -2;  
	  
	return 0;
}

/*!
 *  Read the motor current 
 *
 * \param left  current of left motor [0.1 A]
 * \param right current of right motor [0.1 A]
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_motor_current(int *left, int *right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	buffer[0]='O';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='o')
  	return -1;
  	
  if (sscanf(buffer,"%*c,%d,%d",left,right)!=2)
  	return -2;
	  
	return 0;
}

/*!
 *  Read the motor position [pulse]
 *
 * \param left  encoder value of left motor
 * \param right encoder value of right motor
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_motor_position(int *left, int *right)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	buffer[0]='P';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='p')
  	return -1;
  	
  if (sscanf(buffer,"%*c,%d,%d",left,right)!=2)
  	return -2;
	  
	return 0;
}


/*!
 *  Get GPS data
 *
 * \param valid_sat Valid data flag (V = Warning, A = Valid)
 * \param sat_nb number of satellites used
 * \param lat_val latitude
 * \param lat_car latitude N or S
 * \param long_val longitude
 * \param long_car longitude W or E
 * \param date_time UTC time and time of the latest fix
 * \param speed Speed over ground (in knots)
 * \param altitude Actual altitude (in meters)
 *
 * \return -  >=0 OK

 *         -  <0 error
 */
int koala_gps_data(char *valid_sat,int *sat_nb,double *lat_val,char *lat_car,double *long_val,char *long_car,struct tm *date_time,double *speed,int *altitude)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0,i;
	char *bptr;
	int narg;

	buffer[0]='Q';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='q')
  	return -1; 
  
  if (sscanf(buffer,"%*c,%c,%d,%lf,%c,%lf,%c,%d:%d:%d,%d.%d.%d,%lf,%d",valid_sat,sat_nb,lat_val,lat_car,long_val,long_car,&date_time->tm_hour,&date_time->tm_min,&date_time->tm_sec,&date_time->tm_mday,&date_time->tm_mon,&date_time->tm_year,speed,altitude)!=14)
  	return -2;

	  
	return 0;
}


/*!
 *  Read I2C external bus
 *
 * \param i2c_add i2c address
 * \param i2c_reg i2c register
 * \param nb_read number of bytes to read (max 128)
 * \param data    bytes read
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_i2c(int i2c_add,int i2c_reg, int nb_read,int *data)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;
	int narg=0;
	char *pch;

  sprintf(buffer,"R,%d,%d,%d\r",i2c_add,i2c_reg,nb_read);
	koala_sendcommand(buffer,strlen(buffer));
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='r')
  	return -1; 
  
		
	pch = strtok (buffer+2,KOALA_DELIM);
	while (pch != NULL)
	{
		
		if (sscanf(pch,"%d",&data[narg])!=1)
		  return -2;
		
	  pch = strtok (NULL, KOALA_DELIM);
	  narg++;
	    
	  if (narg == KOALA_MAX_I2C_DATA)
			break;
			
		if (narg==nb_read)
		 break;	
	}
	
	if (narg!=nb_read)
		return -3;
	  
	return 0;
}


/*!
 *  Set the PWR and IO output value 
 *  Set the state of the four PWR output
 *  and four IO (if set as output,  0 = GND, 1 = +3.3V, if set as PWM servo, 
 *  define the position  of the servo for 0 to 250 ).

 * \param power_out power output binary OR bit mask configuration
 *                  see koala_robot.h for constant definition of each bit
 * \param IO0 IO 0
 * \param IO1 IO 1
 * \param IO2 IO 2
 * \param IO3 IO 4

 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_set_pwr_io_output(int power_out,int IO0, int IO1, int IO2, int IO3)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	sprintf(buffer,"S,%d,%d,%d,%d,%d\r",power_out,IO0,IO1,IO2,IO3);
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer); 
  
  if (buffer[0] != 's')
	  return -1;
	  
	return 0;
}

/*!
 *  Read IO state
 *  Read the four IO and the two Input state. If the IO are define as output,
 *  will be the same value as set with S command, if define as PWM servo output,
 *  will be return as 0.
 *
 * \param io_state 4 IO  in binary OR bit configuration (lsb=0, msb = 3)
 * \param in_state 2 digital input 

 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_io(int *io_state, int *in_state)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	buffer[0]='T';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='t')
  	return -1;
  	
  if (sscanf(buffer,"%*c,%d,%d",io_state,in_state)!=2)
  	return -2;
	  
	return 0;
}

/*!
 *  Read the two AD input values (0-1024  => 0 - 3.3V). 
 *
 * \param ad_0 analog input port 0
 * \param ad_1 analog input port 1
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_read_ad(int *ad_0, int *ad_1)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;

	buffer[0]='U';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='u')
  	return -1;
  	
  if (sscanf(buffer,"%*c,%d,%d",ad_0,ad_1)!=2)
  	return -2;
	  
	return 0;
}

/*!
 *  Read the different values of the battery
 *
 * \param bat_type     0 = Li-ION, 1 = NiMH, 2 = not initialised
 * \param bat_voltage	 Voltage of the battery (unit is 0.1V)
 * \param bat_current	 Current of the battery (unit is 0.1A)
 * \param chrg_current Charge Current (unit is 10mA)
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_get_battery_data(int *bat_type,int *bat_voltage, int *bat_current, int *chrg_current)
{

  char buffer[KOALA_MAX_BUFFER];
	int rc=0;
	
	buffer[0]='V';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='v')
  	return -1;
  		
  if (sscanf(buffer,"%*c,%d,%d,%d,%d",bat_type,bat_voltage,bat_current,chrg_current)!=4)
  	return -2;

	return 0;
}

/*!
 *  Write on the I2C external bus	
 *
 * \param i2c_add i2c address
 * \param i2c_reg i2c register
  * \param nb_data number of data to send
 * \param data  bytes to be written
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_write_i2c(int i2c_add,int i2c_reg,int nb_data,int *data)
{
	char buffer[KOALA_MAX_BUFFER];
	char cdata[8];
	int rc=0,i;
	
	// test buffer space
	if (nb_data>((KOALA_MAX_BUFFER-10)/4))
		return -1;
	
	sprintf(buffer,"W,%d,%d",i2c_add,i2c_reg);
	
	for (i=0;i<nb_data;i++)
	{
		sprintf(cdata,",%d",data[i]);
	  strcat(buffer,cdata);
	}
	
	strcat(buffer,"\r");
	
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer);
  
	if (buffer[0]!='w')
  	return -2; 
 
	  
	return 0;
}

/*!
 *  Send GPS command 	
 *
 * \param gps_cmd gps command
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_send_gps_cmd(char *gps_cmd)
{
  char buffer[KOALA_MAX_BUFFER];
  
	if (strlen(gps_cmd)>KOALA_MAX_BUFFER-4)
	  return -1;
	
	sprintf(buffer,"X,%s\r",gps_cmd);
		
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer);
  
	if (buffer[0]!='x')
  	return -2; 
 	  
	return 0;
}

/*!
 *  Get Motor control status
 *
 *  Read the actual status of the motor control
 *
 * \param left_status  left motor control status
 * 											 Type of actual control:
 *											0:	Idle
 * 											1:	Speed
 * 											2:	Speed with Acceleration
 * 											3:	Position
 *											4:	Open Loop
 * 											5:	Current Limitation
 * 											6:	Error
 * \param right_status right motor control status
 * \param left_pos	   left motor: 1 if the target is reach (position control)
 * \param right_pos	   right motor: 1 if the target is reach (position control)
 *
 * \return -  >=0 OK
 *         -  <0 error
 *
 */
int koala_get_motor_status(int *left_status, int *right_status,int *left_pos, int *right_pos)
{

  char buffer[KOALA_MAX_BUFFER];
	int rc=0;
	
	buffer[0]='Y';
  buffer[1]='\r';

	koala_sendcommand(buffer,2);
  rc=koala_getcommand_line(buffer); 
  
	if (buffer[0]!='y')
  	return -1;
  		
  if (sscanf(buffer,"%*c,%d,%d,%d,%d",left_status,right_status,left_pos,right_pos)!=4)
  	return -2;

	return 0;
}

/*!
 *  Reset microcontroller	
 *
 *
 * \return -  >=0 OK
 *         -  <0 error
 */
int koala_reset_microcontroller()
{
  char buffer[KOALA_MAX_BUFFER];
  	
	sprintf(buffer,"Z\r");
		
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer);
  
	if (buffer[0]!='z')
  	return -1; 
 	  
	return 0;
}

/*!
 *  Scan I2C external bus	
 *
 * \param nb_devices  number of devices found
 * \param address     array of address found 
 * \return -  >=0 OK
 *         -  <0 error
 */
#define MAX_I2C_ADDRESS 128
int koala_scan_i2c(int *nb_devices,int *address)
{
  char buffer[KOALA_MAX_BUFFER];
  int narg=0,nb;
	char delim[2]= {',' , '\0'};
	char * pch;
  
	*nb_devices=0;
	
	sprintf(buffer,"?\r");
		
	koala_sendcommand(buffer,strlen(buffer)); 
  
  koala_getcommand_line(buffer);
  
	if (buffer[0]!='!')
  	return -1; 
 	
		
	pch = strtok (buffer+2,delim);
	
	if (pch == NULL)
		  return -2;
	
	if (sscanf(pch,"%d",nb_devices)!=1)
		  return -3;
		  
	pch = strtok (NULL, delim);	  
	
	while (pch != NULL)
	{
		
		if (sscanf(pch,"%d",&address[narg])!=1)
		  return -4;
		
	  pch = strtok (NULL, delim);
	  narg++;
	    
	  if (narg == MAX_I2C_ADDRESS)
			break;
	}
	
	if (narg != *nb_devices)
	  return -5;
	   	  
	return 0;
}


/*!
 *  Get values from auto mode
 *  
 * \param data data received (see koala_robot.h for type definition)
 *
 * \return -  >=0 OK
 *         -  <0 error
 *
 */
int koala_get_from_auto_mode(koala_auto_data_t *data)
{
	char buffer[KOALA_MAX_BUFFER];
	int rc=0;
	char *pch;
	int narg;
	
  rc=koala_getcommand_line_nowait(buffer); 
  
  if (rc<=0)
  {
  	return -1;
	}  
  
  data->mode=buffer[0];
  
  switch(buffer[0])
  {
  	case 'e' : // Motor Speed
  	   if (sscanf(buffer,"%*c,%d,%d",&data->left_speed,&data->right_speed)!=2)
  	     return -2;
  	break;
  	case 'g' : // US sensor			
			 if (process_param(buffer+2,data->us,KOALA_US_SENSORS_NUMBER)!=0)
	  		return -3; 
  	break;
  	case 'm' : // Accelerometer value
  		if (process_param(buffer+2,data->accel,KOALA_ACCEL_VALUES_NUMBER)!=0)
	  		return -4; 
  	break;
  	case 'n' : // Gyroscope value
  	  if (process_param(buffer+2,data->gyro,KOALA_GYRO_VALUES_NUMBER)!=0)
	  		return -5; 
  	break;
  	case 'o' : // Motor Current
  	   if (sscanf(buffer,"%*c,%d,%d",&data->left_current,&data->right_current)!=2)
  	     return -6;
  	break;
  	case 'p' : // Motor Position
  	   if (sscanf(buffer,"%*c,%d,%d",&data->left_position,&data->right_position)!=2)
  	     return -7;
  	break;
  	case 'q' : // GPS data
  	  if (sscanf(buffer,"%*c,%c,%d,%lf,%c,%lf,%c,%d:%d:%d,%d.%d.%d,%lf,%d",&(data->gps.valid_sat),&(data->gps.sat_nb),&(data->gps.lat_val),&(data->gps.lat_car),&(data->gps.long_val),&(data->gps.long_car),&(data->gps.date_time.tm_hour),&(data->gps.date_time.tm_min),&(data->gps.date_time.tm_sec),&(data->gps.date_time.tm_mday),&(data->gps.date_time.tm_mon),&(data->gps.date_time.tm_year),&(data->gps.speed),&(data->gps.altitude))!=14)
  		return -8;
  	break;
  	case '$' : // GPS raw data
  	  strcpy(data->gps_raw,buffer);
  	break;
  	case '@' : // GPS raw data
  	  if (process_param(buffer+2,data->magne,KOALA_MAGNE_VALUES_NUMBER)!=0)
	  		return -9; 
  	break;
    default:
      return -10;
    
  }
  	
	return 0;
}
