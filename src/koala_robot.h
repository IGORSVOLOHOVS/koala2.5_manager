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
/*!   \file koala_robot.h
      \brief header of Funtions for commanding the Koala
*/
////////////////////////////////////////////////////////////////////////////////


#ifndef __koala_robot__
#define __koala_robot__

#include <time.h>
#include <math.h>

/*!
 *  Koala Constants
 */

#define KOALA_SERIAL_PORT_NAME "/dev/ttyS1"
#define KOALA_SERIAL_PORT_BAUDRATE B115200

#define KOALA_MAX_BUFFER 1024 // max input/output buffer for sending/receiving data

#define KOALA_DELIM ","  // delimiter for parameters

// bit configuration for auto monitoring mode
#define KOALA_AUTOM_US_SENSOR_BIT  1<<0	// US sensor
#define KOALA_AUTOM_MOTOR_SPEED    1<<1	// Motor Speed
#define KOALA_AUTOM_MOTOR_POSITION 1<<2	// Motor Position
#define KOALA_AUTOM_MOTOR_CURRENT  1<<3	// Motor Current
#define KOALA_AUTOM_ACCEL_VALUE    1<<4	// Accelerometer value
#define KOALA_AUTOM_GYRO_VALUE     1<<5	// Gyroscope value
#define KOALA_AUTOM_GPS_DATA       1<<6	// GPS data
#define KOALA_AUTOM_GPS_NEMA       1<<7	// GPS NEMA Data (will return all GPS raw data)
#define KOALA_AUTOM_MAGNE_VALUE    1<<8	// Magnometer value
#define KOALA_AUTOM_ALL            0x01FF  // all options activated
#define KOALA_AUTOM_NONE           0x0000  // all options desactivated


// bit configuration for US sensors mask
#define KOALA_US_LEFT_REAR	 1<<0  // Left rear
#define KOALA_US_LEFT_FRONT	 1<<1  // Left front
#define KOALA_US_FRONT_LEFT	 1<<2  // Front left
#define KOALA_US_FRONT       1<<3  // Front
#define KOALA_US_FRONT_RIGHT 1<<4  // Front left 
#define KOALA_US_RIGHT_FRONT 1<<5  // Right front
#define KOALA_US_RIGHT_REAR  1<<6  // Right rear
#define KOALA_US_BACK_RIGHT  1<<7  // Back right
#define KOALA_US_BACK_LEFT   1<<8  // Back left
#define KOALA_US_ALL         511   // all US activated
#define KOALA_US_NONE        0     // all US desactivated

extern const char  *KOALA_US_SENSOR_NAMES[];


#define KOALA_US_SENSORS_NUMBER 9 // number of US sensors

// bit configuration for Input/Output ports mask
#define KOALA_IO_0_OUTPUT   0b00000000  // Port 0 in output
#define KOALA_IO_0_INPUT    0b00000001  // Port 0 in input
#define KOALA_IO_0_PWMS     0b00000010  // Port 0 in PWM servo
#define KOALA_IO_1_OUTPUT   0b00000000  // Port 1 in output
#define KOALA_IO_1_INPUT    0b00000100  // Port 1 in input
#define KOALA_IO_1_PWMS     0b00001000  // Port 1 in PWM servo
#define KOALA_IO_2_OUTPUT   0b00000000  // Port 2 in output
#define KOALA_IO_2_INPUT    0b00010000  // Port 2 in input
#define KOALA_IO_2_PWMS     0b00100000  // Port 2 in PWM servo
#define KOALA_IO_3_OUTPUT   0b00000000  // Port 2 in output
#define KOALA_IO_3_INPUT    0b01000000  // Port 2 in input
#define KOALA_IO_3_PWMS     0b10000000  // Port 2 in PWM servo
#define KOALA_IO_ALL_OUTPUT 0b00000000  // All IO ports in output


// bit configuration for POWER Input/Output ports mask
#define KOALA_PWR_IO_0_0    0b00000000  // Power Port 0 to 0
#define KOALA_PWR_IO_0_1    0b00000001  // Power Port 0 to 1
#define KOALA_PWR_IO_1_0    0b00000000  // Power Port 1 to 0
#define KOALA_PWR_IO_1_1    0b00000010  // Power Port 1 to 1
#define KOALA_PWR_IO_2_0    0b00000000  // Power Port 2 to 0
#define KOALA_PWR_IO_2_1    0b00000100  // Power Port 2 to 1
#define KOALA_PWR_IO_3_0    0b00000000  // Power Port 3 to 0
#define KOALA_PWR_IO_3_1    0b00001000  // Power Port 3 to 1



#define KOALA_ACCEL_G (1.0/16384.0)   // convert to [g]
#define KOALA_ACCEL_VALUES_NUMBER 30 // number of values from the accelerometer: 3 axes * 10 values

#define KOALA_NEW_ACCEL_X  0   // position of newest X acceleration in the buffer
#define KOALA_NEW_ACCEL_Y  1  // position of newest Y acceleration in the buffer
#define KOALA_NEW_ACCEL_Z  2  // position of newest Z acceleration in the buffer

#define KOALA_GYRO_DEG_S (66.0/1000.0) // convert to [deg/s]
#define KOALA_GYRO_VALUES_NUMBER 30  // number of values from the gyrometer: 3 axes * 10 values
#define KOALA_NEW_GYRO_X  0    // position of newest X speed in the buffer
#define KOALA_NEW_GYRO_Y  1   // position of newest Y speed in the buffer
#define KOALA_NEW_GYRO_Z  2   // position of newest Z speed in the buffer

#define KOALA_MAGNE_VALUES_NUMBER 3  // x, y ,z

#define KOALA_MAX_I2C_DATA 256 // max i2c data to be send/received at one time


// robot hardware constants

#define KOALA_WHEELS_DISTANCE 250.0 // distance between wheel, for rotation calculus [mm]

#define KOALA_WHEEL_DIAM 82.5  // wheel diameter [mm] (real 85; without load 82.5, with 3kg load: 80.5)

#define KOALA_PULSE_TO_MM  (M_PI*KOALA_WHEEL_DIAM/23400.0) // motor position factor to convert from pulse to mm

#define KOALA_TIME_BTWN 10 // [ms] time for speed computation

#define KOALA_SPEED_TO_MM_S  (KOALA_PULSE_TO_MM/(KOALA_TIME_BTWN/1000.0)) // motor speed factor to convert from speed units to mm/s 

#define KOALA_US_DISABLED_SENSOR 2000  // disabled sensor
#define KOALA_US_NO_OBJECT_IN_RANGE 1000  // no object in range 25..250cm
#define KOALA_US_OBJECT_NEAR	0 // object at less 25cm

// default motor parameters
#define KOALA_MOTOR_P 10
#define KOALA_MOTOR_I  3
#define KOALA_MOTOR_D  1
#define KOALA_MOTOR_ACC_INC 5
#define KOALA_MOTOR_ACC_DIV 1
#define KOALA_MOTOR_MIN_SPACC 10
#define KOALA_MOTOR_CST_SPEED 200
#define KOALA_MOTOR_POS_MARGIN 10
#define KOALA_MOTOR_MAX_CURRENT 10 


#define KOALA_AD_TO_V (3.3/1024) // convert AD valu to Volt

/*!
 *  Koala Error codes
 */

#define KOALA_RS232_MESSAGE_ERROR_CHAR '#'

/*!

 *  Koala types
 */
 
// koala struct type for gps data
typedef struct  gps_data_s
{ 
	char valid_sat;			// valid data flag	
	int sat_nb;         // number of satellites used
	double lat_val;     // latitude
	char lat_car;       // latitude N or S
	double long_val;    // longitude
	char long_car; 			 // longitude W or E
	struct tm date_time; // UTC date and time of the last fix
	double	speed;		   // speed in knots
	int altitude;	       // altitude in meter
} gps_data_t;
 
 // koala struct type for auto mode
typedef struct auto_struct_s
{ 
  // for speed, current, position 
	int left_speed;     // motor left speed
	int right_speed;    // motor right speed
	int left_position;  // motor left position
	int right_position; // motor right position
	int left_current;   // motor left current
	int right_current;  // motor right current
	int us[KOALA_US_SENSORS_NUMBER]; // us sensors
	int accel[KOALA_ACCEL_VALUES_NUMBER]; // accelerometer
	int gyro[KOALA_GYRO_VALUES_NUMBER];   // gyrometer
	int magne[KOALA_MAGNE_VALUES_NUMBER]; // magnometer
	char gps_raw[KOALA_MAX_BUFFER];
	char mode; // type of the data received
	gps_data_t gps;
} koala_auto_data_t;


/*!--------------------------------------------------------------------
 * Prototypes Declaration
 */
 

// Is called by koala_init
extern int koala_robot_init( void );
extern int koala_robot_release( void );


// "Low level" function to communicate with the KOALA via serial
extern int koala_sendcommand(char *command,int write_len);
extern int koala_getcommand(char *command,int read_len);
extern int koala_getcommand_line(char *command);

// "High level" function that let user to simply retrieve various informations from the robot
extern int koala_get_firmware_version_revision(char *version,char *revision);

extern int koala_get_battery_data(int *bat_type,int *bat_voltage, int *bat_current,int *chrg_current);
extern int koala_configure_auto_monitoring_mode(unsigned int bit_config);
extern int koala_get_from_auto_mode(koala_auto_data_t *data);
extern int koala_configure_us_io(int us_mask,unsigned char io_dir);

extern int koala_configure_pid(int kp, int ki,int kd);
extern int koala_set_speed_profile(int acc_inc,int acc_div,int min_speed,int cst_speed,int pos_margin,int max_current);
extern int koala_set_position_encoders(int left, int right);


extern int koala_set_motor_speed(int left, int right);
extern int koala_set_motor_speed_accel(int left, int right);
extern int koala_set_motor_speed_open_loop(int left, int right);
extern int koala_read_motor_speed(int *left, int *right);
extern int koala_set_motor_target_position(int left, int right);
extern int koala_read_motor_current(int *left, int *right);
extern int koala_read_motor_position(int *left, int *right);
int koala_get_motor_status(int *left_status, int *right_status,int *left_pos, int *right_pos);

extern int koala_read_us_sensors(int *values_array);
extern int koala_read_accelerometer(int *values_array);
extern int koala_read_gyroscope(int *values_array);
extern int koala_read_magnetometer(int *values_array);

extern int koala_gps_data(char *valid_sat, int *sat_nb,double *lat_val,char *lat_car,double *long_val,char *long_car,struct tm *date_time,double *speed,int *altitude);
extern int koala_send_gps_cmd(char *gps_cmd);

extern int koala_read_i2c(int i2c_add,int i2c_reg, int nb_read,int *data);
extern int koala_write_i2c(int i2c_add,int i2c_reg,int nb_data,int *data);
extern int koala_scan_i2c(int *nb_devices,int *address);

extern int koala_set_pwr_io_output(int power_out,int IO0, int IO1, int IO2, int IO3);
extern int koala_read_io(int *io_state, int *in_state);
extern int koala_read_ad(int *ad_0, int *ad_1);

extern int koala_reset_microcontroller();

#endif /* __koala_robot__ */
