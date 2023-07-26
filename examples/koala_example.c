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
/*!   \file koala_example.c
      \brief Example for using the Koala robot of the Koala library
*/
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>
#include <limits.h>

#include <koala/koala.h> // koala library

int main(int argc, char *argv[]) {
  int rc=0;
  int i,n;
  char version,revision;
  int Bat_Type,Bat_Voltage,Bat_Current,Bat_Capacity,Chrg_Current;
  unsigned int bit_conf;
  int us_mask;
  unsigned char io_dir;
  int sat_nb;double lat_val;char lat_car;double long_val;char long_car;struct tm date_time;double speed;int altitude; char valid_sat;  
  int us_sensors[KOALA_US_SENSORS_NUMBER]; 
  #define DATA_NUMBER 5
  int data[DATA_NUMBER];
  int lspeed,lpos,lcur,rspeed,rpos,rcur;
  int accel_array[KOALA_ACCEL_VALUES_NUMBER],gyro_array[KOALA_GYRO_VALUES_NUMBER];
  int magne_array[KOALA_MAGNE_VALUES_NUMBER];
  koala_auto_data_t auto_data;
  int magne_xmax,magne_xmin,magne_ymax,magne_ymin,magne_xrange,magne_yrange,magne[KOALA_MAGNE_VALUES_NUMBER];
  double magne_x,magne_y,angle,goal_angle;
  int out,auto_ctrl,refresh;
  char c;
  char line[80];
  
	// initialise koala library
  if((koala_init( argc , argv )) < 0 )
  {
    fprintf(stderr,"ERROR: Unable to initialize the koala library!\n");
    return -1;
  }


  printf("Koala V2.5 robot example program\n(C) K-Team S.A\n");
  
  
  // get robot firmware version and revision  
  rc=koala_get_firmware_version_revision(&version,&revision);
  
  if (rc<0)
  {
		rc=koala_get_firmware_version_revision(&version,&revision);
  
		if (rc<0) // retry, because of the serial start level
		{
		  fprintf(stderr,"ERROR %d: Koala did not respond correctly!\n",rc);
		  return -2;
		} 
    
  } 
  
  printf("Koala version: %c  revision: %d\n",version,revision);
  
  /*
  
  // ---- BATTERY ----
  // Get battery values
  rc=koala_get_battery_data(&Bat_Type,&Bat_Voltage,&Bat_Current,&Chrg_Current);
  
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not read Koala battery values!\n",rc);
    return -3;
  } 
    
  printf("\nBattery values:\
          \nType              :%s\
          \nVoltage           : %3.1f [V]\
          \nCurrent           : %4d [mA]\
          \nCharge current    : %4d [mA]\n" 
            ,Bat_Type==1?"NiMH":(Bat_Type==0?"Li-ION":"Not Init"),Bat_Voltage/10.0,Bat_Current*100,Chrg_Current*10);

	
	
	// Set auto mode configuration  
  bit_conf=KOALA_AUTOM_NONE;
	rc= koala_configure_auto_monitoring_mode(bit_conf);
	if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set auto mode configuration!\n",rc);
    return -4;
  }  
	
	
	// ---- IO AND US CONFIGURATION ----
	// Set some US and IO
	us_mask=KOALA_US_LEFT_FRONT | KOALA_US_RIGHT_FRONT | KOALA_US_BACK_LEFT | KOALA_US_BACK_RIGHT;
	io_dir=KOALA_IO_0_INPUT | KOALA_IO_1_PWMS | KOALA_IO_2_OUTPUT | KOALA_IO_3_INPUT;

	rc=koala_configure_us_io(us_mask,io_dir);
	if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set US, IO and GPS rate parameters!\n",rc);
    return -5;
  }
  
  
  // ---- US SENSORS ----
  // read US sensors
  rc=koala_read_us_sensors(us_sensors);
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set US, IO and GPS rate parameters!\n",rc);
    return -6;
  }
  
  
  printf("\nUS sensors:\n");
  for (i=0; i<KOALA_US_SENSORS_NUMBER;i++)
  {
    printf("  US %d: %d\n",i,us_sensors[i]);
  }

	// ---- GPS ----
	// Read gps
	rc=koala_gps_data(&valid_sat,&sat_nb,&lat_val,&lat_car,&long_val,&long_car,&date_time,&speed,&altitude);
	
	if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could get GPS data!\n",rc);
    return -7;
  }
	
	printf("\nGPS data:\n  data validity: %s sat number: %d\n  latitude: %5.4f %c\n  longitude: %5.4f %c\n  time: %02d:%02d:%02d\n  date: %02d.%02d.%02d\n  speed: %3.1f [knots]\n  altitude: %d\n",valid_sat=='A'?"yes":" no",sat_nb,lat_val,lat_car,long_val,long_car,date_time.tm_hour,date_time.tm_min,date_time.tm_sec,date_time.tm_mday,date_time.tm_mon,date_time.tm_year,speed,altitude);
  
  
  // ---- I2C ----
  // scan i2c
  rc=koala_scan_i2c(&n,data);						   
	if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not scan i2c bus!\n",rc);
    return -8;
  }
  printf("\ni2c address: found %d devices\n",n);
  for (i=0; i<n;i++)
	{
		printf(" device %d: i2c %d\n",i,data[i]);
	}
  
  
  // ----MOTOR CONFIGURATIONS ----
  // configure motor PID
  rc=koala_configure_pid(KOALA_MOTOR_P,KOALA_MOTOR_I,KOALA_MOTOR_D);
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set PID data!\n",rc);
    return -9;
  }
  
  // set motor speed profile
  rc=koala_set_speed_profile(KOALA_MOTOR_ACC_INC, KOALA_MOTOR_ACC_DIV,KOALA_MOTOR_MIN_SPACC,KOALA_MOTOR_CST_SPEED,KOALA_MOTOR_POS_MARGIN,KOALA_MOTOR_MAX_CURRENT);
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set speed profile!\n",rc);
    return -10;
  }
 
 
  // ---- MOTORS ENCODERS ---- 
   // set position encoders
  rc=koala_set_position_encoders(0,0); // reset encoders
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set position encoders!\n",rc);
    return -11;
  }


  // ---- ACCELEROMETER / GYROSCOPE ----
  // make the robot rotates in place: push any key to stop
  
  lspeed=50;
  rspeed=-50;
  printf("Make the robot rotates in place at speed: %6d %6d\n\n  Push any key to stop\n\n",lspeed,rspeed);
  usleep(800000);
  
  // set motor speed 
	rc=koala_set_motor_speed_accel(lspeed,rspeed);
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set motor speed !\n",rc);
    return -12;
  }
  
  // read speed, position, current and acceleration until any key is pushed
  while(koala_kbhit()==0)
  { 
  	koala_read_motor_speed(&lspeed,&rspeed);
		koala_read_motor_current(&lcur,&rcur);
		koala_read_motor_position(&lpos,&rpos);
		koala_read_accelerometer(accel_array);
		koala_read_gyroscope(gyro_array);
		
		printf("left : speed %6d  position: %6d  current: %6d\n",lspeed,lpos,lcur);
		printf("right: speed %6d  position: %6d  current: %6d\n",rspeed,rpos,rcur);
		printf("\nAcceleration sensor [g]:  x: %6.3f y: %6.3f z: %6.3f\n\n",accel_array[KOALA_NEW_ACCEL_X]*KOALA_ACCEL_G,accel_array[KOALA_NEW_ACCEL_Y]*KOALA_ACCEL_G,accel_array[KOALA_NEW_ACCEL_Z]*KOALA_ACCEL_G);
		printf("\nGyrometer sensor [deg/s]:  x: %6.3f y: %6.3f z: %6.3f\n\n",gyro_array[KOALA_NEW_GYRO_X]*KOALA_GYRO_DEG_S,gyro_array[KOALA_NEW_GYRO_Y]*KOALA_GYRO_DEG_S,gyro_array[KOALA_NEW_GYRO_Z]*KOALA_GYRO_DEG_S);
			
		
		usleep(300000);
  }
  
  koala_set_motor_speed(0,0); // stop robot
  
  */
  
  // ---- MAGNETOMETER ----
  
   // make the robot rotates in place: push any key to stop
  
  koala_set_position_encoders(0,0); // reset encoders
  lspeed=50;
  rspeed=-50;
  
  #define NB_TURN 2.5
  
  printf("The robot will rotate in place %3.1f turns to calibrate magnetometer at speed: %6d %6d\n\n  Push any key to stop\n\n",NB_TURN,lspeed,rspeed);
  usleep(800000);
  
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set motor speed !\n",rc);
    return -12;
  }
  
  // read speed, position, current and magnetometer until any key is pushed
  	
	// Set auto mode configuration  
  bit_conf=KOALA_AUTOM_MOTOR_POSITION | KOALA_AUTOM_MAGNE_VALUE;   // magnetometer and position
	koala_configure_auto_monitoring_mode(bit_conf);
	
	
	magne_xmax=magne_ymax=INT_MIN;
	magne_xmin=magne_ymin=INT_MAX;
	
	koala_set_motor_speed_accel(lspeed,rspeed);
	out=0;
	refresh=0;
	
	
  while((koala_kbhit()==0) && !out)
  { 
		
		if (koala_get_from_auto_mode(&auto_data)>=0) {
  	  //printf("\nreceived mode %c: \n",data.mode); 
  	  
		  switch(auto_data.mode)
			{
				case 'p' : // Motor Position
					 if (refresh>166) { // don't print all the data
					 	 printf("Position: left: %6.1f degree  right: %6.1f degree  Magnetometer: x:%5d  y:%5d  z:%5d [mGauss]\n",auto_data.left_position*360*KOALA_PULSE_TO_MM/M_PI/KOALA_WHEELS_DISTANCE,auto_data.right_position*360*KOALA_PULSE_TO_MM/M_PI/KOALA_WHEELS_DISTANCE,auto_data.magne[0],auto_data.magne[1],auto_data.magne[2]);
					 
					 	 refresh=0; 	 
					 }
					 
					 if (auto_data.left_position >NB_TURN*KOALA_WHEELS_DISTANCE*M_PI/KOALA_PULSE_TO_MM )  // test number of turn
					 {
					 	out=1;
					 }
					 
				break;
				case '@':
					/*if (i>166) { // don't print all the data
						printf("\nMagnetometer: x:%5d  y:%5d  z:%5d [mGauss]\n",auto_data.magne[0],auto_data.magne[1],auto_data.magne[2]);
						i=0;
					}*/
					
					if (auto_data.magne[0]>magne_xmax) {
						magne_xmax=auto_data.magne[0];
					} else
					if (auto_data.magne[0]<magne_xmin) {
						magne_xmin=auto_data.magne[0];
					}
				
					if (auto_data.magne[1]>magne_ymax) {
						magne_ymax=auto_data.magne[1];
					} else
					if (auto_data.magne[1]<magne_ymin) {
						magne_ymin=auto_data.magne[1];
					}
				
				break;
				default:
				 printf("\nERROR: received invalid auto mode: %c (0x%x)\n",auto_data.mode,auto_data.mode);		
			}
  	  
  	} 
		refresh++;	
		usleep(3000);
  }
  
  koala_set_motor_speed_open_loop(0,0);
  if (koala_kbhit())
	{
		printf("\n\nCalibration interrupted by a key, leaving program!\n");
		return -1;
	}
	
  
   
  	// Set auto mode configuration  
  bit_conf=KOALA_AUTOM_NONE;   // none
	koala_configure_auto_monitoring_mode(bit_conf);
  

	magne_xrange=magne_xmax-magne_xmin;
	magne_yrange=magne_ymax-magne_ymin;
	
	#define KEY_ROT_SPEED 50
	#define CONT_ROT_SPEED 30
	out=0;
	auto_ctrl=0;
	#define ANGLE_THRESH 5.0
  //	control angle with arrow keys 
  koala_change_term_mode(1); // change terminal mode for kbhit and getchar to return immediately
  refresh=0;
	while(!out)
	{ 
  
  	if (auto_ctrl)
  	{
  		if (((angle-goal_angle) < ANGLE_THRESH) && ((angle-goal_angle) > -ANGLE_THRESH))
  		{
  			auto_ctrl = 0;
  			koala_set_motor_speed(0,0);	 // stop robot
  			if (goal_angle==0) // NORTH
  				printf("\n\nNORTH angle reached (angle: %6.1f)!\n",angle);
  			else
  				printf("\n\nAngle %6.1f reached (angle: %6.1f)!\n",goal_angle,angle);
  			fflush(stdout); // make the display refresh
  			usleep(2000000);
  			continue;
  		} 
  	
  		// rotates correctly to approach goal 
  		if ((angle-goal_angle) > 0)
  		{
  			koala_set_motor_speed_accel(-CONT_ROT_SPEED ,CONT_ROT_SPEED);
  		} else
  		{
  			koala_set_motor_speed_accel(CONT_ROT_SPEED ,-CONT_ROT_SPEED);
  		}
  		
  	}
  
		if (koala_kbhit())
		{
			c=getchar();

			// get special keys
			if (c== 27  ) 
			{

			 if (c=getchar()==91) // escape with [
			 {
				 c = getchar(); 
			 
				 switch(c)
				 {

					case 68: // LEFT arrow = left
						koala_set_motor_speed_accel(-KEY_ROT_SPEED ,KEY_ROT_SPEED);
					break;

					case 67: // RIGHT arrow = right
						koala_set_motor_speed_accel(KEY_ROT_SPEED ,-KEY_ROT_SPEED);
					break;

					default:
						//printf("special key  : %d\n",c);
					break;
					} // switch(c)
				} // escape with [	
	
			} // if (c== '\027')	 
			else 
			{
				switch(c)
				{				
				 	case 'q': // quit to main menu
				 		out=1;
					 	break;
					case ' ': // spacebar
					case 's': // stop motor
					  auto_ctrl=0;
						koala_set_motor_speed(0,0);	 // stop robot
					break;
					case 'n': // rotates to the North
						auto_ctrl=1;
						goal_angle=0;
					break;
					case 'a': // enter angle
						koala_change_term_mode(0); // switch to normal key input mode
						printf("\nEnter angle value in the range [0.0-360[and push ENTER: ");
						fgets(line,80,stdin);
						if(sscanf(line,"%lf",&goal_angle)!=1) {
							printf("ERROR: invalid angle : %s\n",line);
							fflush(stdout); // make the display refresh
			  			usleep(2000000);
						} else
						{
							if ((goal_angle >=360) || (goal_angle<0)) {
								printf("ERROR: invalid angle range: angle %6.1f is not in range [0.0-360[ !",goal_angle);
								fflush(stdout); // make the display refresh
			  				usleep(2000000);
							} else
							{
								auto_ctrl=1;
							}
							
							
						}
						koala_change_term_mode(1); // switch to special key input mode again
					break;
			 	default:
				 		//printf("normal key : %d\n",c);
					 break;
				}
			}
		} // if (koala_kbhit())
		else
		{
			//koala_set_motor_speed(0,0);	 // stop robot
		}
		
		koala_read_magnetometer(magne);	
	 
		magne_x=(magne[0]-(magne_xmax+magne_xmin)/2.0)*2.0/magne_xrange;
		magne_y=(magne[1]-(magne_ymax+magne_ymin)/2.0)*2.0/magne_yrange;
		
		
		
		if (magne_x==0) {
		{
			if (magne_y>0) {
				angle=270.0;
			} else
			{
				angle=90.0;
			}	
		}
		} else
		{ 
			angle=-atan2(magne_y,magne_x)*180/M_PI; 
			if (angle <0) {
				angle+=360;
			}
		}	
		
		if (refresh>30)
		{
			koala_clrscr(); // erase screen
			printf("Magnetometer max ranges: x: %d  y: %d\nrelative x: %3.2f  relative y: %3.2f\n",magne_xrange,magne_yrange,magne_x,magne_y); 
			 
			if (auto_ctrl)
			{
				printf("Orientation (cw): %6.1f degrees in control mode: goal_angle=%6.1f\n",angle,goal_angle);
			} else
			{
				printf("Orientation (cw): %6.1f degrees \n",angle);
			}
			printf("\nKeys\n  q to quit  \n <- -> rotate left/right\n  n rotates to the North\n  a enter the angle to rotate to\n\n");
			refresh=0;
		}
		refresh++;
		usleep(10000);
	}
  koala_change_term_mode(0); // switch to normal key input mode	 
   
  koala_set_motor_speed_open_loop(0 ,0); // stop robot and release motors       
	return 0;
}
