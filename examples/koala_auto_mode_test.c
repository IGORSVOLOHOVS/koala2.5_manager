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
/*!   \file koala_auto_mode_test.c
      \brief Audo mode test for the Koala robot of the koala library
*/
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include <koala/koala.h> // koala library


/*!
 * Main program
 *
 * \param argc number of arguments
 * \param argv argument array
 *
 * \return 0 if OK, <0 if error
 *
 */
int main(int argc, char *argv[]) {
  int rc=0;
  int i;
  double dval;
  koala_auto_data_t data;
  unsigned int bit_conf;
  char revision,version;
  double dmean;
  
	// initialise koala library
  if((rc=koala_init( argc , argv )) < 0 )
  {
    fprintf(stderr,"ERROR %d: Unable to initialize the koala library!\n",rc);
    return -1;
  }


  printf("Koala V2.5 robot Auto mode test program\n(C) K-Team S.A\n");
  
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
	
	// Set auto mode configuration
	
	bit_conf=KOALA_AUTOM_MOTOR_POSITION | KOALA_AUTOM_MAGNE_VALUE;
	
	//bit_conf=KOALA_AUTOM_ALL;
     
  //bit_conf=KOALA_AUTOM_GPS_DATA ;//KOALA_AUTOM_ACCEL_VALUE; //KOALA_AUTOM_US_SENSOR_BIT | KOALA_AUTOM_GPS_DATA ;

	rc= koala_configure_auto_monitoring_mode(bit_conf);
	if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set auto mode configuration!\n",rc);
    return -4;
  }  
	
  printf("Read serial until any key pushed!\n\n");
  while(koala_kbhit()==0)
  {
  	if (koala_get_from_auto_mode(&data)>=0) {
  	  //printf("\nreceived mode %c: \n",data.mode); 
  	  
		  switch(data.mode)
			{
				case 'e' : // Motor Speed
					 printf("\nMotor speed  : left: %5d  right: %5d\n",data.left_speed,data.right_speed);
				break;
				case 'g' : // US sensor
					printf("\nUS sensors:\n");
					for (i=0; i<KOALA_US_SENSORS_NUMBER;i++)
					{
						printf("  US %d: %d\n",i,data.us[i]);
					}
				break;
				case 'm' : // Accelerometer value
						printf("\nAcceleration sensor [g]\n       new data                                                      old data    average\naxis X: ");
						dmean=0;
						for (i=0;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
						{
							dval=data.accel[i]*KOALA_ACCEL_G;
							printf("%6.1f ",dval);
							dmean+=dval;                                                       
						}   
						
						printf(" %6.1f",dmean/10.0);

						printf("\naxis Y: ");
						dmean=0;
						for (i=1;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
						{

							dval=data.accel[i]*KOALA_ACCEL_G;
							printf("%6.1f ",dval);
							dmean+=dval;                               	                                                                   
						} 
						printf(" %6.1f",dmean/10.0);
							
						printf("\naxis Z: ");
						dmean=0;
						for (i=2;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
						{
							dval=data.accel[i]*KOALA_ACCEL_G;
							printf("%6.1f ",dval);
							dmean+=dval;                               	                                                                   
						}
						printf(" %6.1f",dmean/10.0);     
						printf("\n"); 
				break;
				case 'n' : // Gyroscope value
         		printf("\ngyro sensor [deg/s]\n       new data                                                      old data    average\ngyro X: ");
						dmean=0;
						for (i=0;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
						{
							dval=data.gyro[i]*KOALA_GYRO_DEG_S;
							printf("%6.1f ",dval);
							dmean+=dval;                                                       
						}   
						
						printf(" %6.1f",dmean/10.0);

						printf("\ngyro Y: ");
						dmean=0;
						for (i=1;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
						{

							dval=data.gyro[i]*KOALA_GYRO_DEG_S;
							printf("%6.1f ",dval);
							dmean+=dval;                               	                                                                   
						} 
						printf(" %6.1f",dmean/10.0);
							
						printf("\ngyro Z: ");
						dmean=0;
						for (i=2;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
						{
							dval=data.gyro[i]*KOALA_GYRO_DEG_S;
							printf("%6.1f ",dval);
							dmean+=dval;                               	                                                                   
						}
						printf(" %6.1f",dmean/10.0);     
						printf("\n");  
				break;
				case 'o' : // Motor Current
					printf("\nMotor current: left: %5d  right: %5d\n",data.left_current,data.right_current);
				break;
				case 'p' : // Motor Position
					 printf("\nMotor current: left: %5d  right: %5d\n",data.left_position,data.right_position);
				break;
				case 'q' : // GPS data	
				
					 printf("\nGPS data:\
					 				\n  valid mode: %c\
					        \n  sat number: %2d\
					        \n  latitude  : %5.4f %c\
					        \n  longitude : %5.4f %c\
					        \n  time      : %02d:%02d:%02d\
					        \n  date      : %02d.%02d.%02d\
					        \n  speed     : %3.1f [knots]\
					        \n  altitude  : %5d [m]\n",data.gps.valid_sat,data.gps.sat_nb,data.gps.lat_val,data.gps.lat_car,data.gps.long_val,data.gps.long_car,data.gps.date_time.tm_hour,data.gps.date_time.tm_min,data.gps.date_time.tm_sec,data.gps.date_time.tm_mday,data.gps.date_time.tm_mon,data.gps.date_time.tm_year,data.gps.speed,data.gps.altitude);
				break;
				case '$' : // GPS raw data
					printf("\nGPS raw data: %s\n",data.gps_raw);
				break;
				case '@':
					printf("\nMagnetometer: x:%5d  y:%5d  z:%5d [mGauss]\n",data.magne[0],data.magne[1],data.magne[2]);
				break;
				default:
				 printf("\nERROR: received invalid auto mode: %c (0x%x)\n",data.mode,data.mode);
			
			}
  	  
  	} 
  	 	
  	 	
  	usleep(10000);
  }
  
  bit_conf=KOALA_AUTOM_NONE;
  
  koala_configure_auto_monitoring_mode(bit_conf);

            
	return 0;
}
