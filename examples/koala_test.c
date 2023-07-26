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
/*!   \file koala_test.c
      \brief Test for the Koala robot of the koala library
*/
////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

#include <koala/koala.h> // koala library

#include <signal.h>
#include <math.h>



static int quitReq = 0; // quit variable for loop

/*--------------------------------------------------------------------*/
/*!
 * Make sure the program terminate properly on a ctrl-c
 */
static void ctrlc_handler( int sig ) 
{
  quitReq = 1;
  
  koala_set_motor_speed_open_loop(0,0); 
  koala_change_term_mode(0); // revert to original terminal if called
  exit(0);
}

/*--------------------------------------------------------------------*/

/*!
 * convert integer to string in specific base
 *
 * \param val value to be converted
 * \param base base of the value 
 *
 * \return converted number in zero string
 */
char *kitoa(int val, int base){
    static char buf[32] = {0};
    int i = 30;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    return &buf[i+1];

}





/*!
 * Compute time difference
 *

 * \param difference difference between the two times, in structure timeval type
 * \param end_time end time
 * \param start_time start time  
 *
 * \return difference between the two times in [us]
 *
 */
long long
timeval_diff(struct timeval *difference,
             struct timeval *end_time,
             struct timeval *start_time
            )
{
  struct timeval temp_diff;

  if(difference==NULL)
  {
    difference=&temp_diff;
  }

  difference->tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference->tv_usec=end_time->tv_usec-start_time->tv_usec;

  /* Using while instead of if below makes the code slightly more robust. */

  while(difference->tv_usec<0)
  {
    difference->tv_usec+=1000000;
    difference->tv_sec -=1;
  }

  return 1000000LL*difference->tv_sec+
                   difference->tv_usec;

} /* timeval_diff() */



 
 // #define for driver mode
#define BIG_SPEED_FACTOR 25
#define SPEED_FACTOR 1
#define MAX_SPEED 500
#define MIN_SPEED 1
#define DEFAULT_SPEED 200

#define ROTATE_HIGH_SPEED_FACT 0.5
#define ROTATE_LOW_SPEED_FACT 0.7
#define ROTATE_DIG_SPEED_FACT 0.5
#define ROT_SPEED_HIGH_TRESH 300
#define STOP_TIME 100000 // us

#define SIGN(x) ((x)>0?1:((x)<0?-1:0))  // sign or zero
/*!
 * Drive the robot with the keyboard
 *
 * \param none
 *
 * \return an error code:

 *         - <0  standard error code. See errno.h
 *         - >=0 on success
 *
 */
int drive_robot()
{
	int out=0,speed=DEFAULT_SPEED,vsl,vsr,anymove=0;
	char c;
	//struct timeval startt,endt;

	
	koala_clrscr(); // erase screen
	
	printf("Drive the robot with the keyboard:\n  's' 5 or spacebar for stop\n  arrows (UP, DOWN, LEFT, RIGHT, or NUMPAD 1-9) for direction\n  PAGE UP/DOWN for changing speed  by small increments\n  Home/End for changing speed by big increments\n  'q' for going back to main menu\n");
	
	
	printf("\ndefault parameters:\n  robot speed %d  (%5.1f mm/s)  (min %d, max %d)\n\n",DEFAULT_SPEED,DEFAULT_SPEED*KOALA_SPEED_TO_MM_S,MIN_SPEED,MAX_SPEED);
	
	koala_change_term_mode(1); // change terminal mode for kbhit and getchar to return immediately
	

	
	//gettimeofday(&startt,0x0);
	
	// loop until 'q' is pushed
	while(!out)
	{
		if(koala_kbhit())
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
					case 65: // UP arrow = forward
							 koala_set_motor_speed_accel(speed ,speed);
							anymove=1;						
					break;
					case 66: // DOWN arrow = backward			
							 koala_set_motor_speed_accel(-speed ,-speed);
							anymove=1;
					break;

					case 68: // LEFT arrow = left
							if (speed > ROT_SPEED_HIGH_TRESH) // at high speed, rotate too fast
								 koala_set_motor_speed_accel(-speed*ROTATE_HIGH_SPEED_FACT ,speed*ROTATE_HIGH_SPEED_FACT);
							else
								 koala_set_motor_speed_accel(-speed*ROTATE_LOW_SPEED_FACT ,speed*ROTATE_LOW_SPEED_FACT);
							anymove=1;	
					break;

					case 67: // RIGHT arrow = right
							if (speed > ROT_SPEED_HIGH_TRESH) // at high speed, rotate too fast
								 koala_set_motor_speed_accel(speed*ROTATE_HIGH_SPEED_FACT ,-speed*ROTATE_HIGH_SPEED_FACT);
							else
								 koala_set_motor_speed_accel(speed*ROTATE_LOW_SPEED_FACT ,-speed*ROTATE_LOW_SPEED_FACT );
							anymove=1;	
					break;

					case 53: // PAGE UP  = speed up
						speed+=SPEED_FACTOR;
				 		if (speed>MAX_SPEED)
				 		{
							speed=MAX_SPEED;
				 		};
				 		c = getchar(); // get last character
				 		
				 		koala_read_motor_speed(&vsl,&vsr);
				 		koala_set_motor_speed_accel(SIGN(vsl)*speed ,SIGN(vsr)*speed); // set new speed, keeping direction with sign
				 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KOALA_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
				 		fflush(stdout); // make the display refresh
				 		anymove=1;
					break;

					case 54: // PAGE DOWN = speed down
						speed-=SPEED_FACTOR;
				 		if (speed<MIN_SPEED)
				 		{
							speed=MIN_SPEED;
				 		};
				 		c = getchar(); // get last character
				 		
				 		koala_read_motor_speed(&vsl,&vsr);
				 		koala_set_motor_speed_accel(SIGN(vsl)*speed ,SIGN(vsr)*speed); // set new speed, keeping direction with sign
				 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KOALA_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
				 		fflush(stdout); // make the display refresh
				 		anymove=1;
					break;
			

					default:
						//printf("special key  : %d\n",c);
					break;
					} // switch(c)
				} // escape with [
				else
				{ // other special key code
					
					 c = getchar(); 
					 
					switch(c){
				
						case 72: // Home  = speed up
							speed+=BIG_SPEED_FACTOR;
					 		if (speed>MAX_SPEED)
					 		{
								speed=MAX_SPEED;
					 		};
					 		//c = getchar(); // get last character
					 		
					 		 koala_read_motor_speed(&vsl,&vsr);
					 		 koala_set_motor_speed_accel(SIGN(vsl)*speed ,SIGN(vsr)*speed); // set new speed, keeping direction with sign
					 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KOALA_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
					 		fflush(stdout); // make the display refresh
					 		anymove=1;
						break;

						case 70: // End = speed down
							speed-=BIG_SPEED_FACTOR;
					 		if (speed<MIN_SPEED)
					 		{
								speed=MIN_SPEED;
					 		};
					 		//c = getchar(); // get last character
					 		
					 		koala_read_motor_speed(&vsl,&vsr);
					 		koala_set_motor_speed_accel(SIGN(vsl)*speed ,SIGN(vsr)*speed); // set new speed, keeping direction with sign
					 		printf("\033[1`\033[Krobot speed: %d (%5.1f mm/s)",speed,speed*KOALA_SPEED_TO_MM_S); // move cursor to first column, erase line and print info
					 		fflush(stdout); // make the display refresh
					 		anymove=1;
						break;
						
						default:
							//printf("other special key  : %d\n",c);
						break	;	
						
					}  
			
				} // other special key code
							
				
			} // if (c== '\027')	 
			else 
			{
				switch(c)
				{
				 	case 'q': // quit to main menu
				 		out=1;
				   	break;
					case 's': // stop motor
					case ' ': // spacebar
					case 53:  // 5
						koala_set_motor_speed(0,0);	 // stop robot
					break;
					case 49: // 1 = left backward
							 koala_set_motor_speed_accel(-speed*ROTATE_DIG_SPEED_FACT ,-speed);
							anymove=1;						
					break;
					case 50: // 2 = backward			
							koala_set_motor_speed_accel(-speed ,-speed);
							anymove=1;
					break;
					case 51: // 3 = right backward
						 koala_set_motor_speed(-speed ,-speed*ROTATE_DIG_SPEED_FACT);
					break;
					case 52: // 4 = left
						if (speed > ROT_SPEED_HIGH_TRESH) // at high speed, rotate too fast
							 koala_set_motor_speed_accel(-speed*ROTATE_HIGH_SPEED_FACT ,speed*ROTATE_HIGH_SPEED_FACT);
						else
							 koala_set_motor_speed_accel(-speed*ROTATE_LOW_SPEED_FACT ,speed*ROTATE_LOW_SPEED_FACT);
						anymove=1;	
						break;
					case 54: // 6 = right
						if (speed > ROT_SPEED_HIGH_TRESH) // at high speed, rotate too fast
							 koala_set_motor_speed_accel(speed*ROTATE_HIGH_SPEED_FACT ,-speed*ROTATE_HIGH_SPEED_FACT);
						else
							 koala_set_motor_speed_accel(speed*ROTATE_LOW_SPEED_FACT ,-speed*ROTATE_LOW_SPEED_FACT );
						anymove=1;	
					break;
				  case 55: // 7 = left forward
							 koala_set_motor_speed_accel(speed*ROTATE_DIG_SPEED_FACT ,speed);
							anymove=1;						
					break;
				  case 56: // 8 = forward
							 koala_set_motor_speed_accel(speed ,speed);
							anymove=1;						
					break;
					case 57: // 9 = right forward
							 koala_set_motor_speed_accel(speed,speed*ROTATE_DIG_SPEED_FACT);
							anymove=1;						
					break;
			 	default:
				 		//printf("normal key : %d\n",c);
				   break;
				}
		  }
		  
		  
		} /*else
		{
		
			gettimeofday(&endt,0x0);;
			// stop when no key is pushed after some time
			
			if (anymove &&  (timeval_diff(NULL,&endt,&startt)>STOP_TIME))
			{
				 koala_set_motor_speed(0 ,0);
				anymove=0;
			}	
				
		}*/
		


		usleep(10000); // wait some ms
	} // while

	koala_change_term_mode(0); // switch to normal key input mode	
	koala_set_motor_speed(0,0);	 // stop robot
	return 0;
}

/*--------------------------------------------------------------------*/
/*! Braintenberg demo program
 *
 *  \param none.
 */

#define BR_IRGAIN 1
#define fwSpeed 200//150
#define BMIN_SPEED 10

#define RotSpeedL 50
#define RotSpeedR -50

#define MAX_DIST 60
#define MIN_DIST 25
 
int braitenbergAvoidance()
{
  int Connections_B[9] = { -2, -3, -4, -12,  4,  3,  2, 1, 1}; // weight of every 9 sensor for the right motor 
  int Connections_A[9] = { 2,  3,  4, -10, -4, -3, -2, 1, 1}; // weight of every 9 sensor for the left motor 
  int i, buflen, sensval;


  int lspeed, rspeed;
  int tabsens[9];
  int left_speed, right_speed;
  unsigned int immobility = 0;
  unsigned int prevpos_left, pos_left, prevpos_right,  pos_right;
  int us_sensors[KOALA_US_SENSORS_NUMBER];
  struct timeval startt,endt;
  int bat_type,bat_voltage,bat_current,chrg_current;
  
  koala_auto_data_t data;


	koala_configure_us_io(KOALA_US_ALL,0); // set all us sensors ON

  koala_read_motor_position(&prevpos_left, &prevpos_right);

	koala_configure_auto_monitoring_mode(KOALA_AUTOM_US_SENSOR_BIT);

	printf("\nPush ANY KEY to stop!\n");

	gettimeofday(&startt,0x0);
	
  while(!koala_kbhit())
  {
    lspeed = 0; rspeed = 0;
    
    // display battery value every 30s
    gettimeofday(&endt,0x0);
		if ((timeval_diff(NULL,&endt,&startt)>30000000))
		{
				koala_get_battery_data(&bat_type,&bat_voltage,&bat_current,&chrg_current);
	 			printf("\nBattery: Voltage =%3.1f [V]  Current=%5d [mA]\n", 
		        bat_voltage/10.0,bat_current*100);
				gettimeofday(&startt,0x0);
		}	

	// check if any value received from the robot and process it
	if (koala_get_from_auto_mode(&data)>=0) {
  	 
  	 //printf("\nreceived mode %c: \n",data.mode); 
  	  
		switch(data.mode) {
     case 'g': // ultrasounds
	     
				//limit the sensor values
				for (i = 0; i < KOALA_US_SENSORS_NUMBER; i++)	
				{
					sensval = data.us[i];
					if(sensval > MAX_DIST)
						tabsens[i] = 0;
					else if (sensval < MIN_DIST)
						tabsens[i] = MAX_DIST-MIN_DIST;
					else
						tabsens[i] = MAX_DIST-sensval;
				}

				for (i = 0; i < 9; i++)
				{
				  lspeed += Connections_A[i] * tabsens[i];
				  rspeed += Connections_B[i] * tabsens[i];				
				}

				left_speed = ((lspeed * BR_IRGAIN) + fwSpeed);
				right_speed = ((rspeed * BR_IRGAIN) + fwSpeed);

				if(left_speed > 0 && left_speed < BMIN_SPEED)
				  left_speed = BMIN_SPEED;
				if(left_speed < 0 && left_speed > -BMIN_SPEED)
				  left_speed = -BMIN_SPEED;
				if(right_speed > 0 && right_speed < BMIN_SPEED)
				  right_speed = BMIN_SPEED;
				if(right_speed < 0 && right_speed > -BMIN_SPEED)
				  right_speed = -BMIN_SPEED;

				koala_set_motor_speed_accel(left_speed ,right_speed);

				//printf("lens = %d, rsens = %d lspd = %d rspd = %d\r\n", (int)lspeed16, (int)rspeed16, left_speed, right_speed);	

				/*left_speed = kmot_GetMeasure( mot1 , kMotMesSpeed );
				right_speed = kmot_GetMeasure( mot2 , kMotMesSpeed );
				koala_read_motor_speed(int *left, int *right);*/
				

				/* Get the new position of the wheel to compare with previous values */
				/*koala_read_motor_position(&pos_left, &pos_right);

				if((pos_left < (prevpos_left + 700)) && (pos_left > (prevpos_left -700)) && (pos_right < (prevpos_right + 700)) && (pos_right > (prevpos_right -700)))
				{
				  if(++immobility > 5)

				  {
				     left_speed = RotSpeedL;
				     right_speed = RotSpeedR;

							koala_set_motor_speed_accel(left_speed ,right_speed);

			 do{
					usleep(10000);
					koala_read_us_sensors(us_sensors);
				}while ((us_sensors[2] >250) || (us_sensors[3] >250) || (us_sensors[4] >250));

				     immobility = 0;
				     prevpos_left = pos_left;
				     prevpos_right = pos_right;
				  }
				}
				else
				{

				   immobility = 0;
				   prevpos_left = pos_left;
				   prevpos_right = pos_right;
				}*/
				break;
				
				default:
					printf("\nERROR: received invalid auto mode: %c (0x%x)\n",data.mode,data.mode);
			}
    //printf("lspd = %d rspd = %d\r\n", left_speed, right_speed); 			
		}

    usleep(20000); 
  }
  
  koala_configure_auto_monitoring_mode(KOALA_AUTOM_NONE);
  
  koala_set_motor_speed(0 ,0); // stop robot
  
}


/*!
 * test sound
 *
 *
 * \return 0 if OK, <0 if error
 *
 */
int test_sound()
{
	return 0;
}

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
  
#define US_BAR_LEN 23 	// display bar length for US sensor
#define ACGY_BAR_LEN 30 // display bar length for Accel/gyro sensor
#define MAX_US_DISTANCE 250.0 // max distance US
#define MAX_G 2 		// max acceleration in g

// convert US value to text comment
#define US_VAL(val) ((val)==KOALA_US_DISABLED_SENSOR ? "Not activated" : ((val)==KOALA_US_NO_OBJECT_IN_RANGE ? "No object in range" : ((val)==KOALA_US_OBJECT_NEAR ? "Object at less than 25cm" : "Object in range 25..250cm")))

  double fpos,dval,dmean;
  char Buffer[256],line[80]; // Buffer must be >= line
  char bar[12][64],revision,version;
  int rc=0,i,n,type_of_test=0,sl,sr,pl,pr;
  char c;
  long motspeed;
  char l[9];
  int kp,ki,kd;
  int acc_inc,acc_div,min_spacc,cst_speed,pos_margin,max_current;
  int accel_array[KOALA_ACCEL_VALUES_NUMBER],gyro_array[KOALA_GYRO_VALUES_NUMBER];
  int Bat_Type,Bat_Voltage,Bat_Current,Chrg_Current;
  int lspeed,lpos,lcur,rspeed,rpos,rcur;
  int us_mask,io_dir;
  char valid_sat;int sat_nb;double lat_val;char lat_car;double long_val;char long_car;struct tm date_time;double speed;int altitude;
  int us_sensors[KOALA_US_SENSORS_NUMBER];
  int i2c_add,i2c_reg;
  int i2c_data[KOALA_MAX_I2C_DATA];
  int io_state,in_state,ad_0,ad_1;
  int auto_mon=0;
  int power_out,IO0,IO1,IO2,IO3;
  int left_status,right_status,left_pos,right_pos;
  char *motor_ctrl_status_str[] ={"Idle","Speed","Speed with Acceleration","Position","Open Loop","Current Limitation","Error"};
  int magne[KOALA_MAGNE_VALUES_NUMBER];
  koala_auto_data_t auto_data;
  
	// initialise koala library
  if((rc=koala_init( argc , argv )) < 0 )
  {
    fprintf(stderr,"ERROR %d: Unable to initialize the koala library!\n",rc);
    return -1;
  }


  printf("Koala V2.5 robot test program\n(C) K-Team S.A\n");
  
	rc=koala_get_firmware_version_revision(&version,&revision);
 	
	if (rc<0)
  {
		rc=koala_get_firmware_version_revision(&version,&revision);
  
		if (rc<0) // retry, because of the serial start level
		{
		  fprintf(stderr,"ERROR %d: Koala did not respond correctly (could not get version)!\n",rc);
		  return -2;
		} 
    
  } 
  
  
   // Set auto mode configuration  
  rc= koala_configure_auto_monitoring_mode(auto_mon);
	if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set auto mode configuration!\n",rc);
    return -3;
  } 

  
  printf("Koala version: %c  revision: %d\n",version,revision);
  
  
  // configure motor PID
  kp=KOALA_MOTOR_P;
  ki=KOALA_MOTOR_I;
  kd=KOALA_MOTOR_D;
  rc=koala_configure_pid(kp,ki,kd);
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set PID data!\n",rc);
    return -4;
  } else
  {
  	printf("PID set to default values: P=%d I=%d D=%d\n",kp,ki,kd);
  }
  
  // set motor speed profile
  acc_inc=KOALA_MOTOR_ACC_INC;
  acc_div=KOALA_MOTOR_ACC_DIV;
  min_spacc=KOALA_MOTOR_MIN_SPACC;
  cst_speed=KOALA_MOTOR_CST_SPEED;
  pos_margin=KOALA_MOTOR_POS_MARGIN;
  max_current=KOALA_MOTOR_MAX_CURRENT;
  rc=koala_set_speed_profile(acc_inc,acc_div,min_spacc, cst_speed,pos_margin,max_current);
  if (rc<0)
  {
    fprintf(stderr,"ERROR %d: could not set speed profile!\n",rc);
    return -5;
  } else
  {
  	printf("Speed profile set to default values:\n  acc_inc=%d, acc_div= %d, min_spacc=%d, cst_speed=%d, pos_margin=%d, max_current=%d\n",acc_inc,acc_div,min_spacc, cst_speed,pos_margin,max_current);
  }
  
   
  signal( SIGINT , ctrlc_handler ); // set signal for catching ctrl-c
  
  /* For ever loop until ctrl-c key */
  while(quitReq==0)
  {
  	
  	// print menu
		printf("\nChoose an option: enter (letter) then [its argument(s)], then push ENTER:\n\n");
		printf("  (ad) read A/D ports\n");
		printf("  (ag) accel and gyro sensors\n");
		printf("  (am) auto monitoring mode [mask]  OR active mask: 511=ALL 256=Magne 128=NEMA 64=GPS 32=Gyro\n       16=Accel 8=Current 4=Position 2=Speed 1=US sensor (current: %d (0b%s))\n",auto_mon,kitoa(auto_mon,2));
		printf("  (b)attery status\n");
		printf("  (br)aitenberg avoidance algorithm\n");
		printf("  (d)rive the robot with keyboard\n");
		printf("  (g)et motors speed and position\n");
		printf("  (gp)s get data\n");
		printf("  (gs) gps sent data [data_string]\n");
    printf("  (io) get the digital input values\n");
		printf("  (ir) i2c read [address register number_of_data]\n");
		printf("  (iw) i2c write [address register number_of_data data_1 data_2 ...]\n");
		printf("  (is) i2c scan\n");
		printf("  (ma)gnetometer sensor\n");
		printf("  (ms) motor speed [left] [right] speed in [pulse/%dms]  (1 pulse/%dms = %.3f mm/s ; 1mm/s = %.3f pulses/%dms)\n",KOALA_TIME_BTWN,KOALA_TIME_BTWN,KOALA_SPEED_TO_MM_S,1.0/KOALA_SPEED_TO_MM_S,KOALA_TIME_BTWN);
		printf("  (msp) motor speed profile [left] [right] speed in [pulse/%dms]  (1 pulse/%dms = %.3f mm/s ; 1mm/s = %.3f pulses/%dms)\n",KOALA_TIME_BTWN,KOALA_TIME_BTWN,KOALA_SPEED_TO_MM_S,1.0/KOALA_SPEED_TO_MM_S,KOALA_TIME_BTWN);
		printf("  (mso) motor speed open loop [left] [right] speed (-2000..2000 => -100%%..100%%))\n");		
		printf("  (mst) get motor status\n");
		printf("  (mp) motor position [left] [right] absolute position in [pulses]  (1 pulse = %.4f mm ; 1mm = %.2f pulses)\n",KOALA_PULSE_TO_MM,1.0/KOALA_PULSE_TO_MM);
		printf("  (pa)rameteres [us_mask io_dir]  (us_mask: 1=left rear, 2=left front, 4=front left,\n       8=front, 16=front right, 32=right front, 64=right rear, 128=back right, 256=back left, all=511, 0=none)\n");
		printf("  (pi)d [p i d]  current: %d %d %d\n",kp,ki,kd);
		printf("  (po) set power and io [PWR_Value IO0 IO1 IO2 IO3 ]   (PWR_Value: 1=PW0,2=PWR1,4=PWR2,8=PWR3   IO=0 or 1)\n");
		printf("  (s)top motor\n");
		printf("  (sp) speed profile [Acc_inc  Acc_div Min_speed_acc cst_speed pos_margin max_current ]  current: %d %d %d %d %d %d\n",acc_inc,acc_div,min_spacc, cst_speed,pos_margin,max_current); 
		printf("  (re)set encoders\n");
		printf("  (rm) reset the microcontroller\n");
		printf("  (q)uit program\n");
		printf("  (u)ltrasonic sensors values\n");
		

		
		printf("\noption: "); 
		
		// wait and save choice
		fgets(line,80,stdin);
		
		
		// applay selected choice
		switch(line[0])
		{
			case 'a': // accel and gyro, AD, auto mode
				if ((strlen(line)>1 && (line[1]=='g')))
				{		
					while(!koala_kbhit())
					{	
						koala_clrscr();
						// get gyro sensor
						printf("\ngyro sensor [deg/s]\n       new data                                                      old data    average\ngyro X: ");
						if (koala_read_gyroscope(gyro_array)<0)
						{
							printf("\nERROR: could not get gyro data!\n");
							break;
						}
						
						dmean=0;
						for (i=0;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
						{
							dval=gyro_array[i]*KOALA_GYRO_DEG_S;
							printf("%6.1f ",dval);
							dmean+=dval;                                                       
						}   
						
						printf(" %6.1f",dmean/10.0);

						printf("\ngyro Y: ");
						dmean=0;
						for (i=1;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
						{

							dval=gyro_array[i]*KOALA_GYRO_DEG_S;
							printf("%6.1f ",dval);
							dmean+=dval;                               	                                                                   
						} 
						printf(" %6.1f",dmean/10.0);
							
						printf("\ngyro Z: ");
						dmean=0;
						for (i=2;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
						{
							dval=gyro_array[i]*KOALA_GYRO_DEG_S;
							printf("%6.1f ",dval);
							dmean+=dval;                               	                                                                   
						}
						printf(" %6.1f",dmean/10.0);     
						printf("\n");



						// get accel sensor
						if (koala_read_accelerometer(accel_array)<0)
						{
							printf("\nERROR: could not get accel data!\n");
							break;
						}

						printf("\nAcceleration sensor [g]\n       new data                                            old data  average  [g]: -2     -1      0      1      2\nacc  X: ");
						dmean=0;
						for (i=0;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
						{
							dval=accel_array[i]*KOALA_ACCEL_G;
							printf("%5.2f ",dval);
							dmean+=dval;                                                                                       
						}
						
						dval=dmean/10.0;
						
						printf(" %5.2f",dval);   
						
						// compute bar index
						n = (int)abs(dval*ACGY_BAR_LEN/MAX_G /2.0); 
						// fill up bar
						if (dval < 0)
						{
							for (i=0;i<ACGY_BAR_LEN/2-n;i++)
								bar[0][i]=' ';
						 	for (i=ACGY_BAR_LEN/2-n;i<ACGY_BAR_LEN/2;i++)
								bar[0][i]='-';
							bar[0][ACGY_BAR_LEN/2]='0';								
							for (i=ACGY_BAR_LEN/2+1;i<ACGY_BAR_LEN;i++)
								bar[0][i]=' ';  
						}
						else
						{
							for (i=0;i<ACGY_BAR_LEN/2;i++)
								bar[0][i]=' '; 
								
					    bar[0][ACGY_BAR_LEN/2]='0';	
					
							for (i=ACGY_BAR_LEN/2+1;i<ACGY_BAR_LEN/2+n+1;i++)
							  bar[0][i]='+';
				 
							for (i=ACGY_BAR_LEN/2+n+1;i<ACGY_BAR_LEN;i++)
								bar[0][i]=' ';
						}
								 
				 	  bar[0][ACGY_BAR_LEN/2]='0';
					
						bar[0][ACGY_BAR_LEN]='\0';
					
						printf("        |%s|",bar[0]);
							
						printf("\nacc  Y: ");
						dmean=0;
						for (i=1;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
						{
							dval=accel_array[i]*KOALA_ACCEL_G;
							printf("%5.2f ",dval);
							dmean+=dval;
						}
						
						dval=dmean/10.0;
						printf(" %5.2f",dval);
						
						// compute bar index
						n = (int)abs(dval*ACGY_BAR_LEN/MAX_G /2.0); 
						// fill up bar
						if (dval < 0)
						{
							for (i=0;i<ACGY_BAR_LEN/2-n;i++)
								bar[0][i]=' ';
						 	for (i=ACGY_BAR_LEN/2-n;i<ACGY_BAR_LEN/2;i++)
								bar[0][i]='-';
							bar[0][ACGY_BAR_LEN/2]='0';
								
							for (i=ACGY_BAR_LEN/2+1;i<ACGY_BAR_LEN;i++)
								bar[0][i]=' ';  
						}
						else

						{
							for (i=0;i<ACGY_BAR_LEN/2;i++)
								bar[0][i]=' '; 
								

					    bar[0][ACGY_BAR_LEN/2]='0';	
					
							for (i=ACGY_BAR_LEN/2+1;i<ACGY_BAR_LEN/2+n+1;i++)
							  bar[0][i]='+';
				 
							for (i=ACGY_BAR_LEN/2+n+1;i<ACGY_BAR_LEN;i++)

								bar[0][i]=' ';
						}
								 
				 	  bar[0][ACGY_BAR_LEN/2]='0';
					
						bar[0][ACGY_BAR_LEN]='\0';
					
						printf("        |%s|",bar[0]);
						
						printf("\nacc  Z: ");
						dmean=0;
						for (i=2;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
						{
							dval=accel_array[i]*KOALA_ACCEL_G;
							printf("%5.2f ",dval);
							dmean+=dval;
						}
						
						dval=dmean/10.0;
						printf(" %5.2f",dval);
						
						// compute bar index
						n = (int)abs(dval*ACGY_BAR_LEN/MAX_G/2.0); 
						// fill up bar
						if (dval < 0)
						{
							for (i=0;i<ACGY_BAR_LEN/2-n;i++)
								bar[0][i]=' ';
						 	for (i=ACGY_BAR_LEN/2-n;i<ACGY_BAR_LEN/2;i++)
								bar[0][i]='-';
							bar[0][ACGY_BAR_LEN/2]='0';
								
							for (i=ACGY_BAR_LEN/2+1;i<ACGY_BAR_LEN;i++)
								bar[0][i]=' ';  
						}
						else
						{
							for (i=0;i<ACGY_BAR_LEN/2;i++)
								bar[0][i]=' '; 
								
					    bar[0][ACGY_BAR_LEN/2]='0';	
					
							for (i=ACGY_BAR_LEN/2+1;i<ACGY_BAR_LEN/2+n+1;i++)
							  bar[0][i]='+';
				 
							for (i=ACGY_BAR_LEN/2+n+1;i<ACGY_BAR_LEN;i++)
								bar[0][i]=' ';
						}
								 
				 	  bar[0][ACGY_BAR_LEN/2]='0';
					
						bar[0][ACGY_BAR_LEN]='\0';
					
						printf("        |%s|",bar[0]);
						
						
						printf("\n\nPush any key to stop\n");
						usleep(200000); // wait 200ms
					}
					tcflush(0, TCIFLUSH);
				} else
				if ((strlen(line)>1 && (line[1]=='d')))
				{	// A/D	
					while(!koala_kbhit())
					{	
						koala_clrscr();
						if (koala_read_ad(&ad_0,&ad_1)<0)
						{
							printf("\nERROR: could not get A/D data!\n");
							break;
						}
						printf("AD0: %3d (%.3f V)  AD1: %3d (%.3f V)\n",ad_0,ad_0*KOALA_AD_TO_V,ad_1,ad_1*KOALA_AD_TO_V);
						printf("\n\nPush any key to stop\n");
						usleep(200000); // wait 200ms
					}
					tcflush(0, TCIFLUSH);
				}	else
				if ((strlen(line)>1 && (line[1]=='m')))
				{ // auto monitoring mode
					if (EOF==sscanf(line,"%*c%*c %d",&auto_mon))
						printf("\n*** ERROR ***: auto_mon not correct\n");
					else
					{
						koala_configure_auto_monitoring_mode(auto_mon);
						printf("\nPush any key to stop\n");
						sleep(1);
						while(!koala_kbhit())
						{	
							if (koala_get_from_auto_mode(&auto_data)>=0) {
  	  				//printf("\nreceived mode %c: \n",auto_data.mode); 
  	  
								switch(auto_data.mode)
								{
									case 'e' : // Motor Speed
										 printf("\nMotor speed  : left: %5d  right: %5d\n",auto_data.left_speed,auto_data.right_speed);
									break;
									case 'g' : // US sensor
										printf("\nUS sensors:\n");
										for (i=0; i<KOALA_US_SENSORS_NUMBER;i++)
										{
											printf("  US %d: %d\n",i,auto_data.us[i]);
										}
									break;
									case 'm' : // Accelerometer value
											printf("\nAcceleration sensor [g]\n       new data                                                      old data    average\naxis X: ");
											dmean=0;
											for (i=0;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
											{
												dval=auto_data.accel[i]*KOALA_ACCEL_G;
												printf("%6.1f ",dval);
												dmean+=dval;                                                       
											}   
						
											printf(" %6.1f",dmean/10.0);

											printf("\naxis Y: ");
											dmean=0;
											for (i=1;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
											{

												dval=auto_data.accel[i]*KOALA_ACCEL_G;
												printf("%6.1f ",dval);
												dmean+=dval;                               	                                                                   
											} 
											printf(" %6.1f",dmean/10.0);
							
											printf("\naxis Z: ");
											dmean=0;
											for (i=2;i<KOALA_ACCEL_VALUES_NUMBER;i+=3)
											{
												dval=auto_data.accel[i]*KOALA_ACCEL_G;
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
												dval=auto_data.gyro[i]*KOALA_GYRO_DEG_S;
												printf("%6.1f ",dval);
												dmean+=dval;                                                       
											}   
						
											printf(" %6.1f",dmean/10.0);

											printf("\ngyro Y: ");
											dmean=0;
											for (i=1;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
											{

												dval=auto_data.gyro[i]*KOALA_GYRO_DEG_S;
												printf("%6.1f ",dval);
												dmean+=dval;                               	                                                                   
											} 
											printf(" %6.1f",dmean/10.0);
							
											printf("\ngyro Z: ");
											dmean=0;
											for (i=2;i<KOALA_GYRO_VALUES_NUMBER;i+=3)
											{
												dval=auto_data.gyro[i]*KOALA_GYRO_DEG_S;
												printf("%6.1f ",dval);
												dmean+=dval;                               	                                                                   
											}
											printf(" %6.1f",dmean/10.0);     
											printf("\n");  
									break;
									case 'o' : // Motor Current
										printf("\nMotor current: left: %5d  right: %5d\n",auto_data.left_current,auto_data.right_current);
									break;
									case 'p' : // Motor Position
										 printf("\nMotor current: left: %5d  right: %5d\n",auto_data.left_position,auto_data.right_position);
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
							      \n  altitude  : %5d [m]\n",auto_data.gps.valid_sat,auto_data.gps.sat_nb,auto_data.gps.lat_val,auto_data.gps.lat_car,auto_data.gps.long_val,auto_data.gps.long_car,auto_data.gps.date_time.tm_hour,auto_data.gps.date_time.tm_min,auto_data.gps.date_time.tm_sec,auto_data.gps.date_time.tm_mday,auto_data.gps.date_time.tm_mon,auto_data.gps.date_time.tm_year,auto_data.gps.speed,auto_data.gps.altitude);
									break;
									case '$' : // GPS raw data
										printf("\nGPS raw data: %s\n",auto_data.gps_raw);
									break;
									case '@':
										printf("\nMagnetometer: x:%5d  y:%5d  z:%5d [mGauss]\n",auto_data.magne[0],auto_data.magne[1],auto_data.magne[2]);
									break;
									default:
									 printf("\nERROR: received invalid auto mode: %c (0x%x)\n",auto_data.mode,auto_data.mode);
			
								}
					
							} 
							 	
							//printf("\nPush any key to stop\n"); 	
							usleep(10000);
						}
						auto_mon=0;
						koala_configure_auto_monitoring_mode(auto_mon);
						tcflush(0, TCIFLUSH);
					}	
				}	

				
			break;
			case 'b': // braitenberg, battery status
			  if ((strlen(line)>1) && (line[1]=='r')) {
			  	// Braitenberg
			  	braitenbergAvoidance();
			  	tcflush(0, TCIFLUSH);
			  } else
			  {
					// battery status
					while(!koala_kbhit())
					{	
						koala_clrscr();	
						if (koala_get_battery_data(&Bat_Type,&Bat_Voltage,&Bat_Current,&Chrg_Current)<0)
						{
							printf("\nERROR: could not get battery data!\n");
							break;
						}
	 					printf("\nBattery values:\
		        \n  Type              : %s\
		        \n  Voltage           :  %3.1f [V]\
		        \n  Current           : %5d [mA]\
		        \n  Charge current    : %5d [mA]\
		        \n  Push any key to stop\n", 
		        Bat_Type==1?"NiMH":(Bat_Type==0?"Li-ION":"Not Init"),Bat_Voltage/10.0,Bat_Current*100,Chrg_Current*10);
						printf("\nPush any key to stop\n");
						usleep(200000); // wait 200ms
					}
					tcflush(0, TCIFLUSH);
				}
			break;				
			case 'd': // drive mode
				drive_robot();
			break;		
			case 'g': 	// get gps  OR get motor speed ,position	and current	 							
				if ((strlen(line)>2) && (line[1]=='p'))
				{	// get gps	
					while(!koala_kbhit())
					{	
						koala_clrscr();					
						if (koala_gps_data(&valid_sat,&sat_nb,&lat_val,&lat_car,&long_val,&long_car,&date_time,&speed,&altitude)<0)
						{
							printf("\nERROR: could not get GPS data!\n");
							//break;
						}
						printf("GPS data:\n data validity: %s\n  sat number: %d\n  latitude: %5.4f %c\n  longitude: %5.4f %c\n  time: %02d:%02d:%02d\n  date: %02d.%02d.%02d\n  speed: %3.1f [knots]\n  altitude: %d\n\n",valid_sat=='A'?"yes":" no",sat_nb,lat_val,lat_car,long_val,long_car,date_time.tm_hour,date_time.tm_min,date_time.tm_sec,date_time.tm_mday,date_time.tm_mon,date_time.tm_year,speed,altitude);
						printf("\nPush any key to stop\n");
						usleep(200000); // wait 200ms	
					}	
					tcflush(0, TCIFLUSH); // flush input	
				}	else
				if ((strlen(line)>3) && (line[1]=='s'))
				{	// send gps
					if (EOF==sscanf(line,"%*c%*c %s",Buffer))
						printf("\n*** ERROR ***: data not correct\n");
					else
					{
						if (koala_send_gps_cmd(Buffer)<0)
						{
							printf("\nERROR: could not send GPS data!\n");
							break;
						}
					}	
				} 
				else
				{ // // get motor speed ,position	and current	
					while(!koala_kbhit())
					{	
						koala_clrscr();
						if (koala_read_motor_speed(&lspeed,&rspeed)<0)
						{
							printf("\nERROR: could not get motor speed!\n");
							break;
						}		
						if (koala_read_motor_current(&lcur,&rcur)<0)
						{
							printf("\nERROR: could not get motor current!\n");
							break;
						}	
						if (koala_read_motor_position(&lpos,&rpos)<0)
						{
							printf("\nERROR: could not get motor position!\n");
							break;
						}
						printf("motors speed [mm/s (pulse/ %dms)]: left: %7.1f  (%5d)  | right: %7.1f (%5d) \n",KOALA_TIME_BTWN,lspeed*KOALA_SPEED_TO_MM_S,lspeed,rspeed*KOALA_SPEED_TO_MM_S,rspeed);
						printf("motors position [mm (pulse)]: left: %7.1f (%7d) | right: %7.1f (%7d)\n",lpos*KOALA_PULSE_TO_MM,lpos,rpos*KOALA_PULSE_TO_MM,rpos);
						printf("motors current [mA]: left: %4d | right: %4d\n",lcur*100,rcur*100);
						printf("\nPush any key to stop\n");
						usleep(200000); // wait 200ms					
					}
					tcflush(0, TCIFLUSH); // flush input
				}				
			break;
			case 'i': // i2c read,write and scan
				if ((strlen(line)>2) && (line[1]=='r'))
				{ // i2c read
					if (sscanf(line,"%*c%*c %d %d %d",&i2c_add,&i2c_reg,&n)!=3)
						printf("\n*** ERROR ***: the number of data must be an integer\n");
					else
					{
						if (n> KOALA_MAX_I2C_DATA)
						{
							printf("\n*** ERROR ***: the number of data must be less or equal to %d!\n",KOALA_MAX_I2C_DATA);
							break;
						}

						if (koala_read_i2c(i2c_add,i2c_reg,n,i2c_data)<0)
						{
							printf("\nERROR: could not read i2c!\n");
							break;
						}	
				
						printf("\ni2c data:\n");
						for (i=0; i<n;i++)
						{
							printf("  %d: %d\n",i,i2c_data[i]);
						}
					}
				}	else
				if ((strlen(line)>2) && (line[1]=='w'))
				{ // i2c write
					if (sscanf(line,"%*c%*c %d %d %d %s",&i2c_add,&i2c_reg,&n,Buffer)!=4)
						printf("\n*** ERROR ***: incorrect data\n");
					else
					{
						char *pch;
						i=0;
						pch = strtok (Buffer," ");
						while (pch != NULL)
						{
		
							if (sscanf(pch,"%d",&i2c_data[i])!=1)
								break;
		
							pch = strtok (NULL," ");
							i++;
					
							if (i == n)
								break;
								
							if (i > KOALA_MAX_I2C_DATA)
								break;	
						}
						
						if (i!=n)
						{
							printf("\n*** ERROR ***: incorrect number of data: found %d, should be %d !\n",i,n);
							break;
						}
						
						if (koala_write_i2c(i2c_add,i2c_reg,n,i2c_data)<0)
						{
							printf("\nERROR: could not write i2c data!\n");
							break;
						}	
					}	
				}	else
				if ((strlen(line)>1 && (line[1]=='s')))
				{ // i2c scan
					if (koala_scan_i2c(&n,i2c_data)<0)
					{
						printf("\nERROR: could not scan i2c bus!\n");
						break;
					}							   
					printf("\ni2c address: found %d devices\n",n);
					for (i=0; i<n;i++)
					{
						printf(" device %d: i2c %d\n",i,i2c_data[i]);
					}
				} else
				if ((strlen(line)>1 && (line[1]=='o')))
				{		
					while(!koala_kbhit())
					{	
						koala_clrscr();
						if (koala_read_io(&io_state,&in_state)<0)
						{
							printf("\nERROR: could not read io!\n");
							break;
						}	
						printf("IO_0: %d  IO_1: %d  IO_2: %d IO_3: %d  IN_0: %d IN_1: %d \n",io_state&1,(io_state&2)>>1,(io_state&4)>>2,(io_state&8)>>3,in_state&1,(in_state&2)>>1);
						printf("\n\nPush any key to stop\n");
						usleep(200000); // wait 200ms
					}
					tcflush(0, TCIFLUSH); // flush input
				}	
			break;				
			case 'm': // set motors speed profile or speed or position, get motor status
				if ((strlen(line)>2 && (line[1]=='s') && (line[2]=='p')))
				{ // speed profile
					sl=sr=0;
					if (sscanf(line,"%*c%*c%*c %d %d",&sl,&sr)!=2)
						printf("\n*** ERROR ***: the motors speeds must be integers\n");
					else
					{
						printf("\nspeeds to set: %d %d with speed profile\n",sl,sr);	
						if (koala_set_motor_speed_accel(sl,sr)<0)
						{
							printf("\nERROR: could not set motor speed with profile!\n");
							break;
						}
					}
				}	else
				if ((strlen(line)>2 && (line[1]=='s') && (line[2]=='o')))
				{ // speed openloop
					sl=sr=0;
					if (sscanf(line,"%*c%*c%*c %d %d",&sl,&sr)!=2)
						printf("\n*** ERROR ***: the motors speeds must be integers\n");
					else
					{
						printf("\nspeeds to set: %d %d with openloop\n",sl,sr);	
						if (koala_set_motor_speed_open_loop(sl,sr)<0)
						{
							printf("\nERROR: could not set motor speed with openloop!\n");
							break;
						}
					}	
				}	else				
				if ((strlen(line)>2 && (line[1]=='s') && (line[2]=='t')))
				{ // get motor status
					if (koala_get_motor_status(&left_status,&right_status,&left_pos,&right_pos)<0)
					{
						printf("\nERROR: could not get motor status!\n");
						break;
					}
					printf("\nleft motor : control: %s  position: %sreached\n",motor_ctrl_status_str[left_status],left_pos?"":"not ");
					printf("right motor: control: %s  position: %sreached\n",motor_ctrl_status_str[right_status],right_pos?"":"not ");		
				}	else
				
				if ((strlen(line)>2 && (line[1]=='s')))
				{ // speed
					sl=sr=0;
					if (sscanf(line,"%*c%*c %d %d",&sl,&sr)!=2)
						printf("\n*** ERROR ***: the motors speeds must be integers\n");
					else
					{
						printf("\nspeeds to set: %d %d with speed\n",sl,sr);	
						if (koala_set_motor_speed(sl,sr)<0)
						{
							printf("\nERROR: could not set motor speed!\n");
							break;
						}
					}	
				}	
				else
				if ((strlen(line)>1 && (line[1]=='a')))
				{ // magnetometer
					koala_read_magnetometer(magne);
					printf("\nMagnetometer: x:%5d  y:%5d  z:%5d [mGauss]\n",magne[0],magne[1],magne[2]);
				} else
				if ((strlen(line)>1 && (line[1]=='p')))
				{ // position
					sl=sr=0;
					if (sscanf(line,"%*c%*c %d %d",&sl,&sr)!=2)
						printf("\n*** ERROR ***: the motors positions must be integers\n");
					else
					{
						printf("\nposition to set: %d %d with position regulation\n",sl,sr);	
						if (koala_set_motor_target_position(sl,sr)<0)
						{
							printf("\nERROR: could not set motor target position!\n");
							break;
						}
					}	
				}
			break;
			case 'p':  //pid,position, set power io
				if ((strlen(line)>2 && (line[1]=='i')))
				{ // pid
					if (sscanf(line,"%*c%*c %d %d %d",&kp,&ki,&kd)==3)
					{
						if (koala_configure_pid(kp,ki,kd)<0)
						{
							printf("\nERROR: could not configure pid!\n");
							break;
						}
					}
				}
				else
				if ((strlen(line)>2 && (line[1]=='a')))
				{ 
					// us and io parameters
					if (sscanf(line,"%*c%*c %d %d",&us_mask,&io_dir)==2)
					{
						if (koala_configure_us_io(us_mask,io_dir)<0)
						{
							printf("\nERROR: could not configure us and io parameters!\n");
							break;
						}	
					}
				} else
				if ((strlen(line)>2 && (line[1]=='o')))
				{ 
					// set power io and io
					if (sscanf(line,"%*c%*c %d %d %d %d %d",&power_out,&IO0,&IO1,&IO2,&IO3)==5)
					{						
						if (koala_set_pwr_io_output(power_out,IO0,IO1,IO2,IO3)<0)
						{
							printf("\nERROR: could not set power io and io!\n");
							break;
						}		
					}
				}					
			break;	
			case 'q': // quit
				quitReq=1;
				break;
			case 'r': // reset  encoders the microcontroller
				if ((strlen(line)>1 && (line[1]=='e')))
				{ //or encoders					
					if (koala_set_position_encoders(0,0)<0)
					{
						printf("\nERROR: could not reset encoders!\n");
						break;
					}	
				}
				else
					if ((strlen(line)>1 && (line[1]=='m')))
					{// reset the microcontroller 
						if (koala_reset_microcontroller()<0)
						{
							printf("\nERROR: could not reset microcontroller!\n");
							break;
						}	
						printf("\nWait for microcontroller reset...\n");
						sleep(3); // wait for Koala to restart
						koala_getcommand_line_nowait(Buffer) ; // read any remaining line; usualy Koala version
						printf("\nMicrocontroller reset OK (%s) !\n",Buffer);
					}	
			break;
			case 's': // speed profile or stop motor 			
				if ((strlen(line)>2 && (line[1]=='p')))
				{ // speed profile
					if (sscanf(line,"%*c%*c %d %d %d %d %d %d",&acc_inc,&acc_div,&min_spacc, &cst_speed,&pos_margin,&max_current)==6)
					{	
						if (koala_set_speed_profile(acc_inc,acc_div,min_spacc,cst_speed,pos_margin,max_current)<0)
						{
							printf("\nERROR: could not set speed profile!\n");
							break;
						}
					}
				}
				else
				if ((strlen(line)>2 && (line[1]=='o')))
				{
					test_sound();
				}
				else
				{
					//stop motor
					if (koala_set_motor_speed(0,0)<0)
					{
						printf("\nERROR: could not set stop motor!\n");
						break;
					}
				}		
			break;
			case 'u': // us sensors
				 // display us
					while(!koala_kbhit())
					{	
						koala_clrscr();
		
						// get and print us sensors
						if (koala_read_us_sensors(us_sensors)<0)
						{
							printf("\nERROR: could notread us sensors!\n");
							break;
						}

						printf("\nUS sensors : distance [cm]\
										\n                        50  100  150  200\
										\n                  0|    .    :    .    :   |%.0f\n",MAX_US_DISTANCE);
				 		for (i=0;i<KOALA_US_SENSORS_NUMBER;i++)
				 		{  
				 			
							if((us_sensors[i] == KOALA_US_DISABLED_SENSOR) || (us_sensors[i] == KOALA_US_NO_OBJECT_IN_RANGE))
				 			{ // out of range or disabled
								sprintf(bar[i],"|\33[%dC|",US_BAR_LEN);	
							}  else
							{
								// in range or less than 25cm
								n=(int)(us_sensors[i]*US_BAR_LEN/MAX_US_DISTANCE);
																				        
								if (n==0)
									sprintf(bar[i],"|>\33[%dC|",US_BAR_LEN-1);
								else
									if (n>=US_BAR_LEN-1)
										sprintf(bar[i],"|\33[%dC>|",US_BAR_LEN-1);
									else
									 sprintf(bar[i],"|\33[%dC>\33[%dC|",n,US_BAR_LEN-1-n); 
								
								
							}  
							printf("%11s : %4d %s  %s\n",KOALA_US_SENSOR_NAMES[i],us_sensors[i],bar[i],US_VAL(us_sensors[i]));	                               
				 		}				 		
						printf("\nPush any key to end program\n");
						usleep(200000); // wait 200ms
					}
					tcflush(0, TCIFLUSH); // flush input
			break;
			case ' ':
					if (koala_set_motor_speed(0,0)<0)
					{
						printf("\nERROR: could not set stop motor!\n");
						break;
					}
			break;
			default: 
				printf("\n*** ERROR ***: option %s (first char: 0x%02x) is undefined!\n\n",line,line[0]);		
		}

  } // while
  
  koala_set_motor_speed_open_loop(0 ,0); // stop robot and release motors
  
            
	return 0;
}
