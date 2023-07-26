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

/*! 
 * \file   koala_serial.c RS-232 Communication            
 *
 * \brief 
 *         This module is the low-level communication layer between   
 *         the Koala Library and the Linux Operating System
 *        
 * \author   Julien Tharin (K-Team SA)                               
 *
 * \note     Copyright (C) 2013 K-TEAM SA
 */

#include "koala.h"
#include "koala_serial.h"


#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termio.h>

#define LineLength 256

/*--------------------------------------------------------------------*/
/*! 
 * This function opens a given device on the RS-232 Bus
 *
 * \param name   RS-232 device name
 * \param baudrate  baudrate (B115200,...) 
 *
 * \return A Pointer to a RS-232 device descriptor or 
 *         NULL in case of error.
 *
 * \remark This function is NOT exported outside this module.
 */
koala_rs232_t * koala_rs232_open( 
					    const char * name, int baudrate)
{
  koala_rs232_t * rs232;
  int fd;
 
  if ((fd=open(name,O_RDWR | O_NOCTTY | O_NONBLOCK))<0) {
    return NULL;
  }

  if (baudrate==0)
	{
		baudrate = B115200;
	}

  rs232 = malloc(sizeof(koala_rs232_t));

  fcntl(fd, F_SETFL, 0); // blocking input

  rs232->fd = fd;

  tcgetattr(fd , &rs232->tios);
  rs232->tios.c_cflag = ( CS8 | CLOCAL | CREAD | baudrate );
  rs232->tios.c_iflag = IGNPAR;
  rs232->tios.c_oflag = 0;
  //rs232->tios.c_lflag =  ICANNON;
  rs232->tios.c_lflag = 0;
  rs232->tios.c_cc[VMIN] = 1;
  rs232->tios.c_cc[VTIME] = 1;
  tcflush(fd, TCIFLUSH);
  tcsetattr( fd , TCSANOW , &rs232->tios );
  
  
  return (rs232);
}

/*--------------------------------------------------------------------*/
/*! 
 * This function closes an RS-232 device.
 *
 * \param rs232  RS-232 device descriptor
 *
 */
void koala_rs232_close(koala_rs232_t * rs232 )
{

  if ( rs232 != NULL ) {

    if ( rs232->fd != -1 ) 
      close(rs232->fd );
   
     free( rs232 );
  }
}

/*--------------------------------------------------------------------*/
/*! 
 * This function reads data from an RS-232 device.
 *
 * \param rs232 RS-232 device descriptor
 * \param buf Pointer to the buffer that will receive the data
 * \param len Size of the buffer
 *
 * \return A value:
 *        - <0 on error
 *        - >=0 on success, number of bytes read
 * 
 * \remark This function is NOT exported outside this module.
 */
int koala_rs232_read(koala_rs232_t * rs232,
			    char * buf , unsigned int len )
{
  if ( rs232 != NULL && rs232->fd != -1 )
    return read( rs232->fd , buf , len );

  return -1;
}


/*--------------------------------------------------------------------*/
/*
/*!
 * Read data (Reply) until the termination
 *
 *  \param rs232 RS-232 device descriptor
 *  \param buffer read data
 *
 *  \return return the number of the count of the read data
*/
int koala_rs232_readLine_nowait(koala_rs232_t * rs232, char *buffer) {

  int i;
  struct timeval Timeout;
  int res; 
  fd_set rfds;
  
  
  if ( rs232 == NULL || rs232->fd == -1 )
    return -1;
  
  	FD_ZERO(&rfds);
		FD_SET(rs232->fd, &rfds);
		                                                                          
	 // set timeout value                          
		Timeout.tv_usec = 0;  // microseconds                           
		Timeout.tv_sec  = 0;  // seconds                               
		res = select(rs232->fd+1, &rfds, NULL, NULL, &Timeout);

  if (res<=0) // error or timeout
		 return -2;
  
  for (i = 0; i < LineLength -1; ++i) {
    char recv_ch;
    
	              

		
    
    int n = read(rs232->fd, &recv_ch, 1);// com_recv(&recv_ch, 1, Timeout);
    if (n <= 0) {
      if (i == 0) {
				return -3;		// timeout
      }
      break;
    }
    if ((recv_ch == '\r') || (recv_ch == '\n')) {
      break;
    }
    buffer[i] = recv_ch;
  }
  buffer[i] = '\0';

	
	
  return i;
}

/*--------------------------------------------------------------------*/
/*
/*!
 * Read data (Reply) until the termination with timeout
 *
 *  \param rs232 RS-232 device descriptor
 *  \param buffer read data
 *
 *  \return return the number of the count of the read data
*/
int koala_rs232_readLine(koala_rs232_t * rs232, char *buffer) {

  int i;
  struct timeval Timeout;
  int res; 
  fd_set rfds;
  
  
  if ( rs232 == NULL || rs232->fd == -1 )
    return -1;
  
  	FD_ZERO(&rfds);
		FD_SET(rs232->fd, &rfds);
		                                                                          
	 // set timeout value                          
		Timeout.tv_usec = 500000;  // microseconds                           
		Timeout.tv_sec  = 0;  // seconds                               
		res = select(rs232->fd+1, &rfds, NULL, NULL, &Timeout);

  if (res<=0) // error or timeout
		 return -2;
  
  
  for (i = 0; i < LineLength -1; ++i) {
    char recv_ch;
    
	   
    int n = read(rs232->fd, &recv_ch, 1);// com_recv(&recv_ch, 1, Timeout);
    if (n <= 0) {
      if (i == 0) {
				return -3;		// timeout
      }
      break;
    }
    if ((recv_ch == '\r') || (recv_ch == '\n')) {
      break;
    }
    buffer[i] = recv_ch;
  }
  buffer[i] = '\0';

	
	
  return i;
}

/*--------------------------------------------------------------------*/
/*! 
 * This function writes data to an RS-232 device. 
 *
 * \param rs232 RS-232 device descriptor
 * \param buf Pointer to the buffer that contains the data to be 
 *            written
 * \param len Number of the bytes in the buffer
 *
 * \return A value:
 *        - <0 on error
 *        - >=0 on success, number of byte written
 *
 * \remark This function is NOT exported outside this module.
 */
int koala_rs232_write(koala_rs232_t * rs232 ,
			   const char * buf , unsigned int len )
{
  if ( rs232 != NULL && rs232->fd != -1 )
    return write( rs232->fd , buf , len );

  return -1;
}
