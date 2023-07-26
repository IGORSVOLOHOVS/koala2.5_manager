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
/*!   \file koala_serial.h 
      \brief RS-232 Communication Layer
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef __koala_serial__
#define __koala_serial__

#include <termio.h>

typedef struct koala_rs232_s {
  
  /*! RS-232 device file descriptor */
  int fd;

  /*! */
  struct termios tios;

}
koala_rs232_t;


extern koala_rs232_t * koala_rs232_open( 
					    const char * name, int baudrate);
					    
extern void koala_rs232_close(koala_rs232_t * rs232 );

extern int koala_rs232_read(koala_rs232_t * rs232,
			    char * buf , unsigned int len );
			   
			   
extern int koala_rs232_readLine_nowait(koala_rs232_t * rs232, char *buffer);
extern int koala_rs232_readLine(koala_rs232_t * rs232, char *buffer);
			    
extern int koala_rs232_write(koala_rs232_t * rs232 ,
			   const char * buf , unsigned int len );			
			   
			       

#endif /* __koala_serial__ */
