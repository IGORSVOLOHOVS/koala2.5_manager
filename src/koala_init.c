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
/*!   \file koala_init.c
      \brief init library of the Koala extension board
*/
////////////////////////////////////////////////////////////////////////////////

/* ---- Include Files ---------------------------------------------------- */

#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>

#include "koala.h"


/* ---- Private Constants and Types -------------------------------------- */
static int koala_init_done = 0;

// long options: --version , --debug
struct option opts[] = {   
  { "debug" ,   0 , 0 , 'd' } ,
  { "version" , 0 , 0 , 'v' } ,
  { NULL      ,    0 , 0 , 0    }
};


char short_opts[] = "d:v"; // -v  -d

/* ---- Internal Functions ----------------------------------------------- */



/* ---- Exported Functions------------------------------------------------ */

/*--------------------------------------------------------------------*/
/*! This function initializes the koala Library
 *
 *  \param argc number of arguments
 *  \param argv pointer table of arguments
 *	\return A value
 *             0 : Ok
 *						 1 : already done
 *						<0 : error	
 */
int koala_init( int argc , char * argv[] )
{
  int opt, rc;

  if ( koala_init_done == 0 ) {

    for (;;) {
      opt = getopt_long( argc , argv , short_opts , opts , NULL );
      if ( opt == -1 )
        break;
			
      switch( opt ) {
			
				case 'd': // debug
					break;

        /* --koala-version */
        case 'v':
      	   printf("libkoala v%u.%u [%s]\n" ,KOALA_VERSION ,KOALA_REVISION , __DATE__ );
          break;

        default:
          break;
      }
    }

		if (koala_robot_init()!=0)
		{
			return -1;
		} 

		
    atexit( koala_exit );

    koala_init_done = 1;
    
    return 0;
  }

  return 1;
}

/*--------------------------------------------------------------------*/
/*! This function is called automatically on exit and call the 'exit'
 *  of all sub layer.
 *
 * \remark This function is called automatically at the terminaison
 *         of the application.
 */
void koala_exit(void)
{
  if ( koala_init_done ) {

    koala_init_done = 0;
    
    koala_robot_release();
    
  }
}
