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
/*!   \file koala_utils.c
      \brief Useful functions of the libkoala
*/
////////////////////////////////////////////////////////////////////////////////


/* ---- Include Files ---------------------------------------------------- */

#include <stdio.h>
#include <termio.h>
#include <unistd.h>

#include "koala_utils.h"

/* ---- Exported Functions------------------------------------------------ */

/*--------------------------------------------------------------------*/
/*! Test if anykey was pushed
 * 
 *  
 *  \return -1 if error occured
 *		      >=0  number of characters to read
 */
int koala_kbhit(void)
{
  int cnt = 0;
  int error;
  static struct termios Otty, Ntty;


  tcgetattr( 0, &Otty);
  Ntty = Otty;

  Ntty.c_iflag          = 0;       /* input mode                */
  Ntty.c_oflag          = 0;       /* output mode               */
  Ntty.c_lflag         &= ~ICANON; /* raw mode */
  Ntty.c_cc[VMIN]       = CMIN;    /* minimum time to wait      */
  Ntty.c_cc[VTIME]      = CTIME;   /* minimum characters to wait for */

  if (0 == (error = tcsetattr(0, TCSANOW, &Ntty))) {
    error += ioctl(0, FIONREAD, &cnt);
    error += tcsetattr(0, TCSANOW, &Otty);
  }

  return ( error == 0 ? cnt : -1 );
}


/*--------------------------------------------------------------------*/
/*! Change terminal mode for getchar to return immediately
 * 
 *  \param dir 1= mode changed to non-blocking, 0 mode reverted to previous
 *
 *  \return none
 *		     
 */
void koala_change_term_mode(int dir)
{
  static struct termios oldt, newt;
  static int called=0; // avoid to change if not called

  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
    called=1;
  }
  else
    if (called)
    {
    	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
    	called=0;
    }	
}


/*--------------------------------------------------------------------*/
/*! Clear the console screen
 * 
 */
void koala_clrscr(void)
{
  printf("\033[2J");            /* erase the whole console */
  printf("\033[1;1f");          /* Move cursor to the top left */
}



/*--------------------------------------------------------------------*/
/*! erase line (origin at 1,1)
 * 
 *  \param line line
 */
void koala_erase_line(int line)
{
	koala_move_cursor(1,line);
	printf("\033[K"); //erase to end of line
}

/*--------------------------------------------------------------------*/
/*! move cursor (origin at 1,1)
 * 
 *  \param c column
 *	\param l line
 */
void koala_move_cursor(int c, int l) // column,line
{
	printf("\033[%d;%df",l,c); 
}

/*--------------------------------------------------------------------*/
/*! move cursor column (origin at 1,1), keep line
 * 
 *  \param c column
 */
void koala_move_cursor_column(int c) // column
{
	printf("\033[%d`",c);             // Move the cursor to column c.
}

/*--------------------------------------------------------------------*/
/*! move cursor line (origin at 1,1), keep column
 * 
 *  \param l line
 */
void koala_move_cursor_line(int l) // line
{
	printf("\033[%dd",l);             // Move the cursor line l
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
koala_timeval_diff(struct timeval *difference,
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
