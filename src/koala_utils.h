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
/*!   \file koala_utils.h
      \brief Header of Useful functions of libkoala
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef __koala_utils__
#define __koala_utils__

#include <time.h>
#include <sys/time.h>

/* ---- Constants and Types ---------------------------------------------- */


/* ---- Function Prototypes ---------------------------------------------- */

extern void koala_change_term_mode(int dir);

extern int koala_kbhit(void);

extern void koala_clrscr(void);

extern void koala_move_cursor(int c, int l);

extern void koala_move_cursor_column(int c);

extern void koala_move_cursor_line(int l);

extern void koala_erase_line(int line);

extern long long
koala_timeval_diff(struct timeval *difference,
             struct timeval *end_time,
             struct timeval *start_time
            );


#endif // #ifndef __koala_utils__

