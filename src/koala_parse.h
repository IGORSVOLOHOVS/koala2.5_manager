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
/*!   \file koala_parse.h
      \brief Parsing of the fields in an answer of the Koala robot
*/
////////////////////////////////////////////////////////////////////////////////

#ifndef KOALA_PARSE_H
#define KOALA_PARSE_H

#ifdef __cplusplus
extern "C" {
#endif

#define KOALA_DELIM ","  // delimiter for parameters

/*! process integer parameters from a string into a integer array
 *
 * \param source array of char containing  input parameters sperated by DELIM; strtok writes into it
 * \param destination array of integer containing parameters
 * \param nb_arg number of parameters found; it must be at least 1, or nothing bounds the writes
 *
 * \return A value:
 *       - <0 on error
 *       - 0 on success
 *
 */
int process_param(char *source, int *destination, int nb_arg);

#ifdef __cplusplus
}
#endif

#endif /* KOALA_PARSE_H */
