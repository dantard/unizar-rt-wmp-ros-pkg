/*-------------------------------------------------------------------------
 *--------------------------- RT-WMP IP INTERFACE -------------------------
 *-------------------------------------------------------------------------
 *
 * File: conf.h
 * Authors: Rubén Durán
 *          Danilo Tardioli
 *-------------------------------------------------------------------------
 *  Copyright (C) 2011, Universidad de Zaragoza, SPAIN
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *-----------------------------------------------------------------------*/

#ifndef CSD
#define CSD

typedef struct {
	unsigned char src;
	unsigned int dest;
	signed char priority;
	unsigned int size;
	unsigned int port;
	long long timeout;
	int type,step,id;
	char * data;
} cross_space_data_t;

#endif
