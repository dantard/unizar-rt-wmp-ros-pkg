/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/lqm.h                                        
 *  Authors: Danilo Tardioli                                              
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2010, Universidad de Zaragoza, SPAIN
 *
 *  Contact Addresses: Danilo Tardioli                   dantard@unizar.es
 *
 *  RT-WMP is free software; you can  redistribute it and/or  modify it
 *  under the terms of the GNU General Public License  as published by the
 *  Free Software Foundation;  either  version 2, or (at  your option) any
 *  later version.
 *
 *  RT-WMP  is distributed  in the  hope  that  it will be   useful, but
 *  WITHOUT  ANY  WARRANTY;     without  even the   implied   warranty  of
 *  MERCHANTABILITY  or  FITNESS FOR A  PARTICULAR PURPOSE.    See the GNU
 *  General Public License for more details.
 *
 *  You should have received  a  copy of  the  GNU General Public  License
 *  distributed with RT-WMP;  see file COPYING.   If not,  write to the
 *  Free Software  Foundation,  59 Temple Place  -  Suite 330,  Boston, MA
 *  02111-1307, USA.
 *
 *  As a  special exception, if you  link this  unit  with other  files to
 *  produce an   executable,   this unit  does  not  by  itself cause  the
 *  resulting executable to be covered by the  GNU General Public License.
 *  This exception does  not however invalidate  any other reasons why the
 *  executable file might be covered by the GNU Public License.
 *
 *----------------------------------------------------------------------*/



#ifndef LQM_H_
#define LQM_H_
#include "config/compiler.h"
void init_lqm(int m_size);
void free_lqm(void);
void fill_lqm(char val);
//char lqm_get_val(int i, int j);
void lqm_set_val(int i, int j, char val);
char **lqm_get_ptr(void);
void lqm_calculate_distances(void);
int lqm_get_num_hops(int i, int j);
int lqm_is_leaf(char id);
int lqm_get_distance(char i, char j);

void lqm_backup(void);
void lqm_restore(void);
char f_lqm(char val);
char ** lqm_prune(char ** lqm);
void lqm_copy_to(char ** src, char ** dest);

char (*lqm_get_f(void))(char);
void lqm_set_f( char (*f) (char));

#endif /*LQM_H_*/

