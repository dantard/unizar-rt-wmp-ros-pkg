/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *
 *
 *
 *  File: ./src/platforms/linux_ks/config/compiler.c
 *  Authors: Rubén Durán
 *           Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2011, Universidad de Zaragoza, SPAIN
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

#include "compiler.h"
#include <asm/uaccess.h>

void inline atof(const char *cadena, double *res){
   int entero, decimal, decimales;
   char *next;
   entero = (int) simple_strtol(cadena,&next,10);
   decimales = strlen(next+1);
   decimal = (int) simple_strtol(next+1,&next,10);

/* kernel_fpu_begin();
   *res = decimal;
   while (decimales > 0)
   {
      *res = *res/10;
      decimales--;
   }
   *res += entero;
   kernel_fpu_end();
*/
}

char * fgets(char *s, int size, struct file *f){
   mm_segment_t oldfs;
   int i, leido;

   if (size > 0)
   {
      oldfs = get_fs();
      set_fs(get_ds());

      i = 0;
      do {
         leido=vfs_read(f,&s[i],1, &f->f_pos);
         i++;
      }while(i<size-1 && leido && s[i-1] !='\n');
      // Leemos letra a letra hasta encontrar el fin del fichero, un salto de
      // linea o alcanzar 'size'

      set_fs(oldfs);


      if(!leido && i==1)
         return NULL;
      else if (i == size-1)
         s[i]='\0';
      else
         s[i-1]='\0';

      return s;
   }
   return NULL;
}
EXPORT_SYMBOL(fgets);
