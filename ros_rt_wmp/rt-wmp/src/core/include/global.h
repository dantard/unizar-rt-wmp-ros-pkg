/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/include/global.h                                     
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



#ifndef GLOBAL_H_
#define GLOBAL_H_
#include "config/compiler.h"
#include "definitions.h"

extern Status status;

#define mBits(x,m) ((x)&(m))
#define mBitsExcept(x,m,be) (((x)&(m))& ~(be))
#define mTestBits(x,m,b) (mBits(x,m)==(b)))
#define mTestBitsExcept(x,m,b,be) (mBitsExcept(x,m,be)==(b))

#define mBitsOn(lvx,by) ((lvx)|=(by))
#define mBitsOnExcept(lvx,by,be) ((lvx)|=(by)&~(be))
#define mBitsOff(lvx,bn) ((lvx)&= ~(bn))
#define mBitsOffExcept(lvx,bn,be) ((lvx)&= ~((bn)&~(be)))
#define mBitsToggle(lvx,b) ((lvx)^= (b))
#define mBitsToggleExcept(lvx,b,be) ((lvx)^= ((b)&~(be)))

#define mBitsOnOff(lvx,by,bn) {mBitsOff(lvx,bn);mBitsOn(lvx,by);}
#define mBitsOnOffExcept(lvx,by,bn,be) {mBitsOffExcept(lvx,bn,be);mBitsOnExcept(lvx,by,be);}

#define mBitsSet(x,n) mBitsOn(x, (int) (1 << n) )
#define mBitsUnset(x,n) mBitsOff(x, (int) (1 << n) )
#define mBitsTogg(x,n) mBitsToggle(x, (int) (1 << n) )

#define mBitsIsSet(x,n) mBits(x, (int) (1 << n) )


#endif /*GLOBAL_H_*/

