/*
 * aura.h
 *
 *  Created on: Sep 22, 2010
 *      Author: danilo
 */

#ifndef AURA_H_
#define AURA_H_

#include "config/compiler.h"
#include "core/include/aura.h"
#include "core/include/frames.h"
#include "core/include/global.h"
#include "core/include/dijkstra.h"
#include "core/include/lqm.h"


///MERGING AURA FULL=3
typedef enum {
	aura_auth=1, aura_msg=2, aura_full = 13
} aura_t;


void aura_store_msg(Message * m);
void aura_restore_msg(Message * m);
void aura_add(aura_t val, int id);
void aura_clear(void);
int aura_get(int id);
void aura_discard_unnecessary(int dest);
int aura_get_next(wmpFrame * p, aura_t * type);

#endif /* AURA_H_ */
