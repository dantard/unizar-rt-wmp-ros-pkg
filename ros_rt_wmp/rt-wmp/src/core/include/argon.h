/*------------------------------------------------------------------------
 *---------------------           ARGON               --------------------
 *------------------------------------------------------------------------
 *                                                         V0.1B  15/09/10
 *
 *
 *  File: argon.h
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

#ifndef ARGON_H_
#define ARGON_H_

#define ARGO_MAX_SWITCH 10

int argo_addInt(int * var, char* sw, int dfl, int need_value);
int argo_addDouble(double * var, char* sw, double dfl);
int argo_addString(char * var, char* sw, char * dfl);
void argo_setComment(char * sw, char * text);
void argo_setCommentId(int id, char * text);
void argo_doProcess(int argc, char * argv[], int start);

/* CODE >>>>>>> */

#include "config/compiler.h"

#define ARGON_INT 0
#define ARGON_STRING  1
#define ARGON_DOUBLE  2

typedef struct {
	char sw[16];
	char type;
	void * var;
	int need_value;
	char comment[64];
} argo_data_t;

static struct {
	argo_data_t at[ARGO_MAX_SWITCH];
	char idx;
} regs;

int argo_addInt(int * var, char* sw, int dfl, int need_value) {
	*var = dfl;
	regs.at[(int)regs.idx].var = (void*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_INT;
	regs.at[(int)regs.idx].need_value = need_value;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type int", sw);
	regs.idx++;
	return regs.idx - 1;
}

int argo_addDouble(double * var, char* sw, double dfl) {
	*var = dfl;
	regs.at[(int)regs.idx].var = (double*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_DOUBLE;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type double", sw);
	regs.idx++;
	return regs.idx - 1;
}

int argo_addString(char * var, char* sw, char * dfl) {
	strcpy(var, dfl);
	regs.at[(int)regs.idx].var = (char*) var;
	strcpy(regs.at[(int)regs.idx].sw, sw);
	regs.at[(int)regs.idx].type = ARGON_STRING;
	sprintf(regs.at[(int)regs.idx].comment, "--%s: type string", sw);
	regs.idx++;
	return regs.idx - 1;
}

void argo_setComment(char * sw, char * text) {
	int i;
	for (i = 0; i < regs.idx; i++) {
		if (strcmp(sw, regs.at[i].sw) == 0) {
			sprintf(regs.at[i].comment, "-%s: %s", sw, text);
		}
	}
}

void argo_setCommentId(int id, char * text){
	sprintf(regs.at[id].comment, "--%s: %s", regs.at[id].sw, text);
}

void argo_doProcess(int argc, char * argv[], int start) {
	int i, j;
   int done;
	int error = 0;
	for (i = start + 1; i < argc; i++) {

		/* switch format */
		if (argv[i][0] != '-' || (argv[i][1] != '-')) {
			continue;
		}

		/* switch existence */
		done = 0;
		for (j = 0; j < regs.idx; j++) {
			if (strcmp(&argv[i][2], "h") == 0) {
				error = 1;
				done = 1;
				break;
			}

			if (strcmp(&argv[i][2], regs.at[j].sw) == 0) {
				done = 1;
				if (regs.at[j].type == ARGON_INT) {
					if (regs.at[j].need_value) {
						if (i == argc - 1) {
							error = 4;
							break;
						} else {
							*((int *) regs.at[j].var) = atoi(argv[i + 1]);
							i++;
						}
					} else {
						*((int *) regs.at[j].var) = 1;
					}
				} else if (regs.at[j].type == ARGON_DOUBLE) {
					if (i == argc - 1) {
						error = 4;
						break;
					} else {
                  ATOF(argv[i + 1], ((double *) regs.at[j].var));
						i++;
					}
				} else if (regs.at[j].type == ARGON_STRING) {
					if (i == argc - 1) {
						error = 4;
						break;
					} else {
						strcpy(((char *) regs.at[j].var), argv[i + 1]);
						i++;
					}
				}
			}
		}

		if (!done) {
         WMP_ERROR(stderr, "*** Inexistent switch %s specified\n",argv[i]);
			error = 3;
			break;
		}

	}
	if (error == 4) {
      WMP_ERROR(stderr, "*** Incorrect number of parameters\n");
	}
	if (error != 0) {
      WMP_ERROR(stderr, "List of switch:\n");
		for (i = 0; i < regs.idx; i++) {
         WMP_ERROR(stderr, "%s\n", regs.at[i].comment);
		}
#ifndef __KERNEL__
		exit(0);
#else
      return;  // .................... ???????????????
#endif
	}
}
#endif /* ARGON_H_ */
