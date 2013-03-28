/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: enhanced_io.cc
 *  Authors: Danilo Tardioli
 *  ----------------------------------------------------------------------
 *  Copyright (C) 2000-2012, Universidad de Zaragoza, SPAIN
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
static char buff[2500];
#include "enhanced_io.h"
#include "core/interface/Msg.h"
#include <cstring>

int read_next_token(char * msg, char* bc_msg, int *serial, int * pos, const char * txt) {
    int res = io_read_next_sim_data(buff);
    if (res > 0) {
        wmpFrame * r = (wmpFrame *) (buff + sizeof (simData_Hdr));
        if (r->hdr.type == TOKEN) {
#ifdef	ENABLE_BC_SUPPORT
        	if (r->hdr.bc_len > 0) {
                if (r->hdr.bc_type == BC_TYPE_PLUS) {
                    Simple_bc * sbc = (Simple_bc*) wmp_get_frame_tail_pointer(r, io_get_num_of_nodes());
                    if (strstr(&sbc->data, txt) || strcmp(txt,"")==0) {
                    	*serial = r->hdr.serial;
                        *pos = io_get_filepos() - 1;
                        memcpy(bc_msg, &sbc->data, sbc->len);
                        return 1;
                    }
                }
            }else
#endif
            {
            	*serial = r->hdr.serial;
                *pos = io_get_filepos() - 1;
                sprintf(bc_msg,"%s","");
            }
            return 1;
        } else return 0;
    }
    return -1;
}




#include <sstream>
int read_next_message2(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt) {
    int res = io_read_next_sim_data(buff);
    if (res > 0) {
        int ret = 0;
        return ret;
    }
    return -1;
}

#include <cstdlib>
int read_next_message(char * tmp, char * bc_msg, int *serial, int * pos,
        const char * txt) {
    int res = io_read_next_sim_data(buff);
    if (res > 0) {
        int ret = 0;
        wmpFrame * r = (wmpFrame *) (buff + sizeof (simData_Hdr));
        if (r->hdr.type == MESSAGE) {
            if (atoi(txt) == r->msg.msg_hash || strstr(wmp_get_message_data_pointer(r), txt) || strcmp(txt,"")==0 ) {
                *serial = r->hdr.serial;
                *pos = io_get_filepos() - 1;
                memcpy(tmp, wmp_get_message_data_pointer(r), r->msg.len);
                sprintf(bc_msg,"%s","");
                ret = 1;
            }
#ifdef	ENABLE_BC_SUPPORT
            if (r->hdr.bc_len > 0) {
                if (r->hdr.bc_type == BC_TYPE_PLUS) {
                    Simple_bc * sbc = (Simple_bc*) wmp_get_frame_tail_pointer(r, io_get_num_of_nodes());
                    if (strstr(&sbc->data, txt)) {
                        *serial = r->hdr.serial;
                        *pos = io_get_filepos() - 1;
                        memcpy(bc_msg, &sbc->data, sbc->len);
                        ret = 1;
                    }
                }
            }
#endif
        }
        return ret;
    }
    return -1;
}

int read_next_bc(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt) {
    int res = io_read_next_sim_data(buff);
    if (res > 0) {
		tmp[0] = 0;
		bc_msg[0] = 0;
		int ret = 0;
		wmpFrame * r = (wmpFrame *) (buff + sizeof(simData_Hdr));
		*serial = r->hdr.serial;
		*pos =  - 1;
		int must_return=0;
		if (strcmp(txt,"")==0){
			must_return=1;
		}
#ifdef	ENABLE_BC_SUPPORT
		if (r->hdr.bc_len > 0 && r->hdr.type != DROP_TOKEN) {
			if (r->hdr.bc_type == BC_TYPE_PLUS) {
				Simple_bc * sbc = (Simple_bc*) wmp_get_frame_tail_pointer(r,
						io_get_num_of_nodes());
				if (strstr(&sbc->data, txt) || must_return) {
					memcpy(bc_msg, &sbc->data, sbc->len);
					ret++;
				}
			}
		} else
#endif
		{
			sprintf(bc_msg, "%s","");
		}
        if (r->hdr.type == MESSAGE){
        	if (strstr(wmp_get_message_data_pointer(r), txt) || must_return) {
        		memcpy(tmp, wmp_get_message_data_pointer(r), r->msg.len);
        		ret++;
        	}
        }else{
        	sprintf(tmp,"%s","");
        }
        if (must_return || ret>0 ) return 1;
        else return 0;
    }
    return -1;
}

int read_next_drop(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt) {
    int res = io_read_next_sim_data(buff);
    if (res > 0) {
        wmpFrame * r = (wmpFrame *) (buff + sizeof (simData_Hdr));
        if (r->hdr.type == DROP_TOKEN) {
            *serial = r->hdr.serial;
            *pos = io_get_filepos() - 1;
            return 1;
        } else return 0;
    }
    return -1;

}

int read_next_authorization(char * tmp, char * bc_msg, int *serial, int * pos, const char * txt) {
    int res = io_read_next_sim_data(buff);
    if (res > 0) {
        wmpFrame * r = (wmpFrame *) (buff + sizeof (simData_Hdr));
        if (r->hdr.type == AUTHORIZATION) {
#ifdef	ENABLE_BC_SUPPORT
            if (r->hdr.bc_len > 0) {
                if (r->hdr.bc_type == BC_TYPE_PLUS) {
                    Simple_bc * sbc = (Simple_bc*) wmp_get_frame_tail_pointer(r, io_get_num_of_nodes());
                    if (strstr(&sbc->data, txt) || strcmp(txt,"")==0) {
                        *serial = r->hdr.serial;
                        *pos = io_get_filepos() - 1;
                        memcpy(bc_msg, &sbc->data, sbc->len);
                        return 1;
                    }
                }

            }else
#endif
            {

            	*serial = r->hdr.serial;
            	*pos = io_get_filepos() - 1;
            	sprintf(bc_msg,"%s","");
            }
            return 1;
        } else return 0;
    }
    return -1;
}



static FILE * outf[32];
static int nnodes;

void logger_init(int type, int nnodes_){
	nnodes = nnodes_;
	for (int i=0; i< nnodes ; i++){
		outf[i] = 0;
	}
	if (type == 0){
		for (int i=0; i< nnodes ; i++){
			outf[i] = stderr;
		}
	}else{
		for (int i=0; i< nnodes ; i++){
			char filename[256];
			sprintf(filename,"rt-wmp-node-%d.log",i);
			outf[i] = fopen(filename,"w+");
		}
	}
}

void logger_printData(int node_id, wmpFrame *q, unsigned long long ts) {
	int i, j;
	double evtime;

	evtime = (double)ts/1e6;

	fprintf(outf[node_id], "%5.5f\t%d\t%d\t%d\t%d\t%d\t%d",evtime,(int) q->hdr.serial, q->hdr.rssi,
			q->hdr.retries, q->hdr.from, q->hdr.to, q->hdr.type);

	char * lqm = wmp_get_frame_routing_pointer(q,nnodes);
	for (i = 0; i < nnodes; i++) {
		for (j = 0; j < nnodes; j++) {
			if (i != j) {
				fprintf(outf[node_id], "\t%d",  *lqm);
			} else {
				fprintf(outf[node_id], "\t%d", 0);
			}
			lqm ++;
		}
	}
#ifdef ENABLE_WMP_DEBUG
	for (i = 0; i < 32; i++) {
		fprintf(outf[node_id], "\t%d", q->hdr.path[i]);
	}
	fprintf(outf[node_id], "\n");
#endif
}

void logger_close(){
	for (int i=0; i< nnodes ; i++){
		fclose(outf[i]);
	}
}
