/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/lqm.c
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

#include "include/lqm.h"
#include "include/global.h"
#include "include/dijkstra.h"
#include "include/wmp_utils.h"
#include "include/nstat.h"

static char ** lqm, ** lqm_dist, ** lqm_copy, ** lqm_pruned;
static int ** A0, ** A1, ** Next;
static int size, net_connected = 0, informed = 0;
static THREAD_SEM_T isconn;

static char (*fp) (char);

int logs = 0;

char f_lqm(char val) {

	if (logs){
		if (val <= 1){
			return 0;
		}
		double v1 = (double) val;
		v1 = fabs(log(v1/100.0))*10.0;
		v1 = v1>127?127:v1;
		return (char) v1;
	}

	/* val is a % value (0-100) */
	if (val == 0){
		return 0;
	} else if (val < status.w100)
		return 100;
	else if (val >= status.w100 && val < status.w3)
		return 3;
	else if (val >= status.w2 && val < status.w1)
		return 2;
	else
		return 1;
}


void init_lqm(int m_size){
	int i;
	size=m_size;
	lqm=(char **) MALLOC(size*sizeof(char*));
	for (i=0;i<size;i++){
		lqm[i]=(char *) MALLOC(size*sizeof(char));
	}

	/* init distance matrix */
	lqm_dist=(char**) MALLOC(size*sizeof(char*));
	for (i=0;i<size;i++){
		lqm_dist[i]=(char *) MALLOC(size*sizeof(char));
		memset(lqm_dist[i],-1,size*sizeof(char));
	}

	/* init distance matrix */
	lqm_copy=(char**) MALLOC(size*sizeof(char*));
	for (i=0;i<size;i++){
		lqm_copy[i]=(char *) MALLOC(size*sizeof(char));
	}
	lqm_pruned=(char**) MALLOC(size*sizeof(char*));
	for (i=0;i<size;i++){
		lqm_pruned[i]=(char *) MALLOC(size*sizeof(char));
	}


	A0=(int**) MALLOC(size*sizeof(int*));
	for (i=0;i<size;i++){
		A0[i]=(int *) MALLOC(size*sizeof(int));
	}
	A1=(int**) MALLOC(size*sizeof(int*));
	for (i=0;i<size;i++){
		A1[i]=(int *) MALLOC(size*sizeof(int));
	}

	Next=(int**) MALLOC(size*sizeof(int*));
	for (i=0;i<size;i++){
		Next[i]=(int *) MALLOC(size*sizeof(int));
	}

	fp = f_lqm;

   THREAD_SEM_INIT_LOCKED(&isconn);
}

void free_lqm() {
	int i;

	for (i = 0; i < size; i++) {
		FREE((char*)lqm[i]);
	}
	FREE((char*)lqm);

	for (i = 0; i < size; i++) {
		FREE((char*)lqm_dist[i]);
	}
	FREE((char*)lqm_dist);

	for (i = 0; i < size; i++) {
		FREE((char*)lqm_copy[i]);
	}
	FREE((char*)lqm_copy);

	for (i = 0; i < size; i++) {
		FREE((char*)lqm_pruned[i]);
	}
	FREE((char*)lqm_pruned);

	for (i = 0; i < size; i++) {
		FREE((int*)A0[i]);
	}
	FREE((int*)A0);

	for (i = 0; i < size; i++) {
		FREE((int*)A1[i]);
	}
	FREE((int*)A1);
	for (i = 0; i < size; i++) {
		FREE((int*)Next[i]);
	}
	FREE((int*)Next);

}

void fill_lqm(char val){
	int i,j;
	for (i=0;i<size;i++){
		for (j=0;j<size;j++){
			if (i!=j) lqm[i][j]=val;
			else lqm[i][j]=0;
		}
	}
}

char lqm_get_val(int i, int j){
	return lqm[i][j];
}

int lqm_get_num_hops(int i, int j){
	if (i<0 || j<0 || i>=size || j>=size){
		return -1;
	}else{
		return lqm_dist[i][j];
	}
}

void lqm_set_val(int i, int j, char val){
	lqm[i][j]=val;
}

char** lqm_get_ptr(void){
	return lqm;
}

int  wmpIsNetworkConnected(void){
	return net_connected;
}

int  wmpIsNetworkConnectedBlocking(int timeout_ms){
	int ret;
	informed = 0;
	if (timeout_ms > 0){
      ret = ( THREAD_SEM_WAIT_TIMED(isconn, timeout_ms) == 0 );
	} else{
      THREAD_SEM_WAIT(&isconn);
		ret = 1;
	}
	return ret;
}

void lqm_calculate_distances(void){
	int i, j;
	net_connected = 1;

	/* simetriza la matriz */
	dij_put_matrix(lqm);

	for (i = 0; i < size; i++) {
		/* NOV 30 j=0 */
		for (j = 0; j < i; j++) {
			if (i != j) {
				int len = dijkstra_path_len(0, i, j);

				if (nstat_isLost(i) || nstat_isLost(j)){
					len  = -1;
				}

				lqm_dist[i][j] = len;
				lqm_dist[j][i] = len;
				net_connected = (net_connected && (len > 0));// && (lqm[i][j] <= 100));
			} else {
				lqm_dist[i][j] = 0;
			}

		}
	}
	if (net_connected && !informed) {
      THREAD_SEM_SIGNAL(&isconn);
		informed = 1;
	}else{
		informed = 0;
	}
}

int lqm_is_leaf(char id){
	int i, sum=0;
	for (i=0;i<size;i++){
		if (lqm[(int)id][i]>0) sum+=1;
	}
	if (sum > 1) return 0;
	else if (sum ==1) return 1;
	else return -1;
}
static int copied=0;

void lqm_backup(){
	int i,j;
	for (i=0;i<size;i++){
		for (j=0;j<size;j++){
			lqm_copy[i][j]=lqm[i][j];
		}
	}
	copied = 1;
}

void lqm_restore(){
	if (copied)	{
		int i,j;
		for (i=0;i<size;i++){
			for (j=0;j<size;j++){
				lqm[i][j]=lqm_copy[i][j];
			}
		}
	}
	copied=0;
}

int lqm_get_distance(char i, char j){
	return lqm_dist[(int)i][(int)j];
}


void print_lqm3(char* txt, char **lqm){
	char q[1020];
	lqmToString(lqm,q,status.N_NODES);
   WMP_ERROR(stderr,"@%s@\n%s#\n",txt,q);
}

char ** lqm_prune(char ** mlqm) {
   int i, j, val_ij, val_ji, found;

	if (!isConnected(lqm)) {
		return lqm;
	}

	for (i = 0; i < size; i++) {
		for (j = 0; j < size; j++) {
			lqm_pruned[i][j] = mlqm[i][j];
		}
	}

	//Quito el link más debil y veo si la red sigue conexa, luego el segundo más débil etc...


	found = 1;
	while (found) {
		int umbral = 15, sel_i = 0, sel_j = 0;
		found = 0;
		for (i = 0; i < size; i++) {
			for (j = 0; j < size; j++) {
				if (i == j){
					continue;
				}
				if (lqm_pruned[i][j] > 0 && lqm_pruned[i][j] < umbral) {
					sel_i = i;
					sel_j = j;
					found = 1;
					umbral = lqm_pruned[i][j];
				}
			}
		}

		if (found){
			//fprintf(stderr,"found seli:%d selj:%d umbral:%d\n",sel_i,sel_j,umbral);

			val_ij = lqm_pruned[sel_i][sel_j];
			val_ji = lqm_pruned[sel_j][sel_i];

			lqm_pruned[sel_i][sel_j] = 0;
			lqm_pruned[sel_j][sel_i] = 0;

			if (!isConnected(lqm_pruned)) {
				// marco los elementos ya procesados
				lqm_pruned[sel_i][sel_j] = -val_ij;
				lqm_pruned[sel_j][sel_i] = -val_ji;
			}
		}
		//print_lqm3("afterr",lqm_pruned);
	}
	for (i = 0; i < size; i++) {
		for (j = 0; j < size; j++) {
			if (i==j) {
				continue;
			}
			lqm_pruned[i][j] = abs(lqm_pruned[i][j]);
		}
	}

	//print_lqm3("after",lqm_pruned);

	return lqm_pruned;
}

void lqm_copy_to(char ** dest, char ** src) {
	int i,j;
	for (i = 0; i < size; i++) {
		for (j = 0; j < size; j++) {
			if (i != j) {
				dest[i][j] = src[i][j];
			}
		}
	}
}


char (*lqm_get_f())(char){
	return fp;
}


void lqm_set_f( char (*f) (char)){
	fp = f;
}

void lqm_compute_prob(char ** lqm) {
	int i,j,k;
	for (i=0; i< size; i++){
		for (j=0; j< size; j++){
			Next[i][j]=j;
			if (i==j){
				A0[i][j] = 1000;
			}else{
				A0[i][j] = lqm[i][j]*10; //XXX:
			}
		}
	}

	for (k = 0; k < size; k++) {
		for (i = 0; i < size; i++) {
			for (j = 0; j < size; j++) {
				if (A0[i][k] * A0[k][k] * A0[k][j] / 1000000 > A0[i][j]) {
					Next[i][j] = Next[i][k];
					A1[i][j] = A0[i][k] * A0[k][k] * A0[k][j] / 1000000;

				} else {
					A1[i][j] = A0[i][j];
				}
			}
		}
		memcpy(A0, A1, sizeof(A0));
	}
}

/* returns first hop (if exist) or -1 if not. If path is passed, the function returns the whole path */
int lqm_prob_get_path(int src, int dest, char * path) {
	int res = src, exres = 0, idx = 0;
	while (res != dest) {

		res = Next[res][dest];
//		fprintf(stderr,"src:%d dst:%d res:%d\n",src,dest,res);
		if (path != 0){
			path[idx] = res;
			idx++;
		}
//		if (res == exres){
//			return -1;
//		}
		exres = res;
	}
	return path[0];//Next[res][dest];
}
/* returns first hop (if exist) or -1 if not. If path is passed, the function returns the whole path */
int lqm_prob_get_val(int src, int dest) {
	return A0[src][dest];
}
