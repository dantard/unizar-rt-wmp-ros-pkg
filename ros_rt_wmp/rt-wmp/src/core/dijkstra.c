/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/dijkstra.c
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

#include "include/dijkstra.h"
#include "include/dijkstra_alg.h"
#include "include/global.h"
#include "include/lqm.h"

void initDijkstra(int numNodes) {
	dij_init(numNodes);
}

void freeDijkstra(void) {
   dij_free();
}

#define likely(x)       __builtin_expect(!!(x), 1)

void put_matrix(char ** matrix){
	int i, j;
	for (i = 0; i < status.N_NODES; i++) {
		for (j = i; j < status.N_NODES; j++) {
			if (matrix[i][j] > 0 && matrix[j][i] > 0) {
				if (matrix[i][j] < matrix[j][i]) {
					long val = (long) (lqm_get_f())(matrix[i][j]);
					if (val < 0){
						val = 10000;
					}
					dij_set(i, j, val);
					dij_set(j, i, val);
				} else {
					long val = (long) (lqm_get_f())(matrix[j][i]);
					if (val < 0){
						val = 10000;
					}
					dij_set(i, j, val);
					dij_set(j, i, val);
				}
			} else {
				dij_set(i, j, 0);
				dij_set(j, i, 0);
			}
		}
	}
	//lqm_print();
	//dij_print_matrix();

}

void dij_put_matrix(char ** matrix){
	put_matrix(matrix);
}

int nextStepWithCost(char ** matrix,int src, int dest, int* cost) {
	char path[32];
	int hops;
	put_matrix(matrix);
	hops = dij_getPath(src,dest,path);
	if (hops > 0){
		return path[1];
	}else{
		return -1;
	}
}

int dijkstra_path_len(char ** matrix,int src, int dest) {
	char path[32];
   int hops;
	if (matrix != 0){
		put_matrix(matrix);
	}
	hops = dij_getPath(src,dest,path);
	return hops;
}

int isConnected(char ** matrix) {
	int connected;
	put_matrix(matrix);
	connected = dij_isConnected();
	return connected;
}

int isIsolated(char ** matrix,int node) {
	int isolated;
	put_matrix(matrix);
	isolated = dij_isIsolated(node);
	return isolated;
}

int dijkstra_is_node_within_path(char ** matrix,int src, int dest, int node) {
	char path[32];
   int hops, i;
	put_matrix(matrix);
	hops = dij_getPath(src,dest,path);
	for (i=0;i<=hops;i++){
		if (node == path[i]){
			return 1;
		}
	}
	return 0;
}

int getNeighbors(char ** matrix, int src, char * neighbors){
	char path[32];
	int hops, n_neighbors = 0, i;
	put_matrix(matrix);
	for (i = 0;  i < status.N_NODES; i++){
		hops = dij_getPath(src,i,path);
		if (hops == 1){
			neighbors[n_neighbors] = path[1];
			n_neighbors++;
		}
	}
	return n_neighbors;
}


int getPath(char ** lqm, int src, int dest, char * path){
	put_matrix(lqm);
	return dij_getPath(src,dest,path);
}

