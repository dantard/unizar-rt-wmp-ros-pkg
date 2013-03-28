/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/dijkstra_alg.c
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

#include "config/compiler.h"
#include "include/global.h"
#define DIJ_INFINITY 10000

static int n; /* The number of nodes in the graph */
static long **dist; /* dist[i][j] is the distance between node i and j; or 0 if there is no direct connection */
static long *d; /* d[i] is the length of the shortest path between the source (s) and node i */
static int *prev; /* prev[i] is the node that comes right before i in the shortest path from the source to i*/
static int *visited;

void dijkstra(int s) {
	int i, k, mini;

	for (i = 0; i < n; ++i) {
		d[i] = DIJ_INFINITY;
		prev[i] = -1; /* no path has yet been found to i */
		visited[i] = 0; /* the i-th element has not yet been visited */
	}

	d[s] = 0;

	for (k = 0; k < n; ++k) {
		mini = -1;
		for (i = 0; i < n; ++i)
			if (!visited[i] && ((mini == -1) || (d[i] < d[mini])))
				mini = i;

		visited[mini] = 1;

		for (i = 0; i < n; ++i)
			if (dist[mini][i])
				if (d[mini] + dist[mini][i] < d[i]) {
					d[i] = d[mini] + dist[mini][i];
					prev[i] = mini;
				}
	}
}

void dij_init(int matrix_size) {

	int i;
	n = matrix_size;
	dist= (long**) MALLOC(n*sizeof(long*));
	for (i=0;i<n;i++){
		dist[i]=(long *) MALLOC(n*sizeof(long));
	}
	d=(long *) MALLOC(n*sizeof(long));
	prev=(int *) MALLOC(n*sizeof(long));
	visited=(int *) MALLOC(n*sizeof(int));
}

void dij_free (void) {
   int i;

   for (i=0;i<n;i++){
      FREE(dist[i]);
   }
   FREE(dist);
   FREE(d);
   FREE(prev);
   FREE(visited);
}

static int check_range(int i, int j){
	if (i >= 0 && i < status.N_NODES && j >= 0 && j < status.N_NODES){
		return 1;
	}else{
		return 0;
	}
}

long ** dij_get_ptr(void){
	return dist;
}

void dij_set(int i, int j, long val) {
	if (check_range(i,j)){
		dist[i][j] = val;
	}
}
long dij_get(int i, int j) {
	if (check_range(i,j)){
		return dist[i][j];
	} else{
		return -1;
	}
}


int dij_getPath(int src, int dest, char* path) {
	int idx = 0, j = 0;
	if (!check_range(src,dest)){
		return -1;
	}
	dijkstra(src);
	if (d[dest] == DIJ_INFINITY) {
		WMP_DBG(DIJKSTRA,"No PATH");
		return -1;
	}
	while (prev[dest] != -1) {
		path[idx] = dest;
		idx = idx + 1;
		dest = prev[dest];
	}
	path[idx] = dest;

	for (j = 0; j <= idx / 2; j++) {
		int a = path[j];
		path[j] = path[idx - j];
		path[idx - j] = a;

	}
	return idx;
}

int dij_getDist(int src, int dest) {
	if (!check_range(src,dest)){
		return -1;
	}
	dijkstra(src);
	return d[dest];
}




int dij_isConnected(void) {
	int i=0, connected=1;
	dijkstra(0);
	for (i=0; i<n; i++){
		connected = connected && (d[i]<DIJ_INFINITY);
	}
	return connected;
}

int dij_isIsolated(int src) {
	// This way I'll control if the node can reach me
	//dijkstra(src);
	//return (d[status.id] ==  DIJ_INFINITY);

	// This way I'll control if I can reach the node
	if (!check_range(src,src)){
		return 1;
	}
	dijkstra(status.id);
	return (d[src] ==  DIJ_INFINITY);
}



int dij_isIsolated2(int src) {
	dijkstra(src);
	return (d[status.id] ==  DIJ_INFINITY);
}




