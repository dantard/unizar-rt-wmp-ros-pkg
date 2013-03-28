/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./src/core/prim.c
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
#include "include/wmp_utils.h"

#define DBL_MAX 2^31

static char **tree;
static int **graph;
static char *vertexOnTree,N_ROBOTS;

void init_prim(int nnodes){
	int i;
	N_ROBOTS=nnodes;
	graph=(int **) MALLOC(N_ROBOTS*sizeof(int*));
	tree=(char **) MALLOC(N_ROBOTS*sizeof(char*));
	// ??? vertexOnTree=(int *) MALLOC(N_ROBOTS*N_ROBOTS*sizeof(char));
	vertexOnTree=(char *) MALLOC(N_ROBOTS*N_ROBOTS*sizeof(char));
	for (i=0;i<N_ROBOTS;i++){
		graph[i]=(int *) MALLOC(N_ROBOTS*sizeof(int));
		tree[i]=(char *) MALLOC(N_ROBOTS*sizeof(char));
	}
}

void free_prim(void){
   int i;

   for (i=0;i<N_ROBOTS;i++){
      FREE(graph[i]);
      FREE(tree[i]);
   }
   FREE(vertexOnTree);
   FREE(tree);
   FREE(graph);
}


char ** prim(char **lqm){
	int i,j, numVertexOnTree;
   char val;
	int fail = 0;
	for (i=0;i<N_ROBOTS;i++){
		for (j=0;j<N_ROBOTS;j++){
			tree[i][j]=0;
			val = lqm[i][j] < lqm[j][i] ? lqm[i][j]:lqm[j][i];
			/* DEC09 simetrification matrix (less value) */
			if (val > 0){
				graph[i][j]=100 - val;
				graph[j][i]=100 - val;
			} else{
				graph[i][j]=DBL_MAX;
				graph[j][i]=DBL_MAX;
			}
			WMP_DBG(PRIM,"lqm: %d :%d :%d %f\n",lqm[i][j],i,j,graph[i][j]);
		}
	}

	numVertexOnTree=1;    // ,vertexOnTree[N_ROBOTS];
	vertexOnTree[0]=0;
	while (numVertexOnTree < N_ROBOTS){
		int minWeight=DBL_MAX, v;

		int minWeightIdxi=-1;
		int minWeightIdxj=-1;
		int i,j;
		for (i=0;i<numVertexOnTree;i++){
			int vertex=vertexOnTree[i];
			for (j=0;j<N_ROBOTS;j++){
				if (vertex==j) {
					continue;
				}

				if (graph[vertex][j] <= minWeight){
					minWeight=graph[vertex][j];
					minWeightIdxi=vertex;
					minWeightIdxj=j;
				}
			}
		}
		/* XXX: Hay un caso en el que minWeightIdxi y minWeightIdxj = -1 */
		if (minWeightIdxi < 0 || minWeightIdxj < 0){
			static char txt[1000];
			lqmToString(lqm,txt, N_ROBOTS);
			WMP_DBG(PRIM,"PRIM Failed, LQM:\n %s\n",txt);
			fail = 1;
			break;
		}
		v=lqm[minWeightIdxi][minWeightIdxj];
		if (v>0) {
			v=99;
		} else {
			v=0;
		}
		tree[minWeightIdxi][minWeightIdxj]=v;
		tree[minWeightIdxj][minWeightIdxi]=v;
		for (i=0;i<numVertexOnTree;i++){
			graph[(int)vertexOnTree[i]][minWeightIdxj]=DBL_MAX;
			graph[minWeightIdxj][(int)vertexOnTree[i]]=DBL_MAX;
		}
		vertexOnTree[numVertexOnTree]=minWeightIdxj;
		numVertexOnTree++;
	}
	if (!fail){
		return tree;
	} else {
		return lqm;
	}
}






