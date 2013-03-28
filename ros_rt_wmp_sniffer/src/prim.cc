/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: prim.cc
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

#include "prim.h"
#include "matrix.h"
#define DBL_MAX 10e6
void Prim(Matrix & m){
	Matrix tree=m;
	Matrix graph=m; //tmp matrix
	tree*=0;
	unsigned numVertexOnTree=1,vertexOnTree[m.RowNo()];
	vertexOnTree[0]=0;
	while (numVertexOnTree < m.RowNo()){
		double minWeight=DBL_MAX;
		int minWeightIdxi=-1;
		int minWeightIdxj=-1;
		for (unsigned i=0;i<numVertexOnTree;i++){
			unsigned vertex=vertexOnTree[i]; //TODO: was int
			for (unsigned j=0;j<m.RowNo();j++){
				if (vertex==j)continue;
				if (graph(vertex,j) <= minWeight){
					minWeight=graph(vertex,j);
					minWeightIdxi=vertex;
					minWeightIdxj=j;
				}
			}
		}
		double v=m(minWeightIdxi,minWeightIdxj);
		tree(minWeightIdxi,minWeightIdxj)=v;
		tree(minWeightIdxj,minWeightIdxi)=v;
		for (unsigned i=0;i<numVertexOnTree;i++){
			graph(vertexOnTree[i],minWeightIdxj)=DBL_MAX;
			graph(minWeightIdxj,vertexOnTree[i])=DBL_MAX;
		}
		vertexOnTree[numVertexOnTree]=minWeightIdxj;
		numVertexOnTree++;
	}
	m=tree;
}


void prim(char * q, int size){
	char * p=q;
	Matrix m(size,size);
	for (int i=0; i<size;i++){
		for (int j=0; j<size;j++){
			if (i!=j) m(i,j)=100000/((double)(*p));
			else m(i,j)=0;
			p++;
		}
	}
	for (int i=0; i<size;i++){
		for (int j = 0; j < size; j++) {
			if (m(i, j) > m(j, i)) {
				m(j, i) = m(i, j);
			} else {
				m(i, j) = m(j, i);
			}
		}
	}

	Prim(m);
	p=q;
	for (int i=0; i<size;i++){
		for (int j=0; j<size;j++){
			*p=(char) m(i,j);
			p++;
		}
	}
}


