/*------------------------------------------------------------------------
 *---------------------           RT-WMP              --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: ./example/common/empty/main.c
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <malloc.h>
#include <math.h>
#include <fcntl.h>

int m_size = 256;

void *fthread_tx (void * param){
	int dest, len, i=0, port = 0;
	signed char priority;
	char buff[1500000];
	
	sleep(3);

	fprintf(stderr,"Initializing TX thread...\n");
	int idx = 0;
	while (1){
		if (wmpGetNodeId() == 0){
			dest = 1 << (wmpGetNumOfNodes()-1); //Last node of the present
		}else{
			dest = 1;
		}
 		len = m_size;
		priority = 1;//rand()%5; 
		sprintf(buff,"MESSAGE n.%d from node %d dest %d",i++, wmpGetNodeId(),dest);
		wmpPushData(port,buff,len,dest,priority);
//		wmpPushData(port,buff,len,dest,priority);
//		wmpPushData(port,buff,len,dest,priority);
//		break;
		usleep(10000);
		idx++;
	}
}

void *fthread_rx (void * param){
	char *p;
	char priority;
	unsigned int size, port = 0;
	unsigned char src;
	sleep(3);

	fprintf(stderr,"Initializing RX thread...\n");
	char buff[1500000];

//	sprintf(buff,"MESSAGE2 n.%d from node %d dest %d",555, wmpGetNodeId(),1);
//	wmpPushData(port,buff,m_size,1,priority);

	while (1){

		int res = wmpPopDataTimeout(port,&p,&size,&src,&priority,1000);
		if(res>=0){
			fprintf(stderr,"Node %d Received message-> size: %d src:%d prio:%d text:%s \n",wmpGetNodeId(),size,src, priority,p);
		}
		wmpPopDataDone(res);
		if (0 && res>=0){
			static int i=0;
			sprintf(buff,"MESSAGE n.%d from node %d dest %d",i++, wmpGetNodeId(),1<<src);
			wmpPushData(0,buff,m_size,1<<src,priority);
		}
	}

}

int main(int argc, char* argv[]){
	if (argc < 5) {
		fprintf(stderr,"Use: %s id num_of_nodes size beluga\n",argv[0]);
		return 0;
	}
	wmpSetup(atoi(argv[1]), atoi(argv[2]));
	m_size = atoi(argv[3]);
	int beluga = atoi(argv[4]);
	setBeluga(beluga);

	pthread_t th1, th2;
	if (wmpGetNodeId()==(wmpGetNumOfNodes()-1)) 	{
		pthread_create(&th1,0,fthread_tx,0);
	}	
	pthread_create(&th2,0,fthread_rx,0);
//	wmpForceTopology("chain",0);
	wmpRun();
	return 0;
}
