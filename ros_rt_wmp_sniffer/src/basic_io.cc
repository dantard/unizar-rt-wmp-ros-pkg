/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: basic_io.cc
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
#include <unistd.h>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <sys/io.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "wmp_config.h"

#include "core/include/frames.h"
#include "basic_io.h"
#include <vector>
#include <map>
#include <iostream>
#include <errno.h>

struct  file_hdr_t{
	unsigned short num_of_nodes;
};

static file_hdr_t file_hdr;

static int  num_nodes;



file_t file,tmp_file;

void io_init() {
	memset(&file,0,sizeof(file));
	num_nodes = 0;
}
void io_flush(){
	fflush(file.ptr);
}

std::vector<int> f_offset;
std::map<int,int> serialToOffset;

int io_change_file(){
	io_close_sim_data();
	return 0;
}

int io_open_sim_data(char * filename) {

	strcpy(file.name, filename);

	file.ptr = fopen(file.name, "r");
	if (!file.ptr) {
		file.open = false;
		return 0;
	}else{
		file.open = true;
	}

	int n = fread(&file_hdr, 1,sizeof (file_hdr), file.ptr);
	if (n != sizeof(file_hdr)){
		return 0;
	}

	num_nodes = file_hdr.num_of_nodes;

    char buf[sizeof(simData_Hdr) + sizeof(Token_Hdr)];
    simData_Hdr  * smd = (simData_Hdr *) buf;
    Token_Hdr * th = (Token_Hdr *) (buf + sizeof(simData_Hdr));

    file.pos = ftell(file.ptr);
    f_offset.clear();

    int idx = ftell(file.ptr);
    int unit = sizeof (simData_Hdr) + sizeof(Token_Hdr);

    std::cerr << "Scanning File " << file.name << std::endl;
    while (1) {
        n = fread(buf, 1, unit, file.ptr);
    	if (n == unit) {
    			f_offset.push_back(idx);
    			serialToOffset[th->serial] = f_offset.size();
    			idx+=smd->simDataLen;
    			fseek(file.ptr, idx, SEEK_SET);

   		}else{
   			break;
   		}
    }
    std::cerr << "Done (" << f_offset.size() << " frames detected)" <<std::endl;
    return num_nodes;
}

int io_close_sim_data() {
	if (file.open){
		fflush(file.ptr);
//		if (fclose(file.ptr)!=0){
//			fprintf(stderr,"Problems to close file...");
//			return 0;
//		}
	}
	file.open = false;
	return 1;
}

int io_get_pose_from_serial(int serial){
	return serialToOffset[serial];
}

int io_get_num_of_nodes() {
    return num_nodes;
}

int io_get_file_size() {
    struct stat stbuf;
    stat(file.name, &stbuf);
    return stbuf.st_size;
}

int io_go_to(int n) {
    //fseek(file.ptr, unit_size*n, SEEK_SET);
	//std::cerr << "Size: " << f_offset.size() << std::endl;
	//std::cerr << "request to go to " << n << "=" << f_offset.at(n) << std::endl;
	if (!file.open){
		return -1;
	}
	if (f_offset.size() > n){
		fseek(file.ptr, f_offset.at(n), SEEK_SET);
		file.pos = n;

	} else{
		file.pos = f_offset.size();
	}
	return file.pos;
	//std::cerr << "Done\n";
}

int io_read_next_sim_data(char * p) {
    simData_Hdr  * sdHdr = (simData_Hdr *) p;
    wmpFrame * r = (wmpFrame *) (p + sizeof (simData_Hdr));

    file.pos++;
    int nbytes = fread(sdHdr, 1,sizeof(simData_Hdr), file.ptr);

    if (nbytes == sizeof(simData_Hdr)) {
			nbytes += fread(p + sizeof(simData_Hdr), 1, sdHdr->simDataLen
				- sizeof(simData_Hdr), file.ptr);
	} else {
		return 0;
	}

    if (!valid_frame(r, nbytes,num_nodes)) {
    	return 0;
    }
    return nbytes;
}

int io_read_sim_data(char * p, int pos) {
    file.pos = pos;
    fseek(file.ptr, f_offset.at(pos), SEEK_SET);

    simData_Hdr  * sdHdr = (simData_Hdr *) p;
    int nbytes = fread(sdHdr, 1, sizeof(simData_Hdr), file.ptr);
    nbytes += fread(p + sizeof(simData_Hdr), 1, sdHdr->simDataLen - sizeof(simData_Hdr), file.ptr);

    return nbytes;
}

int io_get_sim_data_num_of_elements() {
    return (int) f_offset.size();
}

int io_reopen_file_to_write(int _num_nodes) {
	file_hdr.num_of_nodes = _num_nodes;
	if (file.open){
		fclose(file.ptr);
	}
	file.ptr = fopen(file.name, "w");
	if (!file.ptr) {
		file.open = false;
		return 0;
	}
	file.open = true;
	fwrite(&file_hdr, 1, sizeof(file_hdr), file.ptr);
	return 0;
}

int io_get_filepos(){
	return file.pos;
}
int io_write_sim_frame(char * fdata, int data_size){
	int size = fwrite(&fdata[0], 1, data_size, file.ptr);
	//fprintf(stderr,"data Written (%d bytes) \n", size);
	return size;
}

int io_write_on_tmp_file(char * fdata, int data_size){

	int size = fwrite(fdata, 1, data_size, tmp_file.ptr);
	fprintf(stderr,"data Written (%d bytes) \n", size);
	return 1;
}

int io_create_tmp_file() {
	tmp_file.ptr = fopen("tmp.vis", "w+");
	if (!tmp_file.ptr) {
		fprintf(stderr,"Unable to open temp file!\n");
		exit(0);
		tmp_file.open = false;
		return 0;
	}
	int size = fwrite(&file_hdr, 1, sizeof(file_hdr), tmp_file.ptr);
	fprintf(stderr,"Header Writter (%d bytes) \n", size);
	return 0;
}
int io_close_tmp_file(){
	return fclose(tmp_file.ptr  );
}

