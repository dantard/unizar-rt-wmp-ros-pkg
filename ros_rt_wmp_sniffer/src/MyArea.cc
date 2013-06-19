/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: MyArea.cc
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

#include "config.h"
#include "MyArea.hh"
#include <cassert>
#include "misc.h"
#include <gtkmm/drawingarea.h>

using namespace std;

#define X_BORDER 30.0
#define Y_BORDER 15.0
class MyArea * my_area_c_wrapper;

MyArea::MyArea() : DrawingArea() {
        pthread_mutex_init(&sem, NULL);
        Glib::RefPtr<Gdk::Colormap> colormap = get_default_colormap ();
        cols[0] = Gdk::Color("black");
        cols[2] = Gdk::Color("red");
        cols[3] = Gdk::Color("green");
        cols[1] = Gdk::Color("orange");
        cols[4] = Gdk::Color("blue");
       	cols[5] = Gdk::Color("yellow");
       	cols[6] = Gdk::Color("orange");
       	cols[7] = Gdk::Color("forest green");
       	cols[8] = Gdk::Color("violet");
       	cols[9] = Gdk::Color("brown");
       	cols[10]= Gdk::Color("dark sea green");

        for (int i=0;i<11;i++){
	        colormap->alloc_color(cols[i]);
        }
        this->set_events(
        Gdk::BUTTON_PRESS_MASK |    //mouse button down
        Gdk::BUTTON_RELEASE_MASK |    //mouse button up
        Gdk::POINTER_MOTION_MASK);
        my_area_c_wrapper=this;
        selected=-1;
        mouse_on=-1;
        this->signal_button_press_event().connect(sigc::mem_fun(*this, &MyArea::on_MyArea_button_press_event), false);
        this->signal_button_release_event().connect(sigc::mem_fun(*this, &MyArea::on_MyArea_button_release_event), false);
        this->signal_motion_notify_event().connect(sigc::mem_fun(*this, &MyArea::on_MyArea_motion_notify_event), false);
}

void MyArea::beginInsert(){

}

void MyArea::delete_older(){
	WAIT(&sem);
	if (frameVec.size()>0)
		frameVec.erase( frameVec.begin() );
	SIGNAL(&sem);
}

int MyArea::get_older_offset(){
	if (frameVec.size()>0)
		return frameVec.at(0).pos_in_file;
		else return -1;
}
int MyArea::insertOne(int from, int to, unsigned long long time ,int pos_in_file,unsigned long serial,int col, int nt){
	WAIT(&sem);
	Frame f;
	f.src=from;
	f.dest=to;
	f.time=time;
	f.serial=serial;
	f.nt=nt;
	f.col=col;
	f.mouseover=0;
	f.pos_in_file=pos_in_file;
	f.mouseover=1;
	f.cross=false;
	f.mark=false;
	f.left_cross=false;
	if (frameVec.size()>0) frameVec.back().mouseover=0;
	frameVec.push_back(f);
	setActive(f.src);
	compute_environment();
	int res = frameVec.size()-1;
	SIGNAL(&sem);
	return res;
}
int MyArea::insertOne(int from, int to, long time ,int pos_in_file,int col, int nt){
	WAIT(&sem);
	Frame f;
	f.src=from;
	f.dest=to;
	f.time=time;
	f.nt=nt;
	f.col=col;
	f.mouseover=0;
	f.pos_in_file=pos_in_file;
	f.mouseover=1;
	f.cross=false;
	f.mark=false;
	f.left_cross=false;
	if (frameVec.size()>0) frameVec.back().mouseover=0;
	frameVec.push_back(f);
	setActive(f.src);
	compute_environment();
	int res = frameVec.size()-1;
	SIGNAL(&sem);
	return res;
}

void MyArea::addDot(int frame, int node, int color){
	WAIT(&sem);
	Dot f;
	f.node=node;
	f.col=color;
	frameVec.at(frame).dot.push_back(f);
	SIGNAL(&sem);
}

void MyArea::addMark(int frame){
	WAIT(&sem);
	frameVec.at(frame).mark=true;
	SIGNAL(&sem);
}

void MyArea::addCross(int frame){
	WAIT(&sem);
	frameVec.at(frame).cross=true;
	SIGNAL(&sem);
}

void MyArea::addLeftCross(int frame){
	WAIT(&sem);
	frameVec.at(frame).left_cross=true;
	SIGNAL(&sem);
}


void MyArea::insert(int from, int to, long time ,int col){
	WAIT(&sem);
	Frame f;
	f.src=from;
	f.dest=to;
	f.time=time;
	f.col=col;
	f.mouseover=1;
	frameVec.back().mouseover=0;
	frameVec.push_back(f);
	setActive(f.src);
	compute_environment();
	SIGNAL(&sem);
}

void MyArea::endInsert(){
	WAIT(&sem);
	compute_environment();
	SIGNAL(&sem);
}

void MyArea::setNumOfNodes(int n){
	WAIT(&sem);
	num_nodes=n;
	activeVec.resize(n);
	SIGNAL(&sem);
}

bool MyArea::timer_callback(){
	return 1;
}

void MyArea::on_realize(){
  	Gtk::DrawingArea::on_realize();
  	window = get_window();
  	gc = Gdk::GC::create(window);
  	window->clear();
}
int MyArea::set_selected(int n){
	WAIT(&sem);
	for (unsigned int i=0;i<frameVec.size();i++){
		frameVec.at(i).mouseover=0;
		if (frameVec.at(i).pos_in_file==n){
			selected=i;
			frameVec.at(i).mouseover=1;
		}
	}
	SIGNAL(&sem);
	return 0;
}

void MyArea::set_max_size(int _max_size){
	WAIT(&sem);
	max_size=_max_size;
	SIGNAL(&sem);
}

int MyArea::get_y_pos(int node_id){
	int height=this->get_height();
	int y_sep=(int) ((height - 2*Y_BORDER )/(num_nodes-1));
	return (int) (Y_BORDER+node_id*y_sep);
}

void MyArea::compute_environment(){
	max=0;min=2^15;
	for (unsigned int i=0;i<frameVec.size();i++){
		if (frameVec.at(i).time > max){
			max=frameVec.at(i).time;
		}
		if (frameVec.at(i).time < min){
			min=frameVec.at(i).time;
		}
	}
}

int idx=0;
void MyArea::paint(){
	WAIT(&sem);
	Glib::RefPtr<Pango::Layout> layout = create_pango_layout("");
	Pango::FontDescription *timesnr_9= new Pango::FontDescription("Times new roman 9");
	Pango::FontDescription *timesnr_10= new Pango::FontDescription("Times new roman bold 9");
    char tmp[64];

	gc->set_foreground(cols[0]);
	for (int i=0;i< num_nodes;i++){
		window->draw_line(gc,15,get_y_pos(i),this->get_width()-15,get_y_pos(i));
		if (!activeVec.at(i)){
			layout->set_font_description(*timesnr_9);
		}else{
			layout->set_font_description(*timesnr_10);
		}
		sprintf(tmp,"%d",i);//+idx++);
	    layout->set_markup(tmp);
	    get_window()->draw_layout(Gdk::GC::create(get_window()),8,get_y_pos(i)-6, layout);
	    get_window()->draw_layout(Gdk::GC::create(get_window()),this->get_width()-11,get_y_pos(i)-6, layout);
	}
	if (max==min) {
		SIGNAL(&sem);
		return;
	}
	int width=(int) (this->get_width()-2*X_BORDER);
	double factor;
	if (0){//(bytime){
		factor=(width/(max-min));
	} else{
		if (frameVec.size()>1){
			factor = (double) width / (double)(frameVec.size()-1);
		}else{
			factor = (double) width;
		}
	}
	for (unsigned int i=0;i< frameVec.size();i++){
		double x_pose = (double)X_BORDER + factor * (double) i;
		frameVec.at(i).x_draw=(int) x_pose;
		frameVec.at(i).y1_draw=get_y_pos(frameVec.at(i).src);
		frameVec.at(i).y2_draw=get_y_pos(frameVec.at(i).dest);
		for (unsigned int j=0; j< frameVec.at(i).dot.size();j++){

			frameVec.at(i).dot.at(j).x_draw=(int) x_pose;
			frameVec.at(i).dot.at(j).y_draw=get_y_pos(frameVec.at(i).dot.at(j).node);
		}
		draw_frame(i);
	}

	SIGNAL(&sem);
}

void MyArea::draw_frame(int i){
	gc->set_foreground(cols[frameVec.at(i).col]);
 	int xpos=frameVec.at(i).x_draw;
 	int y1=frameVec.at(i).y1_draw;
 	int y2=frameVec.at(i).y2_draw;
 	window->draw_line(gc,xpos,y1,xpos,y2);
	if (y1>y2){
		if (frameVec.at(i).nt != 2){
			window->draw_line(gc,xpos-5,y2+5,xpos,y2);
			window->draw_line(gc,xpos+5,y2+5,xpos,y2);
			if (frameVec.at(i).nt) window->draw_line(gc,xpos-5,y2+5,xpos+5,y2+5);
		}
	}else{
		if (frameVec.at(i).nt != 2){
			window->draw_line(gc,xpos-5,y2-5,xpos,y2);
			window->draw_line(gc,xpos+5,y2-5,xpos,y2);
			if (frameVec.at(i).nt) window->draw_line(gc,xpos-5,y2-5,xpos+5,y2-5);
		}
	}
	//if (frameVec.at(i).mouseover==2)gc->set_foreground(cols[0]);
	if (frameVec.at(i).mouseover){
		window->draw_line(gc,xpos-1,y1+1,xpos-1,y2+1);
		window->draw_line(gc,xpos+1,y1+1,xpos+1,y2+1);
	}

	if (frameVec.at(i).mark){
		if (y1 < y2){
			window->draw_line(gc,xpos-3,5+(y1+y2)/2,xpos+3,5+(y1+y2)/2);
		}else{
			window->draw_line(gc,xpos-3,-5+(y1+y2)/2,xpos+3,-5+(y1+y2)/2);
		}
	}

	if (frameVec.at(i).cross){
		window->draw_line(gc,xpos-3,+3+(y1+y2)/2,xpos+3,-3+(y1+y2)/2);
	}
	if (frameVec.at(i).left_cross){
		window->draw_line(gc,xpos-3,-3+(y1+y2)/2,xpos+3,+3+(y1+y2)/2);
	}

	for (unsigned int j=0; j< frameVec.at(i).dot.size(); j++){
		Dot d = frameVec.at(i).dot.at(j);
		gc->set_foreground(cols[d.col]);
		window->draw_point(gc,d.x_draw-1,d.y_draw);
		window->draw_point(gc,d.x_draw+1,d.y_draw);
		window->draw_point(gc,d.x_draw,d.y_draw-1);
		window->draw_point(gc,d.x_draw,d.y_draw+1);
		window->draw_point(gc,d.x_draw-2,d.y_draw);
		window->draw_point(gc,d.x_draw+2,d.y_draw);
		window->draw_point(gc,d.x_draw,d.y_draw-2);
		window->draw_point(gc,d.x_draw,d.y_draw+2);
		window->draw_point(gc,d.x_draw-1,d.y_draw-1);
		window->draw_point(gc,d.x_draw-1,d.y_draw+1);
		window->draw_point(gc,d.x_draw+1,d.y_draw+1);
		window->draw_point(gc,d.x_draw+1,d.y_draw-1);
	}
}

bool MyArea::on_MyArea_motion_notify_event(GdkEventMotion* ev){
	WAIT(&sem);
	mouse_on=-1;
	for (unsigned int i=0; i<frameVec.size(); i++) {
		if (i!=selected) frameVec.at(i).mouseover=0;
		if (abs((frameVec.at(i).x_draw-(int)ev->x))<5){
			frameVec.at(i).mouseover=2;
			mouse_on=i;
		}
	}
	SIGNAL(&sem);
	return 0;
}
int MyArea::get_selected_serial(){
	WAIT(&sem);
	if (selected>=0 && (unsigned int) selected < frameVec.size()){
		int ret = frameVec.at(selected).serial;
		SIGNAL(&sem);
		return ret;
	}
	SIGNAL(&sem);
	return -1;
}
long MyArea::get_selected_time(){
	WAIT(&sem);
	if (selected>=0 && selected< frameVec.size()){
		long ret = frameVec.at(selected).time;
		SIGNAL(&sem);
		return ret;
	}
	SIGNAL(&sem);
	return -1;
}
long MyArea::get_mouse_on_time(){
	WAIT(&sem);
	if (mouse_on>=0 && mouse_on< frameVec.size()){
		long ret = frameVec.at(mouse_on).time;
		SIGNAL(&sem);
		return ret;
	}
	SIGNAL(&sem);
	return -1;
}

int MyArea::get_mouse_on_serial(){
	if (mouse_on>=0 && mouse_on< frameVec.size()){
		int ret = frameVec.at(mouse_on).serial;
		SIGNAL(&sem);
		return ret;
	}
	SIGNAL(&sem);
	return -1;
}

void MyArea::clean_window(){
	WAIT(&sem);
	frameVec.clear();
	SIGNAL(&sem);
}

bool MyArea::on_expose_event(GdkEventExpose* e){
	//fprintf(stderr,"expose\n");
	paint();
    return true;
}

int MyArea::get_selected(){
	return selected;
}

int MyArea::get_offset_of_selected(){
	WAIT(&sem);
	if (selected<0 || frameVec.size()<selected) {
		SIGNAL(&sem);
		return -1;
	}
	int ret = frameVec.at(selected).pos_in_file;
	SIGNAL(&sem);
	return ret;
}

int MyArea::get_selected(int & from, int& to, int &col){
	WAIT(&sem);
	if (selected<0 || frameVec.size()<selected) {
		SIGNAL(&sem);
		return -1;
	}
	from=frameVec.at(selected).src;
	to=frameVec.at(selected).dest;
	col=frameVec.at(selected).col;
	SIGNAL(&sem);
	return selected;
}
 bool MyArea::on_MyArea_button_press_event(GdkEventButton *ev){
	 WAIT(&sem);
	 for (int i=0;i<frameVec.size();i++){
	 	frameVec.at(i).mouseover=0;
	 	if (abs((int)ev->x-frameVec.at(i).x_draw)<5){
	 		frameVec.at(i).mouseover=1;
	 		selected=i;
	 	}
	 }
	 SIGNAL(&sem);
	 return false;
}
bool MyArea::on_MyArea_button_release_event(GdkEventButton *ev){
	return true;
}

void MyArea::resetActive(){
	 WAIT(&sem);
	for (int i = 0 ; i< num_nodes; i++){
		activeVec.at(i)=false;
	}
	SIGNAL(&sem);
}
void MyArea::setActive(int i){
	activeVec.at(i)=true;
}
