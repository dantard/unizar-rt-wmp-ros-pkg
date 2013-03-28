/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: area.cc
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
#include "area.h"
#include <cmath>
#include "values.h"
#include <cstdio>
#include <vector>
#include <exception>

#include <gtk-2.0/gdk/gdk.h>
#define MOBILE_NODES
#define FIXED_ENVIRONMENT
#define ARROW_ONLY_ON_LAST

#include "pthread.h"
#define WAIT(p) pthread_mutex_lock(p)
#define SIGNAL(p) pthread_mutex_unlock(p)

#define X_BORDER 10
#define Y_BORDER 10

class area* area_c_wrapper;

bool area::on_area_button_press_event(GdkEventButton *ev){

	//pointVec.at(i).x_draw= X_BORDER + (int) ((pointVec.at(i).x - min_x)*scalex);
	//pointVec.at(i).y_draw= (int) w_height + Y_BORDER - (int) ((pointVec.at(i).y - min_y)*scaley);

	double w_height=this->get_height()-2*Y_BORDER;
	yClick= - (min_x + ( ev->x - (double) X_BORDER  ) / scalex);
	xClick= min_y - ( ev->y - w_height - (double) Y_BORDER) /scaley;
	//fprintf(stderr,"aaa %f %f\n",xClick,yClick);
	sem_post(&clickSem);
	return 0;
}

void area::waitClick(double &x, double &y){
	sem_wait(&clickSem);
	if (interrupted){
		interrupted=false;
		throw 0;
	}
	x=xClick;
	y=yClick;
}

void area::waitClickInterrupt(){
	interrupted=true;
	sem_post(&clickSem);
}

bool area::on_area_button_release_event(GdkEventButton *ev){
	return 0;
}

bool area::on_area_expose_event(GdkEventExpose *ev){
	return 0;
}

int area::get_selected(){
	return painted;
}

bool area::on_area_motion_notify_event(GdkEventMotion *ev){
	WAIT(&sem);
	if (selected==-1){
			for (unsigned i=0;i<roboVec.size();i++){
			double x_rob=roboVec.at(i).x_draw;
			double y_rob=roboVec.at(i).y_draw;
			if (selected==-1 && sqrt( (x_rob-ev->x)*(x_rob-ev->x) + (y_rob-ev->y)*(y_rob-ev->y) ) < 20){
				roboVec.at(i).has_mouse_over=1;
				selected=i;
				painted=i;
			} else roboVec.at(i).has_mouse_over=0;
		}
	}
	if (!(ev->state & 256)) {
		selected=-1;
		SIGNAL(&sem);
		return true;
	}
#ifdef MOBILE_NODES
	if (selected!=-1){
#ifdef FIXED_ENVIRONMENT
		if (ev->x+X_BORDER>this->get_width() || ev->y+Y_BORDER>this->get_height()
				|| ev->x < X_BORDER || ev->y < Y_BORDER) {
			SIGNAL(&sem);
			return true;
		}
#endif
		double x=(min_x*scalex + ev->x - X_BORDER)/scalex;
		double y=(min_y*scaley - ev->y - Y_BORDER + this->get_height())/scaley;
		roboVec.at(selected).x=x;
		roboVec.at(selected).y=y;
	}
#endif
	compute_environment();
	SIGNAL(&sem);
	return true;
}


bool area::timer_callback(){
		return 1;
}

area::area(): DrawingArea(){

	pthread_mutex_init(&sem, NULL);
	sem_init(&clickSem,1,0);

	maximumNumOfPoints=360;
	fixed_scale=-1;
	snap_id=0;
	has_bg_image=false;
	join_points=false;
	marginSetted=false;
    Glib::RefPtr<Gdk::Colormap> colormap = get_default_colormap ();
    cols[0] = Gdk::Color("black");
    cols[2] = Gdk::Color("red");
    cols[3] = Gdk::Color("green");
    cols[1] = Gdk::Color("yellow");
    cols[4] = Gdk::Color("blue");
    cols[5] = Gdk::Color("violet");
    for (unsigned i=0;i<6;i++){
        colormap->alloc_color(cols[i]);
    }
    this->set_events(
    Gdk::BUTTON_PRESS_MASK |    //mouse button down
    Gdk::BUTTON_RELEASE_MASK |    //mouse button up
    Gdk::POINTER_MOTION_MASK);


    painted=selected=-1;
    area_c_wrapper=this;
	this->signal_size_allocate().connect(sigc::mem_fun(*this, &area::on_size_alloc));
	//this->signal_motion_notify_event().connect(sigc::mem_fun(*this, &area::on_area_motion_notify_event));
	this->signal_button_press_event().connect(sigc::mem_fun(*this, &area::on_area_button_press_event));
	timesnr_12 = new Pango::FontDescription("Times new roman 9");
	timesnr_8 = new Pango::FontDescription("Times new roman 7");
	maintain_aspect_ratio=true;
	interrupted=false;
}
int area::getRobotId(std::string name){
	return roboMap[name];
}

int area::addRobot(std::string name){
	WAIT(&sem);
	Robot r;
	r.x=0;
	r.y=0;
	r.a=0;
	r.col=0;
	r.name=name;
	roboVec.push_back(r);
	compute_environment();
	int id=roboVec.size() - 1;
	roboMap[name]=id;
	SIGNAL(&sem);
	return (id);
}

void area::on_size_alloc(Gtk::Allocation &al){
	WAIT(&sem);
	compute_environment();
	SIGNAL(&sem);
}

void area::addPoint(double x, double y, int color, int with_respect_to_robot){
	/*WAIT(&sem);
	Point p;
	p.col=color;
	if (1 || with_respect_to_robot<0){
		p.x=x;
		p.y=y;
	}
	p.cell_size=1.0;
	pointVec.push_back(p);
    if (pointVec.size()>maximumNumOfPoints) pointVec.erase(pointVec.begin());
    compute_environment();
	SIGNAL(&sem);*/
	addGridPoint(x,y,0.0,color,with_respect_to_robot);
}

void area::addGridPoint(double x, double y, double cell_size, int color, int with_respect_to_robot){
	WAIT(&sem);
	Point p;
	p.col=color;
	p.cell_size=cell_size;
	if (cell_size>0){
		p.x=round(x/cell_size)*cell_size;
		p.y=round(y/cell_size)*cell_size;
	}else{
		p.x=x;
		p.y=y;
	}
	for (unsigned i=0;i<pointVec.size();i++){
		if (pointVec[i].x==p.x && pointVec[i].y==p.y){
			SIGNAL(&sem);
			return;
		}
	}
	pointVec.push_back(p);
    if (pointVec.size()>maximumNumOfPoints) pointVec.erase(pointVec.begin());
    compute_environment();
	SIGNAL(&sem);
}

void addLine(double x1, double y1, double x2, double y2, int color){

}

void area::addGoal(int r1, double x2, double y2, int color,int arrow){
	WAIT(&sem);
	Goal g;
	g.x=x2;
	g.y=y2;
	g.r1=r1;
	g.col=color;
	goalVec.push_back(g);
	compute_environment();
	SIGNAL(&sem);
}

void area::addPoint(double y, int color){
	WAIT(&sem);
	Point p;
	p.col=color;
	p.y=y;
	p.x=pointVec.size();
	pointVec.push_back(p);
    if (pointVec.size()>maximumNumOfPoints) {
    	pointVec.erase(pointVec.begin());
    	for (unsigned i=0;i<pointVec.size();i++){
    		pointVec[i].x=i;
    	}
    }
    compute_environment();
	SIGNAL(&sem);
}

void area::setMaximumNumOfPoints(int n){
	maximumNumOfPoints=n;
}

void area::addLink(int r1, int r2, int arrow, int color, float value, int set){
	WAIT(&sem);
#ifdef ARROW_ONLY_ON_LAST
    for (unsigned i=0;i<linkVec[set].size();i++) linkVec[set].at(i).arrow=0;
#endif
    Link l;
	l.r1=r1;
	l.r2=r2;
	l.col=color;
	l.arrow=arrow;
	l.has_value=true;
	l.value=value;
	linkVec[set].push_back(l);
	SIGNAL(&sem);
}

void area::addLink(int r1, int r2, int arrow, int color, int set){
	WAIT(&sem);
#ifdef ARROW_ONLY_ON_LAST
    for (unsigned i=0;i<linkVec[set].size();i++) linkVec[set].at(i).arrow=0;
#endif
    Link l;
	l.r1=r1;
	l.r2=r2;
	l.col=color;
	l.arrow=arrow;
	l.has_value=false;
	linkVec[set].push_back(l);
	SIGNAL(&sem);
}

int area::addRobot(){
	WAIT(&sem);
	Robot r;
	r.x=0;
	r.y=0;
	r.a=0;
	r.name.assign("");
	r.col=0;
	roboVec.push_back(r);
	compute_environment();
	SIGNAL(&sem);
	return (roboVec.size() - 1);
}
int area::getNumOfRobots(){
	return roboVec.size();
}

void area::setRobotPose(int id, double x0, double y0, double ang, int col=0){
	WAIT(&sem);
	roboVec.at(id).x=x0;
	roboVec.at(id).y=y0;
	roboVec.at(id).a=ang;
	roboVec.at(id).col=col;
	compute_environment();
	SIGNAL(&sem);
}

int area::getRobotPose(int id, double* x0, double* y0, double* ang){
	WAIT(&sem);
	if (id >= 0 && (unsigned) id < roboVec.size()){
		*x0=roboVec.at(id).x;
		*y0=roboVec.at(id).y;
		*ang=roboVec.at(id).a;
		SIGNAL(&sem);
		return id;
	}
	SIGNAL(&sem);
	return -1;
}

void area::draw_robot(double x, double y, double a, int col,int mouse_over, double scale=0.1){
		static const double ROBOT [2][7] = {{-312.5, 312.5, 312.5, -312.5, 312.5, -312.5, -312.5},
						    {   250,   250,  -250,   -250,     0,    250,   -250}};
	    int robotX[7];
	    int robotY[7];
		double s = sin(a+M_PI/2);
	    double c = cos(a+M_PI/2);
	    if (scale>0.05) scale=0.05;
	    if (scale<0.001) scale=0.001;
	    for (unsigned i = 0; i < 7; i++){
        	robotX[i] = (int) rint(ROBOT[0][i] * c * scale + ROBOT[1][i] * s * scale);
	        robotY[i] = (int) rint(ROBOT[1][i] * c * scale - ROBOT[0][i] * s * scale);
	    }
	    //XXXX
	    mouse_over=0;
	    if (mouse_over==1) gc->set_foreground(cols[2]);
	    else gc->set_foreground(cols[col]);

		for (unsigned i=0;i<6;i++){
        	window->draw_line(gc, (int)x+robotX[i], (int)y+robotY[i],(int)x+robotX[i+1], (int)y+robotY[i+1]);
		}
}

void area::draw_triangle(double x, double y, double a, int col, double scale){
		static const double TRIANGLE [2][4] = {{-100.5, 100.5, 0.0, -100.5}, {0.0,0.0,100.5, 0.0 } };
	    int triangleX[4];
	    int triangleY[4];
		double s = sin(a+M_PI/2);
	    double c = cos(a+M_PI/2);
	    if (scale>0.1) scale=0.1;
	    if (scale<0.01) scale=0.01;

	    for (unsigned i = 0; i < 4; i++){
	    	triangleX[i] = (int) rint(TRIANGLE[0][i] * c * scale + TRIANGLE[1][i] * s * scale);
	    	triangleY[i] = (int) rint(TRIANGLE[1][i] * c * scale - TRIANGLE[0][i] * s * scale);
	    }
	    gc->set_foreground(cols[col]);
		for (unsigned i=0;i<3;i++){
        	window->draw_line(gc, (int)x+triangleX[i], (int)y+triangleY[i],(int)x+triangleX[i+1], (int)y+triangleY[i+1]);
		}
}
void area::setMaintainAspectRatio(bool val){
	maintain_aspect_ratio=val;
}

void area::clearLinks(int set){
	WAIT(&sem);
	linkVec[set].clear();
	SIGNAL(&sem);
}

void area::clearRobots(){
	WAIT(&sem);
	roboVec.clear();
	roboMap.clear();
	SIGNAL(&sem);
}
void area::clearPoints(){
	WAIT(&sem);
	pointVec.clear();
	SIGNAL(&sem);
}
void area::on_realize(){
	WAIT(&sem);
	Gtk::DrawingArea::on_realize();
	window = get_window();
	gc = Gdk::GC::create(window);
	window->clear();
	initialized=1;
	SIGNAL(&sem);
}
void area::setMargins(double maxx,double maxy, double minx, double miny){
	marginSetted=true;
	canvas_maxx=maxx;
	canvas_maxy=maxy;
	canvas_minx=minx;
	canvas_miny=miny;
}

void area::compute_environment(){
	double width=0, height=0;
	max_x = -2^30; max_y= -2^30; min_x=2^30; min_y=2^30;
	for (unsigned i=0;i< roboVec.size();i++){
		max_x= roboVec.at(i).x>max_x+25? roboVec.at(i).x+25:max_x;
		max_y= roboVec.at(i).y>max_y+25? roboVec.at(i).y+25:max_y;
		min_y= roboVec.at(i).y<min_y-25? roboVec.at(i).y:min_y-25;
		min_x= roboVec.at(i).x<min_x-25? roboVec.at(i).x:min_x-25;
	}
	for (unsigned i=0;i< pointVec.size();i++){
		max_x= pointVec.at(i).x>max_x? pointVec.at(i).x:max_x;
		max_y= pointVec.at(i).y>max_y? pointVec.at(i).y:max_y;
		min_y= pointVec.at(i).y<min_y? pointVec.at(i).y:min_y;
		min_x= pointVec.at(i).x<min_x? pointVec.at(i).x:min_x;
	}
	for (unsigned i=0;i< goalVec.size();i++){
	        max_x= goalVec.at(i).x>max_x? goalVec.at(i).x:max_x;
	        max_y= goalVec.at(i).y>max_y? goalVec.at(i).y:max_y;
	        min_y= goalVec.at(i).y<min_y? goalVec.at(i).y:min_y;
	        min_x= goalVec.at(i).x<min_x? goalVec.at(i).x:min_x;
	}

	if (marginSetted){
		if (max_x < canvas_maxx) {
			max_x = canvas_maxx;
		}
		if (max_y < canvas_maxy) {
			max_y = canvas_maxy;
		}
		if (min_x > canvas_minx) {
			min_x = canvas_minx;
		}
		if (min_y > canvas_miny) {
			min_y = canvas_miny;
		}
	}

	width=fabs(max_x-min_x);
	height=fabs(max_y-min_y);

	double w_width=this->get_width()-2*X_BORDER;
	double w_height=this->get_height()-2*Y_BORDER;
	double r1=w_width/width;
	double r2=w_height/height;

	if (maintain_aspect_ratio){
		scalex = scaley = r1<r2?r1:r2;
	}else{
		scalex=r1;//1/width;
		scaley=r2;//1/height;
	}

	for (unsigned i=0;i< roboVec.size();i++){
		roboVec.at(i).x_draw= X_BORDER + (int) ((roboVec.at(i).x - min_x)*scalex);
    	roboVec.at(i).y_draw= (int) w_height + Y_BORDER - (int) ((roboVec.at(i).y - min_y)*scaley);
    }
    for (unsigned i=0;i< pointVec.size();i++){
    	pointVec.at(i).x_draw= X_BORDER + (int) ((pointVec.at(i).x - min_x)*scalex);
      	pointVec.at(i).y_draw= (int) w_height + Y_BORDER - (int) ((pointVec.at(i).y - min_y)*scaley);
    }

    for (unsigned i=0;i< imgVec.size();i++){
    	imgVec.at(i).x_draw= X_BORDER + (int) ((imgVec.at(i).x - min_x)*scalex);
        imgVec.at(i).y_draw= (int) w_height + Y_BORDER - (int) ((imgVec.at(i).y - min_y)*scaley);

        double actx=imgVec.at(i).sizex*scalex*0.020;
        //double acty=imgVec.at(i).sizey*scaley*0.020;

        if (imgVec.at(i).sizex!=0 && actx!=0 ){
        	imgVec.at(i).scale=actx/imgVec.at(i).sizex;
        } else{
        	imgVec.at(i).scale=1;
        }
    }

    for (unsigned i=0;i< goalVec.size();i++){
        goalVec.at(i).x_draw= X_BORDER + (int) ((goalVec.at(i).x - min_x)*scalex);
        goalVec.at(i).y_draw= (int) w_height + Y_BORDER - (int) ((goalVec.at(i).y - min_y)*scaley);
    }
    zoom=sqrt(w_width*w_height/(2500*width*height));
}

void area::setJoinPoints(bool n){
	join_points=n;
}

void area::setBackGroundImage(std::string s){
	bg_image = Gdk::Pixbuf::create_from_file(s.c_str());
	has_bg_image=true;
}

int area::addImage(std::string s, double x, double y, double scale, std::string s1){
	WAIT(&sem);
	Img m;
	m.scale_type=0;
	m.filename.assign(s.c_str());
	m.text.assign(s1);
	m.x=x;
	m.y=y;
	m.scale=scale;

	m.pixbuf = Gdk::Pixbuf::create_from_file(s.c_str());

	imgVec.push_back(m);
	compute_environment();
	SIGNAL(&sem);
	return imgVec.size()-1;

}

int area::addImage(std::string s, double x, double y, double scale){
	return addImage(s,x,y,scale,"");
}

int area::addImage(std::string s, double x, double y, double sizex, double sizey, std::string s1){
	WAIT(&sem);
	Img m;
	m.scale_type=1;
	m.filename.assign(s.c_str());
	m.text.assign(s1);
	m.x=x;
	m.y=y;
	m.sizex=sizex;
	m.sizey=sizey;

	m.pixbuf = Gdk::Pixbuf::create_from_file(s.c_str());

	imgVec.push_back(m);
	compute_environment();
	SIGNAL(&sem);
	return imgVec.size()-1;
}


void area::paint(){
	WAIT(&sem);
	get_window()->clear();
	Glib::RefPtr<Pango::Layout> layout = create_pango_layout("");
    layout->set_font_description(*timesnr_12);
    char tmp[64];


	if (has_bg_image){
		/* Paint image >*/
		Glib::RefPtr<Gdk::Pixbuf> image2 = bg_image->scale_simple(get_width(),get_height(),Gdk::INTERP_NEAREST);
		image2->render_to_drawable(get_window(),
		   get_style()->get_black_gc(),
		   0, 0, 0, 0, image2->get_width(),
		   image2->get_height(),
		   Gdk::RGB_DITHER_NONE,0, 0);
	}
	/* < Paint image */
	for (unsigned i=0;i< imgVec.size();i++){
		double scale=imgVec.at(i).scale;
		Glib::RefPtr<Gdk::Pixbuf> image4 ;
		try{
		image4 = imgVec.at(i).pixbuf->scale_simple((int)(imgVec.at(i).pixbuf->get_width()*scale)+1,
					(int)(imgVec.at(i).pixbuf->get_height()*scale)+1,Gdk::INTERP_NEAREST);

		image4->render_to_drawable(get_window(),
			   get_style()->get_black_gc(),
			   0, 0, imgVec.at(i).x_draw-image4->get_width(), imgVec.at(i).y_draw - image4->get_height(), image4->get_width(),
			   image4->get_height(),
			   Gdk::RGB_DITHER_NONE,0, 0);
		} catch(...){

		}
		layout->set_font_description(*timesnr_8);
		sprintf(tmp,"%s",imgVec.at(i).text.c_str());
		layout->set_markup(tmp);
		int y_txt=imgVec.at(i).y_draw;
		int x_txt=imgVec.at(i).x_draw - image4->get_width()-5;
		get_window()->draw_layout(Gdk::GC::create(get_window()),x_txt,y_txt, layout);

	}

    for (unsigned i=0;i< goalVec.size();i++){
			gc->set_foreground(cols[goalVec.at(i).col]);
			window->draw_line(gc, roboVec.at(goalVec.at(i).r1).x_draw, roboVec.at(goalVec.at(i).r1).y_draw,
					goalVec.at(i).x_draw,goalVec.at(i).y_draw);
			sprintf(tmp,"(%3.1f,%3.1f)",goalVec.at(i).x,goalVec.at(i).y);
			layout->set_markup(tmp);
			get_window()->draw_layout(Gdk::GC::create(get_window()),goalVec.at(i).x_draw-50,goalVec.at(i).y_draw-20, layout);
			layout->set_markup("*");
			get_window()->draw_layout(Gdk::GC::create(get_window()),goalVec.at(i).x_draw,goalVec.at(i).y_draw-5, layout);
			//if (goalVec.at(i).arrow){
				//draw_triangle(goalVec.at(i).x_draw,goalVec.at(i).y_draw,ang, goalVec.at(i).col, 10);
			//}
	}

	for (unsigned i=0;i< roboVec.size();i++){
		//XXXXXXXX
		//roboVec.at(i).col=0;
		draw_robot(roboVec.at(i).x_draw,roboVec.at(i).y_draw,roboVec.at(i).a,
    						roboVec.at(i).col,roboVec.at(i).has_mouse_over,0.025); //zoom

		//sprintf(tmp,"%s\n(%3.1f:%3.1f)",roboVec.at(i).name.c_str(),roboVec.at(i).x,roboVec.at(i).y);
		sprintf(tmp,"%s",roboVec.at(i).name.c_str());

		layout->set_markup(tmp);

		get_window()->draw_layout(Gdk::GC::create(get_window()),roboVec.at(i).x_draw-20,roboVec.at(i).y_draw+7, layout);
	}

	for (int set = 0; set < LINK_SETS; set++) {
		for (unsigned i = 0; i < linkVec[set].size(); i++) {
			gc->set_foreground(cols[linkVec[set].at(i).col]);
			window->draw_line(gc, roboVec.at(linkVec[set].at(i).r1).x_draw,
					roboVec.at(linkVec[set].at(i).r1).y_draw, roboVec.at(
							linkVec[set].at(i).r2).x_draw, roboVec.at(
							linkVec[set].at(i).r2).y_draw);
			int disp=0;
			if (linkVec[set].at(i).r2 > linkVec[set].at(i).r1){
				disp = 5;
			}else{
				disp = -5;
			}

			int x_text = roboVec.at(linkVec[set].at(i).r1).x_draw
					+ (roboVec.at(linkVec[set].at(i).r2).x_draw - roboVec.at(
							linkVec[set].at(i).r1).x_draw) / 2;
			int y_text = roboVec.at(linkVec[set].at(i).r1).y_draw
					+ (roboVec.at(linkVec[set].at(i).r2).y_draw - roboVec.at(
							linkVec[set].at(i).r1).y_draw) / 2;
			if (linkVec[set].at(i).has_value){
				sprintf(tmp, "%3.1f", linkVec[set].at(i).value);
				layout->set_markup(tmp);
				get_window()->draw_layout(Gdk::GC::create(get_window()), x_text+disp,
						y_text+disp, layout);
			}

/*			double c1 = roboVec.at(linkVec[set].at(i).r2).x - roboVec.at(
					linkVec[set].at(i).r1).x;
			double c2 = roboVec.at(linkVec[set].at(i).r2).y - roboVec.at(
					linkVec[set].at(i).r1).y;
*/

			double ac1 = roboVec.at(linkVec[set].at(i).r2).x_draw - roboVec.at(
					linkVec[set].at(i).r1).x_draw;
			double ac2 = roboVec.at(linkVec[set].at(i).r2).y_draw - roboVec.at(
					linkVec[set].at(i).r1).y_draw;


			double ang = atan2(ac2, ac1);

			double d1 = -15*cos(ang);
			double d2 = -15*sin(ang);

			if (linkVec[set].at(i).arrow) {
				draw_triangle(roboVec.at(linkVec[set].at(i).r2).x_draw+d1,
						roboVec.at(linkVec[set].at(i).r2).y_draw+d2, -ang,
						linkVec[set].at(i).col, 10);
			}
		}
	}
    gc->set_foreground(cols[2]);
	for (unsigned i=0;i< pointVec.size();i++){
            Point p;
            p=pointVec.at(i);
            if (join_points) {
            	Point p1;
            	if (i!=0) p1=pointVec.at(i-1);
            	else p1=p;
            	window->draw_line(gc, p.x_draw, p.y_draw, p1.x_draw,p1.y_draw);
            }else{
            	window->draw_point(gc,p.x_draw,p.y_draw);
            	window->draw_point(gc,p.x_draw,p.y_draw-1);
            	window->draw_point(gc,p.x_draw,p.y_draw+1);
            	window->draw_point(gc,p.x_draw+1,p.y_draw);
            	window->draw_point(gc,p.x_draw-1,p.y_draw);
            }
    }
    SIGNAL(&sem);
}


void area::grabSnapshot(){
	WAIT(&sem);
	Glib::RefPtr<Gdk::Drawable> drwbl = this->get_window();
	Glib::RefPtr<Gdk::Pixbuf> pxb;
	pxb = Gdk::Pixbuf::create(drwbl, drwbl->get_colormap(), 0, 0, 0, 0,
			this->get_width(), this->get_height());
	char name[256];
	sprintf(name, "snapshot_%04d.png", snap_id);
	try {
		pxb->save(name, "png");
		snap_id++;
	} catch (Glib::Exception &e) {
		//std::cerr << "could not save: " << e.what() << std::endl;
	}
	SIGNAL(&sem);

}

void area::clearGoals(){
	WAIT(&sem);
	goalVec.clear();
	SIGNAL(&sem);
}
#include <gdk/gdk.h>

bool area::on_expose_event(GdkEventExpose* e){
	paint();
	return true;
}

void area::setRobotColor(int idx, int c){
	roboVec.at(idx).col=c;
}

