/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: area.h
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
#ifndef _AREA_HH
#define _AREA_HH
#include <gtkmm/drawingarea.h>
#include <semaphore.h>
#include <string>

#define LINK_SETS 5

struct Link{
	int r1,r2;
	int col;
	int arrow;
	float value;
	bool has_value;
};

struct Robot{
	double x,y,a;
	int col;
	int has_mouse_over;
	int x_draw;
	int y_draw;
	std::string name;
};

struct Goal{
	float x,y,a;
	int r1;
	int col;
	int arrow;
	int x_draw;
	int y_draw;
};

struct Img{
	std::string filename, text;
	int scale_type;
	float x,y,scale;
	double sizex, sizey;
	int x_draw;
	int y_draw;
	Glib::RefPtr<Gdk::Pixbuf> pixbuf;
};

struct Point{
	double x,y;
	double cell_size;
	int robot;
	int col;
	int x_draw;
	int y_draw;
};


class area : public Gtk::DrawingArea{
public:
		area();
		int get_selected();
		int addRobot();
		int addRobot(std::string name);
		int getRobotPose(int id, double* x0, double* y0, double* ang);
        int getNumOfRobots();
        int getRobotId(std::string name);
        void setRobotColor(int idx, int c);
		void setRobotPose(int id, double x0, double y0, double ang, int col);
		void addLink(int r1, int r2, int arrow, int color, float value, int set=0);
		void addLink(int r1, int r2, int arrow, int color, int set=0);
		void addPoint(double x, double y, int color, int with_respect_to_robot=-1);
		void addGridPoint(double x, double y, double cell_size, int color, int with_respect_to_robot=-1);
		void addPoint(double y, int color);
		void addLine(double x1, double y1, double x2, double y2,int color, bool permanent=false);
		void addGoal(int r1, double x2, double y2, int color,int arrow);
		int addImage(std::string s, double x, double y, double scale=1.0);
		int addImage(std::string s, double x, double y, double scale, std::string s1);
		int addImage(std::string s, double x, double y, double sizex, double sizey, std::string s1);
		void setBackGroundImage(std::string s);
		void clearLinks(int set=0);
		void clearRobots();
		void clearPoints();
		void clearGoals();
		void setMaximumNumOfPoints(int n);
		void setJoinPoints(bool n);
		void setMargins(double maxx,double maxy, double minx, double miny);
	    void grabSnapshot();
	    void setMaintainAspectRatio(bool val);
	    void waitClick(double &x, double &y);
	    void waitClickInterrupt();
protected:
		std::map<std::string,int> roboMap;
		double xClick,yClick;
		sem_t clickSem;
		Glib::RefPtr<Gdk::Pixbuf> bg_image;
		bool has_bg_image,interrupted;
		bool maintain_aspect_ratio;
		Pango::FontDescription *timesnr_12;
		Pango::FontDescription *timesnr_8;
	  	std::vector<Link> linkVec[LINK_SETS];
	  	std::vector<Robot> roboVec;
	  	std::vector<Goal> goalVec;
	  	std::vector<Img> imgVec;
	  	std::vector<Point> pointVec;
	  	unsigned int snap_id;
	  	pthread_mutex_t sem;
	  	Gdk::Color cols[10];
	    Glib::RefPtr<Gdk::Window> window;
	    Glib::RefPtr<Gdk::GC> gc;
	    double canvas_minx,canvas_miny,canvas_maxx,canvas_maxy;
	    double max_x, max_y, min_x, min_y, scalex,scaley, zoom;
	    int selected, painted, initialized,mouseover;
	    unsigned maximumNumOfPoints;
	    bool join_points,marginSetted;
	    float fixed_scale;
	    bool on_area_button_press_event(GdkEventButton *ev);
        bool on_area_button_release_event(GdkEventButton *ev);
        bool on_area_expose_event(GdkEventExpose *ev);
        virtual bool on_expose_event(GdkEventExpose* e);
        bool timer_callback();
        void draw_robot(double x, double y, double a,int col, int mouse_over, double scale);
        void draw_triangle(double x, double y, double a, int col, double scale);
        virtual void on_realize();
        void compute_environment();
        void paint();
        void on_size_alloc(Gtk::Allocation &al);
        virtual bool on_area_motion_notify_event(GdkEventMotion *ev);
};

#endif

