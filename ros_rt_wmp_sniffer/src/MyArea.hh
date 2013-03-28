/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: MyArea.hh
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

#ifndef _MYAREA_HH
#define _MYAREA_HH
#include "misc.h"
#include <gtkmm/drawingarea.h>



#define DATA_SIZE 100

struct Dot{
	int node;
	int col;
	int x_draw;
	int y_draw;
};



struct Frame{
	unsigned long long time,diff;
	int x_draw;
	int y1_draw;
	int y2_draw;
	int src;
	int dest;
	int col;
	int mouseover;
	int nt;
	int pos_in_file;
	unsigned long serial;
	std::vector<Dot> dot;
	bool mark;
	bool cross;
	bool left_cross;
};


class MyArea : public Gtk::DrawingArea {
private:
		  pthread_mutex_t sem;
		  int num_nodes, selected;
		  double x_max,y_max,y_min,x_min;
		  std::vector<Frame> frameVec;
		  std::vector<bool> activeVec;
	      Gdk::Color cols[32];
	      Glib::RefPtr<Gdk::Window> window;
	      Glib::RefPtr<Gdk::GC> gc;
	      int y0,x0;
	      int step;
	      unsigned int max,min;
	      int bytime;
	      int max_size;
	      int mouse_on;
	      void draw_frame(int i);
	public:
		MyArea();
		void paint();
		void set_max_size(int _max_size);
		void clean_info();
		void clean_window();
	    void beginInsert();
	    int insertOne(int from, int to, long time ,int pos_in_file,int col, int nt);
	    int insertOne(int from, int to, unsigned long long time ,int pos_in_file,unsigned long serial,int col, int nt);
	    void endInsert();
		void setNumOfNodes(int n);
		void compute_environment();
		int get_y_pos(int node_id);
		void insert(int from, int to, long time ,int col);
		int get_selected();
		int get_offset_of_selected();
		int get_older_offset();
		void delete_older();
	    int get_selected(int & from, int& to, int &col);
	    int set_selected(int n);
	    long get_selected_time();
	    long get_mouse_on_time();
	    int get_mouse_on_serial();
	    int get_selected_serial();
	    void addDot(int frame, int node, int color);
	    void addMark(int frame);
	    void addCross(int frame);
	    void addLeftCross(int frame);
	    void resetActive();
	    void setActive(int i);
	protected:

	    bool timer_callback();
	    virtual void on_realize();
		bool on_MyArea_motion_notify_event(GdkEventMotion* event);
	    virtual bool on_expose_event(GdkEventExpose* e);
	    virtual bool on_MyArea_button_press_event(GdkEventButton *ev);
	    virtual bool on_MyArea_button_release_event(GdkEventButton *ev);
};

#endif
