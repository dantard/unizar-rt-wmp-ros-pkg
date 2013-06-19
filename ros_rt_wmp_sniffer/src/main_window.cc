/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: main_window.cc
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

#define PF(p) //p
/****PF** **/
#include <iostream>
#include <sstream>
#include <string>
#include <gtkmm/dialog.h>
#include <window1.hh>
#include <cstdio>
#include <cstdlib>
#include "misc.h"
#include "wmp_config.h"
#include "core/interface/Msg.h"
#include "math.h"
#include "bridge.hh"
#include "rec_param_dlg.hh"
#include "area_wrapper.h"
#include "wmp_specific.hh"
#include "statistics.h"
#include "wmp_config.h"

#include "core/include/frames.h"
#include "config.h"
#include "main_window.hh"
#include "area.h"
#include "MyArea.hh"
#include "prim.h"
#include "pcap_layer.h"
#include <iostream>
#include <fstream>
#include <gtkmm/main.h>
#include <gtkmm/messagedialog.h>
#include <gtkmm/window.h>
#include <iostream>
#include <gtkmm/filechooserdialog.h>
#include <gtkmm/stock.h>
#include <gtkmm/widget.h>
#include <glibmm/keyfile.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include "enhanced_io.h"
#include <iomanip>
#include <vte-0.0/vte/vte.h>
#include <fcntl.h>

using namespace std;

class main_window* mw;
extern char sniff_iface[16];
extern char iface[16];


std::string nts(char val) {
	std::ostringstream oss;
	oss << (int) val;
	return oss.str();
};
std::string nts(unsigned char val) {
	std::ostringstream oss;
	oss << (int) val;
	return oss.str();
};

template<class T>
std::string nts(T val) {
	std::ostringstream oss;
	oss << (T) val;
	return oss.str();
};

template<class T>
std::string hnts(T val) {
	std::ostringstream oss;
	oss << std::hex;
	oss << val;
	return oss.str();
};




void dec2bin(long decimal, char *binary) {
	PF(fprintf(stderr,"**PF**  dec2bin \n");)
	int k = 0, n = 0;
	int remain;
	char temp[80];
	do {
		remain = decimal % 2;
		decimal = decimal / 2;
		temp[k++] = remain + '0';
	} while (k < 8);
	while (k >= 0)
		binary[n++] = temp[--k];
	binary[n - 1] = 0; // end with NULL
}



void print_path(char * path, int dest) {
	char buf[10000];
	sprintf(buf,"Digraph Path {\n");
	int prev_message_type = 0, previous = 0;
	char len = *path;
	char * p = &path[1];
	path[(unsigned int) len] = 40 + dest;
	int hops = 1;
	for (int i = 0; i < len; i++) {
		int message_type = *p / 10;
		int dest = *p % 10;


		if (i==0){
			previous = dest;
			prev_message_type = message_type;
			p++;
			continue;
		}
		char aux[1000];
		char color[10];
		switch (prev_message_type) {
		case 2:
			strcpy(color, "red");
			break;
		case 3:
			strcpy(color, "green");
			break;
		case 4:
			strcpy(color, "blue");
			break;
		case 5:
			strcpy(color, "black");
			break;
		}
		/* If current is message and previous token, means that this node has authorized itself */
		sprintf(aux, "\tnode%d -> node%d [color=%s, label=%d];\n",
				previous, dest, color, hops);

		strcat(buf, aux);
		if ((message_type == 4) && (prev_message_type == 2) && prev_message_type != -1) {
			sprintf(aux, "\tnode%d -> node%d [color=green, label=%d];\n",
					dest, dest, hops+1);
			strcat(buf, aux);
			hops++;
		}
		p++;
		hops++;
		previous = dest;
		prev_message_type = message_type;

	}
	strcat(buf, "}\n");
	char txt[1000];
	sprintf(txt,"echo \"%s\" | dot -Tpng | display &",buf);
	//FILE* ff = popen("dot -Tpng | display &","w");
	//fprintf(ff,"%s\n",buf);
	//fclose(ff);
	int ret = system (txt);
	if (ret!=0){
		fprintf(stderr,"Error executing %s\n", txt);
	}
}

void main_window::on_hscale_position_value_changed() {
	//fprintf(stderr,"**PF**  main_window::on_hscale_position_value_changed \n");
	if (playing || recording)
		return;
	fill_screen();
}

void main_window::on_hscale_zoom_value_changed() {
	PF(fprintf(stderr,"**PF**  main_window::on_hscale_zoom_value_changed \n");)
	if (recording || playing){
		return;
	}
	fill_screen();
}
bool main_window::on_main_window_expose_event(GdkEventExpose *ev) {
	 if ( Gdk::WINDOW_STATE_MAXIMIZED & get_state() )
	        cerr << "trueaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa" << endl;
return false;

}

void main_window::fill_tree(wmpFrame * r, long long ptime, int pos_in_file) {
	Gtk::TreeModel::Children children = m_refTreeModel->children();
	for (Gtk::TreeModel::Children::iterator iter = children.begin(); iter
			!= children.end(); ++iter) {
		Gtk::TreeModel::Row row = *iter;
		string exp_name((Glib::ustring(row[m_Columns.m_col_name])).c_str());
		Gtk::TreeModel::Path p = m_refTreeModel->get_path(iter);
		rows_expanded[exp_name] = xfield_tw->row_expanded(p);
	}

	m_refTreeModel->clear();
	double ms;
	int sec;
	sec = (int) ((double) ptime / 1000000.0);
	ms = (double) (ptime % 1000000) / 1000.0;
	row_append("Time", nts(sec) + "s " + nts(ms) + "ms");
	if (pos_in_file >= 0)
		row_child_append("Id", nts(pos_in_file));
	update_frame_labels(r);
	show_routing_data(r);
	show_qos_data(r);
	show_broadcast_data(r);

	children = m_refTreeModel->children();
	for (Gtk::TreeModel::Children::iterator iter = children.begin(); iter
			!= children.end(); ++iter) {
		Gtk::TreeModel::Row row = *iter;
		string exp_name((Glib::ustring(row[m_Columns.m_col_name])).c_str());
		bool to_expand = rows_expanded[exp_name];
		if (to_expand) {
			Gtk::TreeModel::Path p = m_refTreeModel->get_path(iter);
			xfield_tw->expand_row(p, true);
		}
	}

}

void main_window::updater(){
	while(1){
		usleep(100000);
		gdk_threads_enter();
		areadx->queue_draw();
		areasx->queue_draw();
		MyArea->queue_draw();
		gdk_threads_leave();

		timered_show_t = true;
		idx3++;


		if (playing) {
			continue_play();
			continue ;
		}
		if (recording) {
			int best, actual;
			char txt[128];
			simData_Hdr * p = (simData_Hdr *) data_backup;
			frame_time = p->time;
			get_actual_gct(&best, &actual);

			sprintf(txt, "%llds %5.3fms", frame_time / 1000000, (float) (frame_time
					% 1000000) / 1000.0);

			gdk_threads_enter();
			label68->set_text(txt);
			sprintf(txt, "Frames: %d", play_idx);
			label69->set_text(txt);
			sprintf(txt, "Max EFZ: %d", best);
			label70->set_text(txt);
			sprintf(txt, "Present EFZ: %d", actual);
			label71->set_text(txt);
			gdk_threads_leave();

			timer++;
			if (timer % 10 == 0) {
				MyArea->resetActive();
				if (w2 && w2->is_visible()) {
					w2->m_refTreeModel->clear();
					statistics_publish(w2);
				}
			}
			continue ;
		}
	}
}

void main_window::notify_selection(int n) {
	PF(fprintf(stderr,"**PF**  main_window::notify_selection \n");)
	int v = (int) (n - hscale_zoom->get_value() / 2);
	if (v < 0){
		v = 0;
	}
	hscale_position->set_value(v);
	io_go_to(n);
	char data[2500]; //TODO: warning
	int bytes = io_read_next_sim_data(&data[0]);
	if (bytes > 0) {
		draw_stuffs(&data[0], n, 0, 1, 1);

	}
	MyArea->set_selected(n);
}

void main_window::reset(int val) {
	char txt[256];
	sprintf(txt, "wmpSniffer - %s", filename);
	this->set_title(txt);
	kf->set_string("Files", "Last_Opened" ,filename);
	nnodes = io_open_sim_data(filename);
	if (val!=-1){
		nnodes = val;
	}

	areadx->clearRobots();
	areasx->clearRobots();
	if (nnodes == 0){
		nnodes = kf->get_integer("Recording", "Num_Of_Nodes");
	}
	if (nnodes > 0) {
		int n = io_get_sim_data_num_of_elements();
		double dist = 150;
		float ang = 2.0 * M_PI / nnodes;
		for (int i = 0; i < nnodes; i++) {
			areadx->addRobot("N" + nts(i));
			int xp = (int) (dist * cos((float) i * ang));
			int yp = (int) (dist * sin((float) i * ang));
			areadx->setRobotPose(i, alignment21->get_width()/2 + xp, yp, 0, 0);
		}

		for (int i = 0; i < nnodes; i++) {
			areasx->addRobot("N" + nts(i));
			int xp = (int) (dist * cos((float) i * ang));
			int yp = (int) (dist * sin((float) i * ang));
			areasx->setRobotPose(i, 	alignment21->get_width()/2 + xp, yp, 0, 0);
		}

		MyArea->setNumOfNodes(nnodes);
		if (n - 2 > 0){
			hscale_position->set_range(0, n - 2);
			//XXX: n-2
			spinbutton1->set_range(0, 1000000000);
			hscale_position->set_sensitive(true);
			spinbutton1->set_sensitive(true);
			button23->set_sensitive(true);
			button6->set_sensitive(true);
			button7->set_sensitive(true);
		} else{
			hscale_position->set_range(0,100);
			hscale_position->set_sensitive(false);
			spinbutton1->set_range(0, 100);
			spinbutton1->set_sensitive(false);
			button23->set_sensitive(false);
			button6->set_sensitive(false );
			button7->set_sensitive(false);
		}
		hscale_zoom->set_range(20, 100);
		hscale_position->set_value(0);
		hscale_zoom->set_value(50);
		this->fill_screen();
	} else {
		areadx->clearLinks();
		areadx->clearRobots();

		areasx->clearLinks();
		areasx->clearRobots();

		MyArea->clean_window();
		MyArea->setNumOfNodes(4);
		hscale_position->set_range(0, 100);
		hscale_zoom->set_value(50);
		hscale_zoom->set_range(20, 100);
	}
}

void main_window::on_w_resize(GdkEventConfigure* event) {
	//hpaned->set_position((int) (0.85 * this->get_width()));
	//vpaned1->set_position((int) (0.65 * this->get_height()));
	//hpaned1->set_position((int) (0.40 * hpaned->get_width()));
	//vpaned1->set_position(vpaned1->get_height()*0.7);
	//hpaned1->set_position(hpaned1->get_width()*0.5);
	//hpaned->set_position(hpaned->get_width()*0.7);


	//hpaned1->set_position(hpaned1->get_width()*0.5);


	sigc::slot<bool> my_slot = sigc::bind(sigc::mem_fun(*this,&main_window::MyCallback), 1);

	  // This is where we connect the slot to the Glib::signal_timeout()
	sigc::connection conn = Glib::signal_timeout().connect(my_slot,
	          50);

}
bool main_window::MyCallback(int n) {
	vpaned1->set_position(vpaned1->get_height()*0.7);
	hpaned->set_position(hpaned->get_width()*0.7);
	double dist = 150;
	float ang = 2.0 * M_PI / nnodes;
	for (int i = 0; i < nnodes; i++) {
		int xp = (int) (dist * cos((float) i * ang));
		int yp = (int) (dist * sin((float) i * ang));
		areadx->setRobotPose(i, alignment21->get_width() / 2 + xp, yp, 0, 0);
	}

	for (int i = 0; i < nnodes; i++) {
		int xp = (int) (dist * cos((float) i * ang));
		int yp = (int) (dist * sin((float) i * ang));
		areasx->setRobotPose(i, alignment21->get_width() / 2 + xp, yp, 0, 0);
	}
	return false;
}

main_window::~main_window() {

}
#include "icon.h"
main_window::main_window(char * s_iface) :
	main_window_glade() {
	set_icon(Gdk::Pixbuf::create_from_xpm_data(icon));
	PF(fprintf(stderr,"**PF**  main_window::main_window \n");)
	prim_cb->hide();
	mw = this;
	w1 = NULL;
	w2 = NULL;
	k = playing = recording = 0;
	//textview3->modify_font(Pango::FontDescription("Courier"));
	//textview4->modify_font(Pango::FontDescription("Courier"));
	/* set-up areas */
	struct stat st;
	ostringstream oss,oss2;
	oss << getenv("HOME") << "/.wmpSniffer";
	oss2 << oss.str();

	if(stat(oss.str().c_str(),&st) != 0){
		std::cerr << "Creating Directory..." << oss.str().c_str() << std::endl;
		mkdir(oss.str().c_str(),0x777);
	}
	oss << "/wmpSniffer.conf";
	oss2 << "/active";

	FILE * flock = fopen(oss2.str().c_str(),"w");
	if (flock!=0) {
		fclose(flock);
	}

	cfg_file = oss.str();

	areadx = new class area();
	areasx = new class area();
	areadx->setMargins(200, 200, -200, -200);
	areasx->setMargins(200, 200, -200, -200);
	//vbox9->pack_end(*area);
	//XXXXXXXXXXXXXXXX:::::::::::::: next tree align2d->add(*area);

	alignment22->add(*areasx); //sinistra
	alignment21->add(*areadx); //destra
	nt_frame->set_visible(false);

	MyArea = new class MyArea();
	//vpaned1->pack2(*MyArea, Gtk::EXPAND | Gtk::SHRINK);
	align1d->add(*MyArea);

	this->signal_configure_event().connect_notify(sigc::mem_fun(this,
			&main_window::on_w_resize));


	this->signal_scroll_event().connect(sigc::mem_fun(*this,
			&main_window::on_main_window_scroll_event), false);
	Glib::signal_timeout().connect(sigc::mem_fun(*this,
			&main_window::timer_callback), 100);
	this->add_events(Gdk::EXPOSURE_MASK | Gdk::POINTER_MOTION_MASK
			| Gdk::POINTER_MOTION_HINT_MASK | Gdk::BUTTON_MOTION_MASK
			| Gdk::BUTTON1_MOTION_MASK | Gdk::BUTTON2_MOTION_MASK
			| Gdk::BUTTON3_MOTION_MASK | Gdk::BUTTON_PRESS_MASK);
	m_refTreeModel = Gtk::TreeStore::create(m_Columns);
	xfield_tw->set_model(m_refTreeModel);
	xfield_tw->append_column("Message                       ",
			m_Columns.m_col_name);
	xfield_tw->append_column("Value", m_Columns.m_col_value);

	MyArea->show();
	areadx->show();
	areasx->show();

	io_init();
	time_t tiempo;
	struct tm *tmPtr;
	tiempo = time(NULL);
	tmPtr = localtime(&tiempo);
	//strftime(filename, 256, "rt-wmp-%H-%M-%S.vis", tmPtr);
	//	outf=fopen(cad,"w+");

	kf = new Glib::KeyFile();
	try {
		kf->load_from_file(cfg_file.c_str());
	} catch (...) {
		cerr << "Config file not found, using default values..." << endl;
		kf->set_integer("Recording", "Num_Of_Nodes", 7);
		kf->set_integer("Recording", "Rec_Type", 1);
		kf->set_string("Recording", "Filename", "");
		kf->set_string("Recording", "Executable", "");
		kf->set_integer("Recording", "Shmem_compile", 1);
		kf->set_integer("Recording", "Shmem_execute", 1);
		kf->set_string("Recording", "Args", "%i %n");
		kf->set_string("Files", "Last_Opened", "rt-wmp.vis");
		kf->set_string("Interfaces", "Sniffer", "wlan0");
		kf->set_string("Interfaces", "Remote_Sniffer", "eth0");
		kf->set_integer("Gui", "Show_Prim", 1);
		kf->set_integer("Gui", "Show_2D", 1);
		kf->set_integer("Gui", "Show_Text", 0);
		kf->set_integer("Gui", "Show_Frames", 1);
		kf->set_integer("Gui", "Show_Foreign", 0);
	}
	int val = kf->get_integer("Gui", "Show_Prim");
	prim_cb->set_active(val);
	val = kf->get_integer("Gui", "Show_Foreign");
	foreign_cb->set_active(val);
	val=kf->get_integer("Gui", "Show_2D");
	d2_cb->set_active(val);
	val=kf->get_integer("Gui", "Show_Text");
	text_cb->set_active(val);
	val=kf->get_integer("Gui", "Show_Frames");
	d1_cb->set_active(val);

	if (s_iface != NULL){
		kf->set_string("Interfaces", "Sniffer" ,s_iface);
	}

	Glib::ustring s = kf->get_string("Interfaces", "Sniffer");
	sprintf(iface,"%s",s.c_str());
	s = kf->get_string("Interfaces", "Remote_Sniffer");
	sprintf(sniff_iface,"%s",s.c_str());

	s = kf->get_string("Files", "Last_Opened");
	sprintf(filename,"%s",s.c_str());
	reset();

	Gtk::Menu::MenuList& menulist = m_Menu_Popup.items();
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Show_path", sigc::mem_fun(
			*this, &main_window::show_path)));
	m_Menu_Popup.accelerate(*this);
	xfield_tw->signal_button_press_event().connect(sigc::mem_fun(*this, &main_window::on_xfield_tw_button_press_event), false);
	xfield_tw->modify_font(Pango::FontDescription("Courier"));

	hpaned->set_position(hpaned->get_width()*0.7);
	vpaned1->set_position(vpaned1->get_height()*0.7);
	hpaned1->set_position(hpaned1->get_width()*0.5);

	timer = idx3 = 0;
	Glib::Thread *const updater = Glib::Thread::create(
	      sigc::mem_fun(this, &main_window::updater), true);
	w3 = 0;
}
void main_window::show_path(){
#ifdef ENABLE_WMP_DEBUG
	if (selectedFrame.hdr.type==MESSAGE){
		print_path(selectedFrame.hdr.path,selectedFrame.msg.dest);
	}
#endif
}

bool main_window::on_xfield_tw_button_press_event(GdkEventButton *ev) {
	if ((ev->type == GDK_BUTTON_PRESS) && (ev->button == 3)) {
		if (selectedFrame.hdr.type==MESSAGE){
			m_Menu_Popup.popup(ev->button, ev->time);
		}
	}
	return false;
}

void getDataString(void * q, int size, char * txt){
	if (size > 2500) size = 2500;
char *k = (char * ) q;
	for (int i=0, j=0;i < size ; i++){
		if (k[i]>=32 &&  k[i]<=126){
			txt[j] = k[i];
		} else{
			txt[j] = '.';
		}
		j++;
		if (j%33==32){
			txt[j] = '\n';
			j++;
		}
		txt[j] = 0;
	}
}

void main_window::update_frame_info(simData_Hdr * p, wmpFrame * q) {
	row_append("Frame Info", "");
	row_child_append("Type", nts(p->frame_type));
	row_child_append("Len", nts(p->len) + "B");
	//row_child_append("Rate", nts(p->rate) + " Mbps");
	row_child_append("Num nodes", nts(p->num_nodes));
	//row_child_append("Key", nts((int) p->key));
	//row_child_append("Data source", nts(p->data_src));
	double time = (double) p->time / 1000;
	row_child_append("Time", nts(time));
	row_child_append("Protocol", "0x"+hnts(p->proto));
	char txt[p->len + 1000];
	getDataString(q,p->len,txt);
	row_child_append("RAW", txt);
}

void main_window::saveRowsStatus() {
	Gtk::TreeModel::Children children = m_refTreeModel->children();
	for (Gtk::TreeModel::Children::iterator iter = children.begin(); iter
			!= children.end(); ++iter) {
		Gtk::TreeModel::Row row = *iter;
		string exp_name((Glib::ustring(row[m_Columns.m_col_name])).c_str());
		Gtk::TreeModel::Path p = m_refTreeModel->get_path(iter);
		rows_expanded[exp_name] = xfield_tw->row_expanded(p);
	}
}

void main_window::restoreRowsStatus() {
	Gtk::TreeModel::Children children = m_refTreeModel->children();
	for (Gtk::TreeModel::Children::iterator iter = children.begin(); iter
			!= children.end(); ++iter) {
		Gtk::TreeModel::Row row = *iter;
		string exp_name((Glib::ustring(row[m_Columns.m_col_name])).c_str());
		bool to_expand = rows_expanded[exp_name];
		if (to_expand) {
			Gtk::TreeModel::Path p = m_refTreeModel->get_path(iter);
			xfield_tw->expand_row(p, true);
		}
	}
}

void main_window::draw_stuffs(char * data, int pos_in_file, int show_f,
		int show_2D_f, int show_t) {
	PF(fprintf(stderr,"**PF**  main_window::draw_stuffs \n");)
	simData_Hdr * p = (simData_Hdr *) data;
	wmpFrame * r = (wmpFrame *) (data + sizeof(simData_Hdr));

	frame_time = p->time;
	selectedFrame = *r;
	MyArea->setActive(r->hdr.from);

	if (show_t) {
		//if (r->hdr.type == TOKEN){
			double ms;
			int sec;
			sec = (int) ((double) p->time / 1000000.0);
			ms = (double) (p->time % 1000000) / 1000.0;

			saveRowsStatus();
			m_refTreeModel->clear();
			row_append("Time", nts(sec) + "s " + nts(ms) + "ms");
			row_child_append("Id", nts(pos_in_file));
			update_frame_info(p, r);

			if (p->frame_type != SP_FOREIGN) {
				update_frame_labels(r);
				show_routing_data(r);
				show_qos_data(r);
				show_broadcast_data(r);
				// TODO: fill_tree(r,p->time,pos_in_file);
			}
			restoreRowsStatus();
		//}
	}

	if (show_f) {
		int col = r->hdr.type;
		int nt = (r->hdr.type == TOKEN);// && r->tkn.new_token;
		if (p->frame_type == SP_LUS_WMP_FRAME_DUP) {
			col = 9;
			MyArea->insertOne(r->hdr.from, r->hdr.to, p->time, pos_in_file,
					r->hdr.serial, col, nt);
			k++;
		} else if (p->frame_type == SP_FOREIGN) {
			col = 8;
			nt = 2;
			MyArea->insertOne(0, p->num_nodes - 1, p->time, pos_in_file, 0,
					col, nt);
			k++;
		} else {
			if (r->hdr.type == MESSAGE){
				if (!mBitsIsSet(r->msg.type, 4)){
					col = 6;
				}
#ifdef WMP_ROUTING_tree
				if (!mBitsIsSet(r->msg.type,4)){
					col = 10;
				}
#endif
			}
			if (r->hdr.type == DROP_TOKEN){
				col = 0;
			}
			if (r->hdr.type == ACK){
				col = 1;
			}
			int f_idx = MyArea->insertOne(r->hdr.from, r->hdr.to, p->time,
					pos_in_file, r->hdr.serial, col, nt);
			k++;
			if (r->hdr.retries > 0) {
				MyArea->addMark(f_idx);
			}
			if (p->time_source == 99) {
				MyArea->addLeftCross(f_idx);
			}

			//new
			//for (int i = 0; i < nnodes; i++) {
				//if (u->reached) {
					MyArea->addDot(f_idx, r->hdr.from, 0);
				//}
				//u++;
			//}
			//new
#ifdef ENABLE_WMP_DEBUG
			if (r->hdr.last_to >0) {
				MyArea->addCross(f_idx);
			}
#endif
		}
	}
	//XXX:
#ifdef ENABLE_WMP_DEBUG
	if (r->hdr.sender_pow >= 0){
		areadx->setRobotPose(r->hdr.from,10*r->hdr.sender_x, 10*r->hdr.sender_y, 0, 0);
		areasx->setRobotPose(r->hdr.from,10*r->hdr.sender_x, 10*r->hdr.sender_y, 0, 0);
	} else {
		for (int i = 0; i < nnodes; i++) {
			if (q->pose_is_valid) {
				areadx->setRobotPose(i, q->x, q->y, q->a, 0);
				areasx->setRobotPose(i, q->x, q->y, q->a, 0);
			} else {
				float dist = 100;
				float ang = 2.0 * M_PI / nnodes;
				int xp = (int) (dist * cos((float)i * ang));
				int yp = (int) (dist * sin((float)i * ang));
				//area->setRobotPose(i, xp, yp, 0, 0);
			}
			q++;
		}
	}
#endif
	if (show_2D_f) {
		if (p->frame_type != SP_FOREIGN) {
			areadx->clearLinks(2);
			areasx->clearLinks(2);
//			if (r->tkn.new_token){
//				areadx->clearLinks();
//				areasx->clearLinks();
//				areadx->clearLinks(1);
//				areasx->clearLinks(1);
//			}
#ifdef WMP_ROUTING_tree
				areadx->clearLinks(1);
				areasx->clearLinks(1);
				char * p1;

				/* area2->current tree */
				areasx->addLink(r->hdr.from, r->hdr.to,1, r->hdr.type, 2);

				/* CURRENT TREE */
				p1 = wmp_get_frame_routing_pointer(r, nnodes);
				for (int i = 0; i < nnodes; i++) {
					if (p1[i]>=0 && p1[i]<32) {
						//fprintf(stderr,"i:%d p[i]:%d\n",i,p1[i]);
						areasx->addLink(i,p1[i],0,0,1,1);
					}
				}

				if (r->hdr.type == TOKEN){
					p1 = wmp_get_frame_routing_pointer(r, nnodes) + 2 * nnodes;

					/* NEXT TREE */
					for (int i = 0; i < nnodes; i++) {
						if (p1[i]>=0 && p1[i]<32)  {
							areadx->addLink(i, p1[i], 0, 0, 1, 1);
						}
					}
				} else{
					areadx->clearLinks();
					//area2->clearLinks();
				}



				for (int i = 0; i < nnodes; i++) {
					if (i == r->hdr.to) {
						areadx->setRobotColor(i, r->hdr.type);
						areasx->setRobotColor(i, r->hdr.type);
					} else {
						areadx->setRobotColor(i, 0);
						areasx->setRobotColor(i, 0);
					}
				}
#else



			if (r->hdr.type == TOKEN) {
				areadx->clearLinks(0);
				areadx->clearLinks(1);
				areasx->clearLinks(0);
				areasx->clearLinks(1);
				char * p1 = wmp_get_frame_routing_pointer(r, nnodes);
				char p2[nnodes*nnodes];
				memcpy(p2,p1,nnodes*nnodes);
				if (prim_cb->get_active()) {
					try {
						prim(p1, nnodes); //modifica directamente la trama
					} catch (...) {

					}
				}
				/* DRAW prim links on dx area */
				int idxxx=0;
				for (int i = 0; i < nnodes; i++) {
					for (int j = 0; j < nnodes; j++) {
						if ((*p1) && i != j) {
							float val = (float) (p2[idxxx]);
							areadx->addLink(i, j, 0, 0, val, 1);
						}
						p1++;
						idxxx++;
					}
				}
				/* DRAW all links on sx area */
				idxxx=0;
				char *p3 = p2;
				for (int i = 0; i < nnodes; i++) {
					for (int j = 0; j < nnodes; j++) {
						if ((*p3) && i != j) {
							float val = (float) (*p3);
							areasx->addLink(i, j, 0, 0, val, 1);
						}
						p3++;
						//idxxx++;
					}
				}
			}
			int color=r->hdr.type;
			if (color==6){
				color = 1;
			}

			areadx->addLink(r->hdr.from, r->hdr.to,1, color,2);
			areasx->addLink(r->hdr.from, r->hdr.to,1, color,2);
#endif


			for (int i = 0; i < nnodes; i++) {
				if (i == r->hdr.from || i == r->hdr.to){
					areadx->setRobotColor(i, r->hdr.type);
				}else{
					areadx->setRobotColor(i, 0);
				}
			}
		}
	}
	//updater();
}

void main_window::begin_play() {
	PF(fprintf(stderr,"**PF**  main_window::begin_play \n");)
	int pos = (int) hscale_position->get_value();
	hscale_position->set_sensitive(false);
	hscale_zoom->set_sensitive(false);
	MyArea->clean_window();
	io_go_to(pos);
	play_idx = 0, play_idx2 = pos;
	playing = 1;
}

void main_window::statistics(char * text) {
PF(fprintf(stderr,"**PF**  main_window::statistics \n");)}

void main_window::continue_play() {
	PF(fprintf(stderr,"**PF**  main_window::continue_play \n");)
	char data[2500];
	int bytes = io_read_next_sim_data(&data[0]);
	if (bytes > 0) {
		int elem = (int) hscale_zoom->get_value();
		if (play_idx > elem)
			MyArea->delete_older();
		int n = MyArea->get_older_offset();
		if (n >= 0)
			hscale_position->set_value(n);
		draw_stuffs(&data[0], play_idx2, 1, 1, 1);
		play_idx++;
		play_idx2++;
	} else
		stop_playing();
}

void main_window::stop_playing() {
	PF(fprintf(stderr,"**PF**  main_window::stop_playing \n");)
	playing = 0;
	togglebutton1->set_active(false);
	hscale_position->set_sensitive(true);
	hscale_zoom->set_sensitive(true);
}

#include "window3.hh"

int shmem_pre_init();

bool main_window::begin_online_record() {
	PF(fprintf(stderr,"**PF**  main_window::begin_record \n");)

	if (w2){
		w2->hide();
		delete(w2);
		w2 = 0;
	}

	rp_dlg = new class rec_param_dlg();

	int nn = kf->get_integer("Recording", "Num_Of_Nodes");
	rp_dlg->num_node_spin->set_value(nn);
	int rec_type_rb = kf->get_integer("Recording", "Rec_Type");

	string args = kf->get_string("Recording", "Args");
	rp_dlg->param_txt->set_text(args);


	string shmem_file = kf->get_string("Recording", "Executable");
	if (rec_type_rb == 1) {
		rp_dlg->real_rb->set_active();
	} else if (rec_type_rb == 3) {
		rp_dlg->marte_rb->set_active();
	}else if (rec_type_rb == 4) {
		rp_dlg->shmem_rb->set_active();
	}
	rp_dlg->compile_cb->set_active(kf->get_integer("Recording", "Shmem_compile"));
	rp_dlg->exec_cb->set_active(kf->get_integer("Recording", "Shmem_execute"));

	rp_dlg->shmem_file->set_text(shmem_file.c_str());
	rp_dlg->filename_txt->set_text(this->filename);

	int res = rp_dlg->run();
	if (res == Gtk::RESPONSE_CANCEL) {
		delete (rp_dlg);
		return false;
	}

	nn = rp_dlg->num_node_spin->get_value_as_int();
	kf->set_integer("Recording", "Num_Of_Nodes", nn);

	bool compilee = rp_dlg->compile_cb->get_active();
	bool executee = rp_dlg->exec_cb->get_active();
	kf->set_integer("Recording", "Shmem_compile", rp_dlg->compile_cb->get_active());
	kf->set_integer("Recording", "Shmem_execute", rp_dlg->exec_cb->get_active());
	kf->set_integer("Recording", "Rec_Type", rec_type_rb);
	kf->set_string("Recording", "Executable",rp_dlg->shmem_file->get_text());
	kf->set_string("Recording", "Args",rp_dlg->param_txt->get_text());

	save_cfg_file();

	/* file */
	ostringstream oss;
	oss << "rt-wmp-" << nn << "N.vis";

	strcpy(this->filename,rp_dlg->filename_txt->get_text().c_str());
	io_change_file(this->filename);



	this->set_title("wmpSniffer -" + std::string(this->filename));
	kf->set_string("Files", "Last_Opened" ,this->filename);

	hscale_position->set_value(0);
	hscale_position->set_sensitive(false);
	hscale_zoom->set_sensitive(false);
	button23->set_sensitive(false);
	spinbutton1->set_sensitive(false);
	nnodes = nn;
	reset(nn);

	if (rp_dlg->real_rb->get_active()) {
		rec_type = ST_PCAP;
		rec_type_rb = 1;
	} else if (rp_dlg->marte_rb->get_active()) {
		rec_type = ST_MARTE;
		rec_type_rb = 3;
	} else if (rp_dlg->shmem_rb->get_active()) {
		rec_type = ST_SHARED_MEM;
		rec_type_rb = 4;
		if (rp_dlg->shmem_file->get_text() != "") {
			if (compilee || executee) {
				if (w3 == 0) {
					w3 = new window3();
				}
				w3->create(nn);
				w3->show();
				shmem_pre_init();
				w3->compile_and_execute(rp_dlg->compile_cb->get_active(),
						rp_dlg->exec_cb->get_active(),
						rp_dlg->log_cb->get_active(), nn,
						rp_dlg->shmem_file->get_text(),
						rp_dlg->param_txt->get_text());
			}
		}
	}
	rp_dlg->hide();
	delete (rp_dlg);
	reset_actual_gct();
	play_idx = 0;
	if (!begin_record()){
		return false;
	}
	return true;
}

bool main_window::begin_record(){
	recording = 1;
	play_idx = 0;

	sleep(1);
	areadx->clearLinks();
	areadx->clearLinks(1);
	areadx->clearRobots();
	areasx->clearLinks();
	areasx->clearLinks(1);
	areasx->clearRobots();
	MyArea->clean_window();
	MyArea->setNumOfNodes(nnodes);
	double dist = 200;//sim_ysize / 3.0;


	float ang = 2.0 * M_PI / nnodes;
	for (int i = 0; i < nnodes; i++) {
		areadx->addRobot("N" + nts(i));
		int xp = (int) (dist * cos((float) i * ang));
		int yp = (int) (dist * sin((float) i * ang));
		areadx->setRobotPose(i, alignment21->get_width()/2  + xp, yp, 0, 0);
	}


	for (int i = 0; i < nnodes; i++) {
		areasx->addRobot("N" + nts(i));
		int xp = (int) (dist * cos((float) i * ang));
		int yp = (int) (dist * sin((float) i * ang));
		areasx->setRobotPose(i, alignment21->get_width()/2 + xp, yp, 0, 0);
	}

	areadx->clearPoints();
	areadx->addPoint(-sim_xsize / 2, -sim_ysize / 2, 0);
	areadx->addPoint(sim_xsize / 2, sim_ysize / 2, 0);

	areasx->clearPoints();
	areasx->addPoint(-sim_xsize / 2, -sim_ysize / 2, 0);
	areasx->addPoint(sim_xsize / 2, sim_ysize / 2, 0);

	hscale_zoom->set_value(50);
	reset_actual_gct();
	if (!start_bridge(nnodes, rec_type)){
		return false;
	}
	return true;
}

void main_window::continue_record(char * data, int data_size) {
	PF(fprintf(stderr,"**PF**  main_window::continue_record \n");)
	int elem = (int) hscale_zoom->get_value();
	if (play_idx > elem){
		MyArea->delete_older();
	}
	if (!d1_cb->get_active()){
		MyArea->clean_window();
		play_idx = 0;
	}
	draw_stuffs(&data[0], play_idx, d1_cb->get_active(), d2_cb->get_active(), text_cb->get_active() && !tw_has_focus && timered_show_t);
	timered_show_t = false;
	memcpy(data_backup, data, data_size);
	play_idx++;
	simData_Hdr * p = (simData_Hdr *) data;
	wmpFrame * r = (wmpFrame *) (data + sizeof(simData_Hdr));
	/* ONLINE STATISTICS > */
	if (w2 && w2->is_visible()) {
		if (p->is_wmp) {
			statistics_new_frame(r, p->time, play_idx, 1000); /* TODO: 1000 ????*/
		}
	}
}
void main_window::recording_stopped(){
	gdk_threads_enter();
	if (w3) {
		w3->die();
	}
	recording = false;
	hscale_position->set_sensitive(true);
	hscale_zoom->set_sensitive(true);
	gdk_threads_leave();

	sleep(1);

	gdk_threads_enter();
	reset();
	hscale_position->set_sensitive(true);
	hscale_zoom->set_sensitive(true);
	button8->set_sensitive(true);
	button6->set_sensitive(true);
	button7->set_sensitive(true);
	button13->set_sensitive(true);
	button9->set_sensitive(true);
	togglebutton1->set_sensitive(true);
	rec_tb->set_sensitive(true);
	rec_tb->set_active(false);
	gdk_threads_leave();
}

void main_window::request_stop_recording() {
	PF(fprintf(stderr,"**PF**  main_window::stop_recording \n");)
	stop_bridge();
	if (rec_type == ST_SHARED_MEM) {
	}
}

void main_window::fill_screen() {
	PF(fprintf(stderr,"**PF**  main_window::fill_screen \n");)
	int pos = (int) hscale_position->get_value();
	int elem = (int) hscale_zoom->get_value();
	MyArea->clean_window();
	char data[2500];
	k = 0;
	if (io_go_to(pos)<0) {
		return;
	};

	for (int i = 0; i < elem; i++) {
		int bytes = io_read_next_sim_data(&data[0]);

		if (bytes <= 0){
			continue;
		}

		if (i == elem - 1) {
			draw_stuffs(&data[0], pos, 1, 1, 1);
		}else{
			draw_stuffs(&data[0], pos, 1, 1, 0);
		}
		pos++;
	}

}

bool main_window::on_main_window_motion_notify_event(GdkEventMotion *ev) {
	PF(fprintf(stderr,"**PF**  main_window::on_main_window_motion_notify_event \n");)
	update_pose_labels();
	if (playing || recording) return false;
	int v1 = MyArea->get_selected_time();
	int v2 = MyArea->get_mouse_on_time();
	int v3 = MyArea->get_mouse_on_serial();
	int v4 = MyArea->get_selected_serial();
	label71->set_text("");
	label70->set_text("");
	label69->set_text("");
	if (v1 > 0 && v2 > 0) {
		label70->set_text("D: " + nts((int) (v2 - v1) / 1000000) + "s " + nts(
				(double) ((v2 - v1) % 1000000) / 1000.0) + "ms");
	}
	if (v3 > 0) {
		label68->set_text("Serial: " + nts(v3));
		if (v4 > 0) {
			label69->set_text("SD: " + nts(abs((int) (v4 - v3))));
		} else {
			//label69->set_text("Serial: -");
		}
	} else {

	}
	return false;
}

bool main_window::on_main_window_button_press_event(GdkEventButton *ev) {
	PF(fprintf(stderr,"**PF**  main_window::on_main_window_button_press_event \n");)
	if (playing || recording) return true;
	/* here we receive all the clicks on the drawing areas */
	if ((ev->type == GDK_BUTTON_PRESS)) {

		int s = MyArea->get_offset_of_selected();
		if (s >= 0) {
			char data[2500];
			io_go_to(s);
			io_read_next_sim_data(data);




			draw_stuffs(data, s, 0, 1, 1);
		}
		if (ev->button == 3 && selectedFrame.hdr.type == MESSAGE) {
			m_Menu_Popup.popup(ev->button, ev->time);
		}
	}
	return true;
}

void main_window::update_pose_labels() {
	PF(fprintf(stderr,"**PF**  main_window::update_pose_labels \n");)
	double x, y, a;
	int t = areadx->get_selected();
	if (t >= 0) {
		int res = areadx->getRobotPose(t, &x, &y, &a);
		if (res >= 0) {

		}
	}
}
int main_window::checkstring(char * txt) {
	PF(fprintf(stderr,"**PF**  main_window::checkstring \n");)
	//if (strlen(txt)!=(len-1)) return 0;
	for (int i = 0; i < strlen(txt); i++) {
		//fprintf(stderr,"c:%d\n",txt[i]);
		if (txt[i] < (char) 32 || txt[i] > (char) 125){
			return 0;
		}
	}
	return 1;

}
void main_window::show_broadcast_data(wmpFrame *f) {
#ifdef ENABLE_BC_SUPPORT
	if (f->hdr.bc_len > 0) {
		row_append("Broadcast", "");
		row_child_append("BC Type", nts(f->hdr.bc_type));
		if (f->hdr.bc_type == BC_TYPE_PLUS) {
			Simple_bc * sbc =
					(Simple_bc*) wmp_get_frame_tail_pointer(f, nnodes);
			row_child_append("BC Len", nts(sbc->len));
			//fprintf(stderr,"BC LEN:%d nnodes:%d\n",f->hdr.bc_len,nnodes);
			row_child_append("MaxPri", nts(sbc->maxPri));
			row_child_append("MaxPriId", nts(sbc->idMaxPri));
			row_child_append("Age", nts((int) sbc->age));
//			row_child_append("Msg Univ. Id", nts((int) sbc->univ_id));

			//row_child_child_append("Priority",nts(int(f->hdr.qos_ac_pri)));
			row_child_append("Destination", "");
			for (int i = 0; i < 16; i++) {
				int val = (sbc->dest & (unsigned int) pow(2.0, (double) i)) > 0;
				if (val) {
					row_child_child_append("Node", nts(i));
				}
			}
			char txt[sbc->len + 1000];
			getDataString(&sbc->data,sbc->len,txt);
			row_child_append("Data", txt);
		}
	}
	/*	if (f->hdr.bc_type==BC_TYPE_MULTI){
	 Multi_bc * mbc =(Multi_bc*) wmp_get_frame_tail_pointer(f,nnodes);
	 sprintf(txt,"BC Type:%d\nBC Size:%d\n", f->hdr.bc_type,mbc->slot_size);
	 char tmp[512];
	 for (int i=0;i<nnodes;i++){
	 char * pnt=(char*) ((&mbc->data)+mbc->slot_size*i);
	 sprintf(tmp,"\nNode %d: ",i);
	 strcat(txt,tmp);
	 snprintf(tmp,mbc->slot_size,"%s",pnt);
	 strcat(txt,tmp);
	 }
	 }

	 if (f->hdr.bc_type==BC_TYPE_RTEP){
	 Simple_bc * sbc =(Simple_bc*) wmp_get_frame_tail_pointer(f,nnodes);
	 sprintf(txt,"BC Type:%d BC Len:%d MaxPriId:%d\nMSG:%s", f->hdr.bc_type,sbc->len,sbc->idMaxPri,&sbc->data);
	 fprintf(stderr,"%s\n",txt);
	 }
	 */
#endif
}

void main_window::show_routing_data(wmpFrame *f) {
	PF(fprintf(stderr,"**PF**  main_window::show_routing_data \n");)
#ifdef WMP_ROUTING_tree
		if (f->hdr.type == TOKEN) {
		row_append("Status", " ");
		char * cp = wmp_get_frame_routing_pointer_tree(f,nnodes);
		cp += nnodes;
		ostringstream s1, s2;
		for (int i = 0; i < nnodes; i++) {
			s1 << "Node " << i;
			char txt[20];
			dec2bin((unsigned char) cp[i], txt);
			s2 << (char) txt[0] << " " << (char) txt[1] << " " << (char) txt[2]
					<< " " << &txt[3];
			row_child_append(s1.str(), s2.str());
			s1.str("");
			s2.str("");
		}
	}
#else
		if (f->hdr.type == TOKEN){// && f->hdr.routing_type == 0) { //warning

				row_append("Routing", " ");

				char lqm[10000], tmp[2000];
				sprintf(lqm, "%s","");
				char *cp = (char *) f;
				cp += sizeof(Token_Hdr) + sizeof(Token);

				strcat(lqm, "     ");
				for (int i = 0; i < nnodes; i++) {
					sprintf(tmp, "%3d ", i);
					strcat(lqm, tmp);
				}
				strcat(lqm, "\n");

				strcat(lqm, "    ");
				for (int i = 0; i < nnodes; i++) {
					sprintf(tmp, "----");
					strcat(lqm, tmp);
				}
				strcat(lqm, "\n");

				for (int i = 0; i < nnodes; i++) {
					sprintf(tmp, "%2d - ", i);
					strcat(lqm, tmp);
					for (int j = 0; j < nnodes; j++) {
						if (i != j) {
							sprintf(tmp, "%3d ", (int) *cp);
							strcat(lqm, tmp);
						} else {
							strcat(lqm, "  0 ");
						}
						cp++;
					}
					strcat(lqm, "\n");
				}
				xfield_tw->modify_font(Pango::FontDescription("Courier"));
				row_child_append("LQM", lqm);
				row_append("Status", " ");
				row_child_append("Node Id", "R L U NEXT");
				cp = (char *) f;
				cp += sizeof(Token_Hdr) + sizeof(Token);
				ostringstream s1, s2;
				for (int i = 0; i < nnodes; i++) {
					s1 << "Node " << i;
					char txt[20];
					dec2bin((unsigned char) *cp, txt);
					s2 << (char) txt[0] << " " << (char) txt[1] << " " << (char) txt[2]
							<< " " << &txt[3];
					row_child_append(s1.str(), s2.str());
					s1.str("");
					s2.str("");
					cp += (nnodes + 1);
				}
			}
#endif
}

void main_window::update_frame_labels(wmpFrame * f) {

	PF(fprintf(stderr,"**PF**  main_window:: \n");)

#ifdef ENABLE_WMP_DEBUG
	row_append("Debug","");
	//row_child_append("Hash",nts(f->hdr.hash));
	row_child_append("Timeout",nts(f->hdr.timeout));
	row_child_append("Elapsed",nts(abs(f->hdr.last_to)));
	row_child_append("Informing Serial",nts(f->hdr.last_to_serial));
	row_child_append("Informing reported Key",nts(f->hdr.last_to_key));
#ifdef V8
	row_child_append("Debug Text",std::string(f->hdr.debug));
#endif
	int key = f->hdr.serial * 10000 + f->hdr.from * 1000 + f->hdr.to
										* 100 + f->hdr.retries * 10 + f->hdr.type;
	row_child_append("Frame Key",nts(key));
	row_child_append("Live config cmd ",nts(f->hdr.lc_cmd));
	row_child_append("Live config val ",nts(f->hdr.lc_value));

	ostringstream oss;
	row_child_append("Reason",oss.str().c_str());
	int j=0;
	for (double i=0; i<6; i += 1.0, j++) {
		int val = ((((int) f->hdr.reason) & ((int)pow(2.0,i))) > 0);
		if (val) {
			switch (j) {
				case 0: row_child_child_append("SEARCHING","");
				break;
				case 1: row_child_child_append("SEARCH ALL","");
				break;
				case 2: row_child_child_append("NOTHING TO SEND","");
				break;
				case 3: row_child_child_append("ISOLATED","");
				break;
				case 4: row_child_child_append("GOING BACK","");
				break;
				case 5: row_child_child_append("GOING FORWARD","");
				break;
				case 6: row_child_child_append("NET INACTIVITY","");
				break;
			}
		}
	}

	oss.str("");
//	for (int i=0; i< f->hdr.status[0]-1; i++) {
	//	oss << (int) f->hdr.status[i+1] <<" ";
//	}
//	row_child_append("Status",oss.str().c_str());
//	row_child_append("Msg id",nts(f->hdr.msg_id));
	//XXX to be changed
	row_child_append("CPU Time",nts(f->hdr.node_tx_id));

	//row_child_append("Instance Id",nts(f->hdr.instance_id));
	row_append("Position","");
	row_child_append("x",nts(f->hdr.sender_x));
	row_child_append("y",nts(f->hdr.sender_y));
	row_child_append("Power (mw)",nts(f->hdr.sender_pow));

#endif

	/* TREEVIEW > */
	row_append("Header", "");
	row_child_append("Rssi", nts(f->hdr.rssi));
	row_child_append("Noise", nts(f->hdr.noise));
	row_child_append("Type", nts(f->hdr.type));
	row_child_append("Serial", nts((int) f->hdr.serial));
	row_child_append("From", nts((int)f->hdr.from));
	row_child_append("To", nts((int)f->hdr.to));
	row_child_append("Retries", nts(f->hdr.retries));
	row_child_append("Ack", nts(f->hdr.ack));
	row_child_append("Flow Control", nts(f->hdr.fc));
	row_child_append("Loop Id", nts(f->hdr.loop_id));
	row_child_append("Burst", nts(f->hdr.burst));
	row_child_append("Sleep", nts(f->hdr.sleep));
	row_child_append("Waiting", nts(f->hdr.waiting));

	/* TREEVIEW < */

	if (f->hdr.type == TOKEN) {
		row_append("Token", "");
		row_child_append("Beginner", nts((int) f->tkn.beginner));
		row_child_append("Max Priority", nts((int)f->tkn.maxPri));
		row_child_append("Id Max Pri", nts((int)f->tkn.idMaxPri));
		row_child_append("Age", nts((int) f->tkn.age));
		row_child_append("Ack Hash", nts((int) f->tkn.ack_hash));
		row_child_append("Ack Part", nts((int) f->tkn.ack_part));

		//row_child_append("Loop Beginner", nts((int) f->hdr.loop_beginner));

	} else if (f->hdr.type == AUTHORIZATION) {
		row_append("Authorization", "");
		row_child_append("Source", nts(f->aut.src));
		row_child_append("Destination", nts(f->aut.dest));
		row_child_append("Ack Hash", nts(f->aut.ack_hash));
		row_child_append("Ack Part", nts(f->aut.ack_part));
		ostringstream o1;
		for (int i = 0; i < nnodes; i++) {
			o1 << (int) (bool) (mBitsIsSet(f->aut.reached,i) != 0);
		}
		row_child_append("Reached", o1.str());
	} else if (f->hdr.type == MESSAGE) {
		row_append("Message", "");
		row_child_append("Type", "");

		if (mBitsIsSet(f->msg.type,4)) {
			row_child_child_append("Aura", "Msg");
		} else {
			row_child_child_append("Aura", "Auth");
		}
		if (mBitsIsSet(f->msg.type,1)) {
			row_child_child_append("Burst", "Yes");
		} else {
			row_child_child_append("Burst", "No");
		}
		if (mBitsIsSet(f->msg.type,2)) {
			row_child_child_append("Rescheduled", "Yes");
		} else {
			row_child_child_append("Rescheduled", "No");
		}
		row_child_append("Source", nts(f->msg.src));
		row_child_append("Destination", nts(f->msg.dest));
		row_child_append("Port", nts(f->msg.port));
		row_child_append("Priority", nts((int)f->msg.priority));
		row_child_append("Length", nts(f->msg.len));
		row_child_append("Age", nts((int) f->msg.age));

		row_child_append("Msg Hash", nts(f->msg.msg_hash));
		row_child_append("Msg Part ID", nts(f->msg.part_id));

		ostringstream o1;
		for (int i = 0; i < nnodes; i++) {
			o1 << (int) (bool) (mBitsIsSet(f->msg.reached,i) != 0);
		}
		row_child_append("Reached", o1.str());


		char txt[f->msg.len + 100];

		if (f->msg.len > 0){
			getDataString(wmp_get_message_data_pointer(f),f->msg.len,txt);
			row_child_append("Data", txt);
		}


//		row_child_append("Msg Id", nts(f->hdr.msg_id));
#ifdef		ENABLE_BC_SUPPORT
		if (f->hdr.bc_extra == 77){
			row_child_append("Rescheduled","Yes");
		}else{
			row_child_append("Rescheduled","No");
		}
#endif
#ifdef ENABLE_WMP_DEBUG
		ostringstream o1;
		for (int i = 0; i < nnodes; i++) {
			o1 << (int) (bool) (mBitsIsSet(f->msg.reached,i) != 0);
		}
		row_child_append("Reached", o1.str());

		row_child_append("PATH: ", nts(f->hdr.path[0]));
		if (f->hdr.path[0] < 32) {
			for (int i = 1; i <= f->hdr.path[0]; i++) {
				if (f->hdr.path[i] >= 40) {
					row_child_child_append("Msg", nts(int(f->hdr.path[i] - 40)));
				} else if (f->hdr.path[i] >= 30) {
					row_child_child_append("Auth",
							nts(int(f->hdr.path[i] - 30)));
				} else if (f->hdr.path[i] >= 20) {
					row_child_child_append("Token", nts(
							int(f->hdr.path[i] - 20)));
				}
			}
		}
#endif
	} else if (f->hdr.type == DROP_TOKEN) {
		row_append("Drop", "");
		row_child_append("Drop Serial", nts(f->drop.drop_serial));
	}else if (f->hdr.type == ACK) {
		row_append("Ack", "");
		row_child_append("Ack Serial", nts(f->drop.drop_serial));
	}
}

void main_window::show_qos_data(wmpFrame * f) {
#ifdef ENABLE_QOS_SUPPORT
	if (f->hdr.subtype == 97) {
		row_append("QOS", "");
		row_child_append("Loop Id", nts(f->hdr.loop_id));
		row_child_append("Remaining", nts(f->hdr.remaining));
#ifdef ENABLE_WMP_DEBUG

		row_child_append("Discarded", "");
		for (int i = 0; i< nnodes; i++){
			row_child_child_append("Messages",nts(f->hdr.qos_discarded[i]));
		}
		row_child_append("Msg this loop",nts(f->hdr.msg_this_loop));
#endif

		char qos_ac_interval;
		char qos_ac_pri;
		unsigned short qos_ac_lot;
		row_child_append("Access Control", "");
		row_child_child_append("Interval", nts(int(f->hdr.qos_ac_interval)));
		row_child_child_append("Priority", nts(int(f->hdr.qos_ac_pri)));
		row_child_child_append("Acc. LOT", nts(int(f->hdr.qos_ac_lot)));
		for (int i = 0; i < QOS_MAX_SRC; i++) {
			row_child_append("Flow " + nts(i), "");
			row_child_child_append("Source", nts(f->hdr.qos_src[i]));
			row_child_child_append("Destination", nts(f->hdr.qos_dst[i]));
			row_child_child_append("Deadline", nts(f->hdr.deadline[i]));
			row_child_child_append("Priority", nts(f->hdr.qos_priority[i]));
		}
		if (f->hdr.type == MESSAGE) {
			row_child_append("Data", "");
			QoS_Packet_Header * qph = (QoS_Packet_Header *) wmp_get_message_data_pointer(f);
			row_child_child_append("QoS Size", nts((int) qph->size));
			row_child_child_append("QoS Serial ", nts( qph->serial));
			row_child_child_append("Orig Src ", nts((int) qph->orig_src));
			row_child_child_append("Samples ", nts((int) qph->samples));
			row_child_child_append("Flow ", nts((int) qph->flow));
			row_child_child_append("Checksum ", nts((int) qph->cs));
			row_child_child_append("El. queue ", nts((int) qph->eq));
#ifdef ENABLE_WMP_DEBUG
			row_child_append("Stop Count",nts(qph->stop_count));
			for (int i = 0; i< qph->stop_count ; i++){
				row_child_child_append("Node Id",nts(qph->stop_node_id[i]));
				row_child_child_append("Delay",nts(qph->stop_delay[i]));
			}
#endif

		}
	}
#endif
}

void main_window::on_button6_clicked() {
	PF(fprintf(stderr,"**PF**  main_window::on_button6_clicked \n");)
	if (playing)
		return;
	hscale_position->set_value(hscale_position->get_value() + 1);
}

void main_window::on_button7_clicked() {
	PF(fprintf(stderr,"**PF**  main_window::on_button7_clicked \n");)
	if (playing)
		return;
	hscale_position->set_value(hscale_position->get_value() - 1);
}

void main_window::on_togglebutton1_clicked() {
	PF(fprintf(stderr,"**PF**  main_window::on_togglebutton1_clicked \n");)
	bool act = togglebutton1->get_active();
	if (act) {
		begin_play();
	} else {
		stop_playing();
	}
	button8->set_sensitive(!act);
	button9->set_sensitive(!act);
	button6->set_sensitive(!act);
	button7->set_sensitive(!act);
	button13->set_sensitive(!act);
}
/* Record */

void main_window::on_rec_btn_clicked(){
}


void main_window::on_togglebutton2_clicked() {
	return;
	PF(fprintf(stderr,"**PF**  main_window::on_togglebutton2_clicked \n");)

	/*button8->set_sensitive(!act);
	button6->set_sensitive(!act);
	button7->set_sensitive(!act);
	button13->set_sensitive(!act);
	button9->set_sensitive(!act);
*/

}

void main_window::save_cfg_file(){
	/* if on_exit focus is on the tw, segfault */
	int val=prim_cb->get_active();
	kf->set_integer("Gui", "Show_Prim",val);

	val = foreign_cb->get_active();
	kf->set_integer("Gui", "Show_Foreign",val);

	val = d2_cb->get_active();
	kf->set_integer("Gui", "Show_2D",val);

	val = text_cb->get_active();
	kf->set_integer("Gui", "Show_Text",val);

	val = d1_cb->get_active();
	kf->set_integer("Gui", "Show_Frames",val);


	this->set_focus(*spinbutton1);
	std::ofstream cf;
	cf.open(cfg_file.c_str(), ofstream::out);
	cf << kf->to_data();
	cf.close();
}

bool main_window::on_exit(){
	save_cfg_file();
	//fprintf(stderr,"aqui?");
	return true;
}

void main_window::on_button8_clicked() {
	//fprintf(stderr,"o aqui?");
	ostringstream oss;
	oss << getenv("HOME") << "/.wmpSniffer/active";
	unlink((const char*) oss.str().c_str());
	io_close_sim_data();
	Gtk::Main::quit();
}

void main_window::on_button9_clicked() {
	PF(fprintf(stderr,"**PF**  main_window::on_button9_clicked \n");)
	Gtk::MessageDialog
			dialog(*this, "RT-WMP Statistics for the last execution");

	if (w2 == NULL) {
		w2 = new window2(this);
	}
	w2->show();
	if (!recording) {
		w2->show_statistics(this->filename);
	} else {
		w2->show_statistics_from_values();
	}

}

bool main_window::on_main_window_scroll_event(GdkEventScroll * e) {
	PF(fprintf(stderr,"**PF**  main_window::on_main_window_scroll_event \n");)
	if (e->direction == GDK_SCROLL_UP) {
		on_button7_clicked();
	} else if (e->direction == GDK_SCROLL_DOWN) {
		on_button6_clicked();
	}
	return true;
}
void main_window::on_button13_clicked() {
	if (w1 == NULL) {
		w1 = new window1(this);
	}
	w1->show();
}

void main_window::on_nuevo1_activate() {
	PF(fprintf(stderr,"**PF**  main_window::on_nuevo1_activate \n");)
	Gtk::FileChooserDialog dialog("Please choose a Filename",
			Gtk::FILE_CHOOSER_ACTION_SAVE);
	dialog.set_transient_for(*this);
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
	dialog.set_current_name("hola");
	int result = dialog.run();
	if (result == Gtk::RESPONSE_CANCEL)
		return;


	char new_filename[256];
	strcpy(new_filename, dialog.get_filename().c_str());
	if (strstr(new_filename, ".vis") == NULL)
		strcat(new_filename, ".vis");
	strcpy(filename, new_filename);

	io_change_file();
	reset();
}

void main_window::on_abrir1_activate() {
	PF(fprintf(stderr,"**PF**  main_window::on_abrir1_activate \n");)

	Gtk::FileChooserDialog dialog("Please choose a File",
			Gtk::FILE_CHOOSER_ACTION_OPEN);
	dialog.set_transient_for(*this);
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
	Gtk::FileFilter filter_vis;
	filter_vis.set_name("wmpSniffer files (*.vis)");
	filter_vis.add_pattern("*.vis");
	dialog.add_filter(filter_vis);
	int result = dialog.run();
	if (result == Gtk::RESPONSE_CANCEL){
		return;
	}
	io_change_file();
	sprintf(filename, "%s", dialog.get_filename().c_str());
	reset();
}
void main_window::on_guardar1_activate() {
PF(fprintf(stderr,"**PF**  main_window::on_guardar1_activate \n");)
}

void main_window::on_guardar_como1_activate() {
	PF(fprintf(stderr,"**PF**  main_window::on_guardar_como1_activate \n");)

	Gtk::FileChooserDialog dialog("Please choose a Filename",
			Gtk::FILE_CHOOSER_ACTION_SAVE);
	dialog.set_transient_for(*this);
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
	Gtk::FileFilter filter_vis;
	filter_vis.set_name("wmpSniffer files (*.vis)");
	filter_vis.add_pattern("*.vis");
	dialog.add_filter(filter_vis);
	int result = dialog.run();
	if (result == Gtk::RESPONSE_CANCEL)
		return;

	io_change_file();

	char cmd[256];
	char new_filename[256];
	strcpy(new_filename, dialog.get_filename().c_str());
	if (strstr(new_filename, ".vis") == NULL)
		strcat(new_filename, ".vis");
	sprintf(cmd, "cp %s %s", filename, new_filename);
	if (system(cmd) == -1) {
		fprintf(stderr, "*** WARNING: Unable to execute 'cp %s %s'\n",
				filename, new_filename);
	}
	strcpy(filename, new_filename);
	reset();
}


void main_window::on_salir1_activate() {
	PF(fprintf(stderr,"**PF**  main_window::on_salir1_activate \n");)
	//io_close_sim_data();
	_exit(0);
}
void main_window::on_acerca_de1_activate() {
	PF(fprintf(stderr,"**PF**  main_window::on_acerca_de1_activate \n");)
	Gtk::MessageDialog dialog(*this, "wmpSniffer v0.2b");
	dialog.set_secondary_text(
			"Written by Danilo Tardioli\nUniversity of Zaragoza\nemail:dantard@unizar.es\nThanks to: Domenico Sicignano\n and Samuel Cabrero");
	dialog.run();
}

void main_window::on_cortar1_activate() {
PF(fprintf(stderr,"**PF**  main_window::on_cortar1_activate \n");)
}
void main_window::on_copiar1_activate() {
PF(fprintf(stderr,"**PF**  main_window::on_copiar1_activate \n");)
}
void main_window::on_pegar1_activate() {
PF(fprintf(stderr,"**PF**  main_window::on_pegar1_activate \n");)
}
void main_window::on_borrar1_activate() {
	PF(fprintf(stderr,"**PF**  main_window::on_borrar1_activate \n");)
			rec_tb->set_active(true);
}

void main_window::on_button23_clicked(){
	double val = spinbutton1->get_value();
	int y = io_get_pose_from_serial(val);
	//hscale_position->set_value(val);
	y = y > 50 ? y - 50 : 0;
	hscale_position->set_value(y);
}
void main_window::on_spinbutton1_activate(){
	on_button23_clicked();
}

/* C wrapper */

void new_frame(char * data, int data_size) {
	PF(fprintf(stderr,"**PF**  new_frame \n");)
	gdk_threads_enter();
	mw->continue_record(data,data_size);
	gdk_threads_leave();
}

void main_window::row_child_append(string txt1, string txt2) {
	Gtk::TreeModel::iterator iter_child =
			m_refTreeModel->append(row.children());
	subrow = *iter_child;
	//(*iter_child)[m_Columns.m_col_id]=id;
	//(*iter_child)[m_Columns.m_col_type]=type;
	(*iter_child)[m_Columns.m_col_name] = txt1;
	(*iter_child)[m_Columns.m_col_value] = txt2;

}
void main_window::row_child_child_append(string txt1, string txt2) {
	Gtk::TreeModel::iterator iter_child = m_refTreeModel->append(
			subrow.children());
	(*iter_child)[m_Columns.m_col_name] = txt1;
	(*iter_child)[m_Columns.m_col_value] = txt2;

}

void main_window::row_child_child_append(string txt1, int id, int val,
		string uni) {
	ostringstream s1, s2;
	s1 << txt1 << " " << id;
	s2 << val << " " << uni;
	Gtk::TreeModel::iterator iter_child = m_refTreeModel->append(
			subrow.children());
	subrow = *iter_child;
	(*iter_child)[m_Columns.m_col_name] = s1.str();
	(*iter_child)[m_Columns.m_col_value] = s2.str();
}

void main_window::row_append(string txt1, string txt2) {
	row = *(m_refTreeModel->append());
	row[m_Columns.m_col_name] = txt1;
	row[m_Columns.m_col_value] = txt2;
}


bool  main_window::on_xfield_tw_focus_in_event(GdkEventFocus *ev){
	tw_has_focus=true;
	Glib::RefPtr<Gtk::TreeSelection> refTreeSelection =
		xfield_tw->get_selection();
	Gtk::TreeModel::iterator iter = refTreeSelection->get_selected();

	if (iter) {
		Gtk::TreeModel::Row row = *iter;
	}
	return true;
}


bool  main_window::on_xfield_tw_focus_out_event(GdkEventFocus *ev){
	tw_has_focus=false;
	return true;
}

void main_window::on_xfield_tw_row_collapsed(const Gtk::TreeModel::iterator& iter, const Gtk::TreeModel::Path& path){

}
void main_window::on_xfield_tw_row_expanded(const Gtk::TreeModel::iterator& iter, const Gtk::TreeModel::Path& path){

}

void main_window::on_import_activate(){
}

void main_window::on_create_log_activate(){
	io_go_to(0);
	char data[2500];
	simData_Hdr * p = (simData_Hdr *) data;
	wmpFrame * r = (wmpFrame *) (data + sizeof(simData_Hdr));
	logger_init(1,nnodes);
	while (io_read_next_sim_data(data)>0){
		if (r->hdr.type == (char) MESSAGE && (int) r->hdr.to == r->msg.dest){
			logger_printData(r->msg.dest,r,p->time);
		}
		usleep(1);
	}
}

/* C WRAPPER */

void setRobotPose(int id, double x0, double y0, double ang, int col) {
	mw->areadx->setRobotPose(id, x0, y0, ang, col);
}
void addLink(int r1, int r2, int arrow, int col, float value, int set) {
	mw->areadx->addLink(r1, r2, arrow, col, value, set);
}
void addLink(int r1, int r2, int arrow, int col, int set) {
	mw->areadx->addLink(r1, r2, arrow, col, set);
}

void clearLinks(int set) {
	mw->areadx->clearLinks(set);
}

void getRobotPose(int id, double* x0, double* y0, double* ang) {
	mw->areadx->getRobotPose(id, x0, y0, ang);
}
int getSelected() {
	return mw->areadx->get_selected();
}
int addRobot() {
	return mw->areadx->addRobot();
}
void setRobotColor(int idx, int c) {
	mw->areadx->setRobotColor(idx, c);
}

void addPoint(int x, int y, int color, int with_respect_to_robot) {
	mw->areadx->addPoint(x, y, color, with_respect_to_robot);
}

void getSimParam(double & threshold, double & x_size, double & y_size) {
	threshold = mw->sim_thresh;
	x_size = mw->sim_xsize;
	y_size = mw->sim_ysize;
}

void lowerLayersRecStop(){
	//mw->reset();
}

bool showForeign(){
	return mw->foreign_cb->get_active();
}

void informBridgeOff(){
	mw->recording_stopped();
}


void  main_window::on_rec_tb_toggled(){}

void  main_window::on_rec_tb_released(){
	if (recording){
			rec_tb->set_sensitive(false);
			request_stop_recording();
	}else{
		recording = begin_online_record();
		button8->set_sensitive(!recording);
		button6->set_sensitive(!recording);
		button7->set_sensitive(!recording);
		button13->set_sensitive(!recording);
		button9->set_sensitive(!recording);
		togglebutton1->set_sensitive(!recording);
		rec_tb->set_active(recording);
	}
}

void  main_window::on_rec_tb_activate(){}

bool main_window::timer_callback(){
	return true;
}
