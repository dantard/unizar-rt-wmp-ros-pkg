/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: main_window_glade.hh
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

#ifndef _MAIN_WINDOW_HH
#  include "main_window_glade.hh"
#  include "core/include/frames.h"
#  include "bridge.hh"
#  define _MAIN_WINDOW_HH
#  include "window1.hh"
#  include "window2.hh"
#  include "window3.hh"
#  include "area.h"
#include "basic_io.h"
#include <map>
#include <glibmm/keyfile.h>
class main_window : public main_window_glade
{  public:
	main_window(char * s_iface);
	~main_window();
	void setRobotPose(int idx,double x, double y, double a, int col);
    virtual void on_button6_clicked();
    virtual void on_button7_clicked();
    virtual void on_togglebutton2_clicked();
    virtual void on_button8_clicked();
    virtual void on_button9_clicked();
    virtual void on_togglebutton1_clicked();
    virtual void on_hscale_position_value_changed();
    virtual void on_hscale_zoom_value_changed();
    virtual void on_nuevo1_activate();
    virtual void on_abrir1_activate();
    virtual void on_guardar1_activate();
    virtual void on_guardar_como1_activate();
    virtual void on_salir1_activate();
    virtual void on_cortar1_activate();
    virtual void on_copiar1_activate();
    virtual void on_pegar1_activate();
    virtual void on_borrar1_activate();
    virtual void on_acerca_de1_activate();
    virtual void on_button23_clicked();
    virtual bool on_exit();
    virtual void on_xfield_tw_cursor_changed(){}
    virtual bool on_xfield_tw_focus_in_event(GdkEventFocus *ev);
    virtual bool on_xfield_tw_focus_out_event(GdkEventFocus *ev);
    virtual void on_xfield_tw_row_collapsed(const Gtk::TreeModel::iterator& iter, const Gtk::TreeModel::Path& path);
    virtual void on_xfield_tw_row_expanded(const Gtk::TreeModel::iterator& iter, const Gtk::TreeModel::Path& path);
    virtual bool on_xfield_tw_button_press_event(GdkEventButton *ev);
    virtual void on_import_activate();
    virtual void on_create_log_activate();
    void begin_play();
    void continue_play();
    void stop_playing();
    bool begin_online_record();
    bool begin_record();
    void continue_record(char * data, int data_size);
    void request_stop_recording();
    void recording_stopped();
    virtual bool on_main_window_button_press_event(GdkEventButton *ev);
    void on_w_resize(GdkEventConfigure* event);
    virtual bool on_main_window_motion_notify_event(GdkEventMotion *ev);

    virtual bool on_main_window_expose_event(GdkEventExpose *ev);
    virtual void on_spinbutton1_activate();
    void fill_screen();
    void reset(int val = -1);
    /* update view > */
    void draw_stuffs(char * data, int pos_in_file,int show_frames, int show_nodes,int show_text);
    void update_frame_labels(wmpFrame * f);
    void update_pose_labels();
    void update_frame_info(simData_Hdr *p, wmpFrame * q);
    void show_broadcast_data(wmpFrame *f);
    void show_routing_data(wmpFrame *f);
    void show_qos_data(wmpFrame * f);
    /* update view < */
    void statistics(char *);
    bool on_main_window_scroll_event(GdkEventScroll * e);
    void on_button13_clicked();
    void notify_selection(int);
    void save_cfg_file();
    class area *areadx, *areasx;
    class MyArea * MyArea;
    double sim_xsize,sim_ysize,sim_thresh;
    Glib::KeyFile *kf;
    wmpFrame selectedFrame;

private:
	int timer;
	int yap;
	int rec_type;
	/* extra-fields > */
	class ModelColumns: public Gtk::TreeModel::ColumnRecord {
	public:
		ModelColumns() {
			add(m_col_id);
			add(m_col_name);
			add(m_col_value);
			add(m_col_type);
		}
		Gtk::TreeModelColumn<unsigned int> m_col_type;
		Gtk::TreeModelColumn<unsigned int> m_col_id;
		Gtk::TreeModelColumn<Glib::ustring> m_col_name;
		Gtk::TreeModelColumn<Glib::ustring> m_col_value;
	};
	ModelColumns m_Columns;
	std::map<std::string,bool> rows_expanded;
	Glib::RefPtr<Gtk::TreeStore> m_refTreeModel;
	Gtk::TreeModel::Row row;
	Gtk::TreeModel::Row subrow;
	Gtk::Menu m_Menu_Popup;
	void row_child_child_append(std::string txt1, std::string txt2);
	void row_append(std::string txt1, std::string txt2);
	void row_child_append(std::string txt1, std::string txt2);
	void row_child_child_append(std::string txt1, int id, int val, std::string uni);
	void saveRowsStatus();
	void restoreRowsStatus();
	void show_path();
	/* extra-fields < */

	char filename[256];
	char data_backup[2500];
	int play_idx, play_idx2, playing, recording;
	int nnodes,k,idx3;
	int checkstring(char * txt);
	long long frame_time;
	void fill_tree(wmpFrame * r, long long ptime, int pos_in_file=-1);
	bool timer_callback();
    class rec_param_dlg * rp_dlg;
    window1* w1;
    window2* w2;
    window3* w3;
    std::string cfg_file, executable;
    bool tw_has_focus;
    bool timered_show_t;
    void updater();
    virtual void on_rec_btn_clicked();
    virtual void on_rec_tb_toggled();
    virtual void on_rec_tb_released();
    virtual void on_rec_tb_activate();
    bool MyCallback(int n);
};

/* C wrapper */
void new_frame(char * data, int data_size);
void getSimParam(double & threshold, double & x_size, double & y_size);
void lowerLayersRecStop();
bool showForeign();
void informBridgeOff();
#endif
