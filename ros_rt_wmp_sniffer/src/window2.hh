/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: window2.cc
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

#ifndef _WINDOW2_HH
#  include "window2_glade.hh"
#  define _WINDOW2_HH
#include <gtkmm/treestore.h>
#include <vector>
#include <string>
#include <sstream>
#include "stats.h"
//using namespace std;


struct  QoSD{
	int flow_id;
	int msg_id;
};

class main_window;
class window2 : public window2_glade{

        void on_button10_clicked();
        void on_button11_clicked();
        void on_button12_clicked();
        void on_button20_clicked();
        void on_button21_clicked();
        void on_cursor_changed();
        void on_button22_clicked();
        bool on_treeview2_button_press_event(GdkEventButton *ev);
        void time_graph();
        void dist_graph_1000();
        void dist_graph_100();
        void dist_graph_10();
        void dist_graph_a5();
        void dist_graph_100_5();
        void dist_graph();
        void write_to_file();
        std::vector<std::string> xrow_name;
        Gtk::Menu m_Menu_Popup;
        int selected;
        int range_setted,min_c,max_c;
        int level, old_level;
        bool already_created;
        main_window* mw;
        FILE * sfile;
        char stats[100000];
        char filename[256];
        std::ostringstream oss;
        class ModelColumns : public Gtk::TreeModel::ColumnRecord {
        public:
        		ModelColumns() {
        			add(m_col_id);
        			add(m_col_uid);
        			add(m_col_name);
        			add(m_col_value);
        			add(m_col_type);
        		}
        		Gtk::TreeModelColumn<int> m_col_type;
        		Gtk::TreeModelColumn<int> m_col_id;
        		Gtk::TreeModelColumn<int> m_col_uid;
        		Gtk::TreeModelColumn<Glib::ustring> m_col_name;
        		Gtk::TreeModelColumn<Glib::ustring> m_col_value;

        };
		ModelColumns m_Columns;
		Gtk::TreeModel::Row row, xrow, actual;
		Gtk::TreeModel::Row subrow, raiz;
		Gtk::TreeModel::Row row_level[10];
		Gtk::TreeNodeChildren  * arow;
		int rowSelected;
        public:
    		Glib::RefPtr<Gtk::TreeStore> m_refTreeModel;
        	int get_selected();
        	window2(main_window *);
        	void show_statistics(char * filename = NULL);
        	void show_statistics_from_values();

        	void row_append(int id, const char* txt, long long value,const char* uni);
        	void row_append(int id, const char* txt, int value,const char* uni);
        	void row_append(int id, const char* txt, float value,const char* uni);


        	void row_child_append(int id, const char* txt, float value,const char* uni);
        	void row_child_append(int id, const char* txt, long long value,const char* uni);
        	void row_child_append(int type, int id, const char* txt, long long value, const char*uni);
        	void row_child_append(int id, const char* txt, int value,const char* uni);
        	void row_child_append(int type, int id, const char* txt, int value, const char*uni);

        	void row_append(int id, const char* txt, const char*uni);
        	void row_child_child_append(int id, const char* txt, long long value, const char*uni);

        	void root();
        	void subSectionBegin();
        	void subSectionEnd();
        	void row_create(std::string txt1, std::string txt2, int pos = -1, int uid = -1);
        	char *  getFileName();
        	std::string  getCurrentRowName();

};

#endif
