/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: window1.hh
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

#ifndef _WINDOW1_HH
#  include "window1_glade.hh"
#  define _WINDOW1_HH

#include <gtkmm/window.h>
#include <gtkmm/treeview.h>
#include <gtkmm/scrolledwindow.h>
#include <gtkmm/radiobutton.h>
#include <gtkmm/box.h>
#include <gtkmm/alignment.h>
#include <gtkmm/entry.h>
#include <gtkmm/liststore.h>
#include <gtkmm/treestore.h>

class main_window;

class window1 : public window1_glade
{  
    void on_button10_clicked();
    void on_button11_clicked();
    void on_button12_clicked();
    void on_cursor_changed();
    int selected;
    main_window* mw;
    class ModelColumns : public Gtk::TreeModel::ColumnRecord {    	
public:
		ModelColumns() {
			add(m_col_id);
			add(m_col_number);
			add(m_col_name);
			add(m_col_bc);
		}
		Gtk::TreeModelColumn<unsigned int> m_col_id;
		Gtk::TreeModelColumn<Glib::ustring> m_col_name;
		Gtk::TreeModelColumn<Glib::ustring> m_col_bc;
		Gtk::TreeModelColumn<unsigned int> m_col_number;
		
	};
	ModelColumns m_Columns;
	Glib::RefPtr<Gtk::TreeStore> m_refTreeModel;
public:
	int get_selected();
	window1(main_window *);
};
#endif
