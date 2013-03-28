/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: window1.cc
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
#include "window1.hh"
#include "bridge.hh"
#include "main_window.hh"

    void window1::on_button10_clicked(){
    	entry1->set_text("");
    	set_title("Search");
    	m_refTreeModel->clear();
    }
    int window1::get_selected(){
    	return selected;
    }
    void window1::on_button11_clicked(){
    	m_refTreeModel->clear();
    	char msg[1000];
    	char bc_msg[1000];

    	//wmpFrame * p= (wmpFrame*) tmp;
    	io_go_to(0);
    	int res=0,serial=0,idx=0,pos=0;
    	while (res>=0){
       		if (token_rb->get_active()){
       			res=read_next_token(msg,bc_msg,&serial,&pos,entry1->get_text().c_str());
       			if (res>0){
       				Gtk::TreeModel::Row row = *(m_refTreeModel->append());
       				row[m_Columns.m_col_id] = pos;
       				row[m_Columns.m_col_bc] = bc_msg;
       				row[m_Columns.m_col_number] = serial;
       				idx++;
       			}
       		} else if (message_rb->get_active()){
   				res=read_next_message(msg,bc_msg,&serial,&pos,entry1->get_text().c_str());
   			    if (res>0){
   			    	Gtk::TreeModel::Row row = *(m_refTreeModel->append());
   			    	row[m_Columns.m_col_id] = pos;
   			    	row[m_Columns.m_col_name] = msg;
   			    	row[m_Columns.m_col_bc] = bc_msg;
   			    	row[m_Columns.m_col_number] = serial;
   			    	idx++;
   			    	selected=pos;
   				}
       		} else if (aut_rb->get_active()){
       			res=read_next_authorization(msg,bc_msg,&serial,&pos,entry1->get_text().c_str());
       			if (res>0){
       				Gtk::TreeModel::Row row = *(m_refTreeModel->append());
       				row[m_Columns.m_col_id] = pos;
       				row[m_Columns.m_col_bc] = bc_msg;
       				row[m_Columns.m_col_number] = serial;
       				idx++;
       			}
       		} else if (drop_rb->get_active()){
       			res=read_next_drop(msg,bc_msg,&serial,&pos,entry1->get_text().c_str());
       			if (res>0){
       				Gtk::TreeModel::Row row = *(m_refTreeModel->append());
       				row[m_Columns.m_col_id] = pos;
       				row[m_Columns.m_col_number] = serial;
       				idx++;
       			}
   			} else if (all_rb->get_active()){
       			res=read_next_bc(msg,bc_msg,&serial,&pos,entry1->get_text().c_str());
       			if (res>0){
       				Gtk::TreeModel::Row row = *(m_refTreeModel->append());
       				row[m_Columns.m_col_id] = pos;
   			    	row[m_Columns.m_col_name] = msg;
       				row[m_Columns.m_col_bc] = bc_msg;
       				row[m_Columns.m_col_number] = serial;
       				idx++;
       			}
   			}
    	}
    	char txt[30];
    	sprintf(txt,"Search (%d)",idx);
    	set_title(txt);
    }
    void window1::on_button12_clicked(){
    	hide();
    }

window1::window1(main_window* mw):window1_glade(){
	this->mw=mw;
	set_size_request(200, 600);
	treeview1->signal_cursor_changed().connect(sigc::mem_fun(*this, &window1::on_cursor_changed));
	m_refTreeModel = Gtk::TreeStore::create(m_Columns);
	treeview1->set_model(m_refTreeModel);
  	treeview1->append_column("ID", m_Columns.m_col_id); //This number will be shown with the default numeric formatting.
  	treeview1->append_column("Serial", m_Columns.m_col_number);
  	treeview1->append_column("Message                       ", m_Columns.m_col_name);
  	treeview1->append_column("BC Message", m_Columns.m_col_bc);
	set_title("Search");
	set_keep_above(true);
}

void window1::on_cursor_changed(){
	Glib::RefPtr<Gtk::TreeSelection> refTreeSelection = treeview1->get_selection();
    Gtk::TreeModel::iterator iter = refTreeSelection->get_selected();
    if(iter){
        Gtk::TreeModel::Row row = *iter;
        selected = row[m_Columns.m_col_id];
        mw->notify_selection(selected);
    }
}
