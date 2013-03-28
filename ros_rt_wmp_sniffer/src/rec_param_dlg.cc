/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: rec_param_dlg.cc
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
#include "rec_param_dlg.hh"
#include "unistd.h"
#include <gtkmm/messagedialog.h>
#include <gtkmm/filechooserdialog.h>
#include <gtkmm/stock.h>
void rec_param_dlg::on_sim_rb_clicked(){
    	ok_btn->set_sensitive(true);
    	if (marte_rb->get_active()){
    		int id=getuid();
    		if (id!=0){
    			ok_btn->set_sensitive(false);
    		 	Gtk::MessageDialog dialog(*this, "wmpSniffer v0.1b");
    			 	dialog.set_secondary_text("Root Privileges are needed to use MaRTE Monitor");
    			dialog.run();
    		}
    	}
}

void rec_param_dlg::on_button24_clicked(){
	Gtk::FileChooserDialog dialog("Please choose a folder",
			Gtk::FILE_CHOOSER_ACTION_SELECT_FOLDER);
	dialog.set_transient_for(*this);

	//Add response buttons the the dialog:
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);

	int result = dialog.run();
	switch (result) {
		case (Gtk::RESPONSE_OK): {
			break;
		}
		case (Gtk::RESPONSE_CANCEL): {

			return;
			break;
		}
		default: {
			//std::cout << "Unexpected button clicked." << std::endl;
			break;
		}
	}
}

void rec_param_dlg::on_shmem_btn_clicked(){
	Gtk::FileChooserDialog dialog("Please choose an executable",
				Gtk::FILE_CHOOSER_ACTION_OPEN);
		dialog.set_transient_for(*this);

		//Add response buttons the the dialog:
		dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
		dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);

		int result = dialog.run();
		switch (result) {
			case (Gtk::RESPONSE_OK): {
				break;
			}
			case (Gtk::RESPONSE_CANCEL): {
				return;
				break;
			}
			default: {
				//std::cout << "Unexpected button clicked." << std::endl;
				break;
			}
		}
		shmem_file->set_text(dialog.get_filename());
}

#include <vte-0.0/vte/vte.h>

rec_param_dlg::rec_param_dlg() : rec_param_dlg_glade(){

}
void rec_param_dlg::on_filename_clicked(){
	Gtk::FileChooserDialog dialog("Please choose a Filename",
			Gtk::FILE_CHOOSER_ACTION_SAVE);
	dialog.set_transient_for(*this);
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
	dialog.run();
	filename_txt->set_text(dialog.get_filename());
}
