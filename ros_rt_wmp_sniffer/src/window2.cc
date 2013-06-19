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

#include "config.h"
#include "window2.hh"
#include "window2.hh"
#include "wmp_config.h"

#include "core/include/frames.h"
#include "core/interface/Msg.h"
#include "math.h"
#include "wmp_specific.hh"
#include "bridge.hh"
#include "main_window.hh"
#include <sstream>
#include <gtkmm/dialog.h>
#include <gtkmm/stock.h>
#include <gtkmm/filechooserdialog.h>
#include "statistics.h"

void window2::on_button10_clicked() {
	Glib::RefPtr<Gtk::Clipboard> refClipboard = Gtk::Clipboard::get();
	refClipboard->set_text(oss.str().c_str());
	std::ostringstream fn;
	Gtk::FileChooserDialog dialog("Please choose a Filename",
			Gtk::FILE_CHOOSER_ACTION_SAVE);
	dialog.set_transient_for(*this);
	dialog.add_button(Gtk::Stock::CANCEL, Gtk::RESPONSE_CANCEL);
	dialog.add_button(Gtk::Stock::OK, Gtk::RESPONSE_OK);
	Gtk::FileFilter filter_vis;
	filter_vis.set_name("log files (*.wlg)");
	filter_vis.add_pattern("*.wlg");
	dialog.add_filter(filter_vis);
	int result = dialog.run();
	if (result == Gtk::RESPONSE_CANCEL)
		return;
	FILE * f = fopen(dialog.get_filename().c_str(), "w+");
	//fprintf(f,"%s", oss.str().c_str());
	fclose(f);
}

void window2::on_button11_clicked() {
	show_statistics_from_values();
}

void window2::on_button12_clicked() {
	range_setted = 0;
	hide();
}
#include "icon.h"
window2::window2(main_window* mw) :
	window2_glade() {
	set_icon(Gdk::Pixbuf::create_from_xpm_data(icon));
	this->mw = mw;
	button10->hide();
	set_size_request(600, 600);
	treeview2->signal_cursor_changed().connect(sigc::mem_fun(*this,
			&window2::on_cursor_changed));
	m_refTreeModel = Gtk::TreeStore::create(m_Columns);
	treeview2->set_model(m_refTreeModel);
	treeview2->append_column("Message                       ",
			m_Columns.m_col_name);
	treeview2->append_column("Value", m_Columns.m_col_value);
	set_keep_above(true);
	range_setted = 0;

	Gtk::Menu::MenuList& menulist = m_Menu_Popup.items();
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Write to file", sigc::mem_fun(
			*this, &window2::write_to_file)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Time Graph", sigc::mem_fun(
			*this, &window2::time_graph)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Distribution Graph (Auto)", sigc::mem_fun(
			*this, &window2::dist_graph)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Distribution Graph (1000 bin)", sigc::mem_fun(
			*this, &window2::dist_graph_1000)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Distribution Graph (100 bin)", sigc::mem_fun(
			*this, &window2::dist_graph_100)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Distribution Graph (10 bin)", sigc::mem_fun(
				*this, &window2::dist_graph_10)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Distribution Graph (50% Auto)", sigc::mem_fun(
				*this, &window2::dist_graph_a5)));
	menulist.push_back(Gtk::Menu_Helpers::MenuElem("_Distribution Graph (50% 100 bin)", sigc::mem_fun(
				*this, &window2::dist_graph_100_5)));


	m_Menu_Popup.accelerate(*this);

}
void window2::write_to_file(){
	stat_write_to_file(rowSelected);
}
void window2::time_graph(){
	stat_plot_tg(rowSelected);
}
void window2::dist_graph(){
	stat_plot_hist(rowSelected,0,1);
}
void window2::dist_graph_1000(){
	stat_plot_hist(rowSelected,1000,1);
}
void window2::dist_graph_100(){
	stat_plot_hist(rowSelected,100,1);
}
void window2::dist_graph_10(){
	stat_plot_hist(rowSelected,10,1);
}
void window2::dist_graph_a5(){
	stat_plot_hist(rowSelected,0,0.5);
}
void window2::dist_graph_100_5(){
	stat_plot_hist(rowSelected,100,0.5);
}

void window2::row_append(int id, const char* txt, int value,const char* uni){
	long long llvalue=value;
	row_append(id,txt, llvalue, uni);
}

void window2::row_append(int id, const char* txt, long long value, const char*uni) {
	char v[64];
	sprintf(v, "%lld %s", value, uni);
	row = *(m_refTreeModel->append());
	row[m_Columns.m_col_id] = id;
	row[m_Columns.m_col_name] = txt;
	row[m_Columns.m_col_value] = v;
	oss << "\n" << txt << " " << v << "\n";
}

void window2::row_append(int id, const char* txt, const char*uni) {
	char v[64];
	sprintf(v, "%s", uni);
	row = *(m_refTreeModel->append());
	row[m_Columns.m_col_id] = id;
	row[m_Columns.m_col_name] = txt;
	row[m_Columns.m_col_value] = v;
	oss << "\n" << txt << " " << v << "\n";
}

/* button efz*/
void window2::on_button22_clicked() {
	int nnodes = 1;
	fprintf(stderr,"bgn:%d end:%d\n", 22, 22);

	//io_open_sim_data(this->filename);
	if (nnodes > 0) {
		int n = (int) to_sb->get_value();
		int from = (int) from_sb->get_value();


		io_go_to(from);
		io_create_tmp_file();
		char data[2500];
		for (int i = from; i < n; i++) {
			int bytes = io_read_next_sim_data(&data[0]);
			if (bytes <= 0 || !io_write_on_tmp_file(data,bytes)){
				fprintf(stderr, "File End...\n");
				break;
			}
		}
		io_close_tmp_file();
	}
	char cmd[256];
	sprintf(cmd, "cp tmp.vis %s", this->filename);

	if (system(cmd) == -1){
		fprintf(stderr, "*** WARNING: Unable to execute 'cp tmp.vis %s'\n", this->filename);
	}
	mw->reset();
	range_setted = 0;
	this->show_statistics(0);
}

void window2::row_child_append(int id, const char* txt, int value,const char* uni){
	long long llvalue=value;
	row_child_append(id, txt, llvalue, uni);
}
void window2::row_child_append(int type, int id, const char* txt, int value, const char*uni){
	long long llvalue=value;
	row_child_append(type,id,txt,llvalue,uni);
}

void window2::row_child_append(int id, const char* txt, long long value, const char*uni) {
	row_child_append(NONE, id, txt, value, uni);
}

void window2::row_child_child_append(int id, const char* txt, long long value, const char*uni) {
	char v[64];
	sprintf(v, "%lld %s", value, uni);
	Gtk::TreeModel::iterator iter_child = m_refTreeModel->append(
			subrow.children());
	(*iter_child)[m_Columns.m_col_id] = id;
	(*iter_child)[m_Columns.m_col_type] = NONE;
	(*iter_child)[m_Columns.m_col_name] = txt;
	(*iter_child)[m_Columns.m_col_value] = v;
}

void window2::row_child_append(int type, int id, const char* txt, long long value, const char*uni) {
	char v[64];
	if (uni != NULL) {
		sprintf(v, "%lld %s", value, uni);
	}else{
		sprintf(v, "%lld", value);
	}
	Gtk::TreeModel::iterator iter_child =
			m_refTreeModel->append(row.children());
	subrow = (*iter_child);
	(*iter_child)[m_Columns.m_col_id] = id;
	(*iter_child)[m_Columns.m_col_type] = type;
	(*iter_child)[m_Columns.m_col_name] = txt;
	(*iter_child)[m_Columns.m_col_value] = v;
	if (uni != NULL) {
		oss << txt << " " << v << "\n";
	}
}

void window2::row_append(int id, const char* txt, float value, const char*uni) {
	char v[64];
	sprintf(v, "%5.3f %s", value, uni);
	row = *(m_refTreeModel->append());
	row[m_Columns.m_col_id] = id;
	row[m_Columns.m_col_name] = txt;
	row[m_Columns.m_col_value] = v;
	oss << "\n" << txt << " " << v << "\n---------------\n";
}

////////////NEW


void window2::row_child_append(int id, const char* txt, float value, const char*uni) {
	char v[64];
	sprintf(v, "%5.3f %s", value, uni);
	Gtk::TreeModel::iterator iter_child =
			m_refTreeModel->append(row.children());
	subrow = *iter_child;
	(*iter_child)[m_Columns.m_col_id] = id;
	(*iter_child)[m_Columns.m_col_name] = txt;
	(*iter_child)[m_Columns.m_col_value] = v;
	oss << txt << " " << v << "\n";
}

void window2::show_statistics(char * filename) {
	statistics_init();
	m_refTreeModel->clear();
	int bgn = -1, end = -1;
	if (filename != NULL){
		strcpy(this->filename, filename);
	}
	statistics_from_file(this->filename, bgn, end);
	statistics_publish(this);
	from_sb->set_value(bgn);
	to_sb->set_value(end);
}

void window2::show_statistics_from_values() {
	statistics_init();
	m_refTreeModel->clear();
	int bgn = from_sb->get_value();
	int end = to_sb->get_value();
	statistics_from_file(this->filename, bgn, end);
	statistics_publish(this);
}
char *  window2::getFileName(){
	return filename;
}
void window2::on_cursor_changed() {

	Glib::RefPtr<Gtk::TreeSelection> refTreeSelection =
			treeview2->get_selection();
	Gtk::TreeModel::iterator iter = refTreeSelection->get_selected();

	if (iter) {
		Gtk::TreeModel::Row row = *iter;
		selected = row[m_Columns.m_col_id];
//		fprintf(stderr, "Selected:%d\n", selected);
		if (row[m_Columns.m_col_type] == MIN_V) {
			from_sb->set_value(selected);
		}
		if (row[m_Columns.m_col_type] == MAX_V) {
			to_sb->set_value(selected);
		}
		if (selected >= 0)
			mw->notify_selection(selected);

		std::string exp_name((Glib::ustring(row[m_Columns.m_col_name])).c_str());
		rowSelected=row[m_Columns.m_col_uid];
	}



	//Glib::RefPtr<Gtk::Clipboard> refClipboard = Gtk::Clipboard::get();
	//refClipboard->set_text("hola");

	/*FILE *  a = popen("gnuplot","w");
	fprintf(a,"plot 'dat.dat'\n");
	fflush(a);
*/
}




bool window2::on_treeview2_button_press_event(GdkEventButton *ev) {
	if ((ev->type == GDK_BUTTON_PRESS) && (ev->button == 3)) {
		if (stat_ask_plotter(rowSelected)){
			m_Menu_Popup.popup(ev->button, ev->time);
		}
	}

	return false;
}

void window2::on_button21_clicked() {
	int a,b;
	statistics_get_fez(a, b);
	from_sb->set_value(a);
	to_sb->set_value(b);
	show_statistics_from_values();
}

void window2::on_button20_clicked() {
	show_statistics();
}


/////////////////// NEW

void window2::root() {
	level = 0;
	xrow_name.clear();
}

void window2::subSectionBegin() {
	level ++;
}

void window2::subSectionEnd() {
	level --;
	xrow_name.erase(xrow_name.end());
}

void window2::row_create(std::string txt1, std::string txt2, int pos, int uid) {
	if (level == 0){
		xrow = row_level[0] = *(m_refTreeModel->append());
	}else{
		xrow = row_level[level] = *(m_refTreeModel->append(row_level[level-1].children()));
	}
	xrow[m_Columns.m_col_id] = pos;
	xrow[m_Columns.m_col_uid] = uid;
	xrow[m_Columns.m_col_name] = txt1;
	xrow[m_Columns.m_col_value] = txt2;
	xrow_name.resize(level+1);
	xrow_name.at(level) = txt1;
}

std::string window2::getCurrentRowName(){
	std::string s = "";
	for (int i= 0 ; i< level;i++){
		if (i!=0){
			s+= "::";
		}
		s+= xrow_name.at(i);
	}

	return s;
}

