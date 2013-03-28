/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: window3.cc
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
/*
 * Standard gettext macros.
 */
#ifdef ENABLE_NLS
#  include <libintl.h>
#  undef _
#  define _(String) dgettext (GETTEXT_PACKAGE, String)
#  ifdef gettext_noop
#    define N_(String) gettext_noop (String)
#  else
#    define N_(String) (String)
#  endif
#else
#  define textdomain(String) (String)
#  define gettext(String) (String)
#  define dgettext(Domain,Message) (Message)
#  define dcgettext(Domain,Message,Type) (Message)
#  define bindtextdomain(Domain,Directory) (Domain)
#  define _(String) (String)
#  define N_(String) (String)
#endif
#include <gtkmmconfig.h>
#if GTKMM_MAJOR_VERSION==2 && GTKMM_MINOR_VERSION>2
#define GMM_GTKMM_22_24(a,b) b
#else //gtkmm 2.2
#define GMM_GTKMM_22_24(a,b) a
#endif //
#include "window3_glade.hh"
#include <gdk/gdkkeysyms.h>
#include <gtkmm/accelgroup.h>
#include <gtkmm/alignment.h>
#include <gtkmm/label.h>
#include <gtkmm/frame.h>
#if GTKMM_MAJOR_VERSION==2 && GTKMM_MINOR_VERSION>2
#include <gtkmm/expander.h>
#else //
#include <gtkmm/handlebox.h>
#endif //
#include "window3.hh"
#include <vte-0.0/vte/vte.h>
#include <ostream>
#include <sys/types.h>
#include <sys/wait.h>

void func(GtkWidget *widget, gpointer * data){
//	int exit_code = vte_terminal_get_child_exit_status((VteTerminal*)w3->vte);
}

void window3::die(){
	char txt[]={3,0};
	for (int i=0;i<nn;i++){
		vte_terminal_feed_child(VTE_TERMINAL(vtes[i]), txt, strlen(txt));
	}
	return;
//	g_signal_handler_disconnect(vte, handler);
//	gtk_widget_destroy(vte);
}

void window3::compile_and_execute(bool docompile, bool doexecute, bool dolog,int nn,std::string command,std::string args){
	this->nn = nn;
	this->command = command;
	this->doexecute = doexecute;
	this->dolog = dolog;
	this->args = args;
	int pos = command.find_last_of("/");
	s = command.substr(0, pos + 1);
	std::string cd("cd"), touch("touch");
	cd = cd + " " + s + "\n";
	touch = touch + " " + s;
	std::string alias = "";
	if (docompile){
		int ret = system(touch.c_str());
		if (ret !=0){
		        fprintf(stderr,"Error executing %s\n",touch.c_str());
		}
		vte_terminal_feed_child(VTE_TERMINAL(vte), cd.c_str(), strlen(cd.c_str()));
		std::string alias = "alias make=\"touch lock && make && sleep 2 && rm lock\"\n";
		vte_terminal_feed_child(VTE_TERMINAL(vte), alias.c_str(), strlen(alias.c_str()));
		vte_terminal_feed_child(VTE_TERMINAL(vte), "make\n", strlen("make\n"));
	}
	if (doexecute){
		alias = "alias wce=\"while test -f lock ; do echo waiting compilation; sleep 1 ; done\"\n";
		for (int i=0;i<used;i++){
			std::string cmd = command;
			std::ostringstream s1,s2;
			s1 << i;
			s2 << nn;
			std::string rep = args;
			int pos = rep.find_first_of("%i",0);
			rep.replace(pos,2,s1.str());
			pos = rep.find_first_of("%n",0);
			rep.replace(pos,2,s2.str());
			cmd = "wce && " + cmd + " " + rep + "\n";
			vte_terminal_feed_child(VTE_TERMINAL(vtes[i]), alias.c_str(), strlen(alias.c_str()));
			vte_terminal_feed_child(VTE_TERMINAL(vtes[i]), cd.c_str(), strlen(cd.c_str()));
			vte_terminal_feed_child(VTE_TERMINAL(vtes[i]), cmd.c_str(), strlen(cmd.c_str()));
		}
	}
}

void window3::execute(){
}

window3::window3() : window3_glade(){
	vte = vte_terminal_new();
	vte_terminal_set_background_transparent(VTE_TERMINAL(vte), FALSE);
	vte_terminal_set_size(VTE_TERMINAL(vte), 80, 25);
	vte_terminal_set_scroll_on_keystroke(VTE_TERMINAL (vte), TRUE);
	gtk_container_add(GTK_CONTAINER(align->gobj()), vte);
	gtk_widget_show(vte);
	//handler = g_signal_connect(vte, "child-exited", G_CALLBACK(func), (void *) this);
	vte_terminal_fork_command(VTE_TERMINAL(vte), NULL, NULL, NULL,
							NULL, FALSE, FALSE, FALSE);
	created = used = 0;
	for (int i=0;i<20;i++){
		vtes[i] = vte_terminal_new();
		vte_terminal_fork_command(VTE_TERMINAL(vtes[i]), NULL, NULL, NULL,
								NULL, FALSE, FALSE, FALSE);
	}

}


void window3::create(int nn) {
	for (int i = created; i < nn ; i++){

		std::ostringstream oss;
		oss << "N#" << i;

		Gtk::Alignment * align;
		Gtk::Label * label;
		Gtk::Label * nblabel;

		align = Gtk::manage(new class Gtk::Alignment(0.5, 0.5, 1, 1));
		label = Gtk::manage(new class Gtk::Label(_("")));
		Gtk::Frame *frame15 = Gtk::manage(new class Gtk::Frame());
		nblabel = Gtk::manage(new class Gtk::Label(_(oss.str())));
		label->set_alignment(0.5, 0.5);
		label->set_padding(0, 0);
		label->set_justify(Gtk::JUSTIFY_LEFT);
		label->set_line_wrap(false);
		label->set_use_markup(true);
		label->set_selectable(false);
		frame15->set_shadow_type(Gtk::SHADOW_NONE);
		frame15->set_label_align(0, 0.5);
		frame15->add(*align);
		frame15->set_label_widget(*label);
		nblabel->set_alignment(0.5, 0.5);
		nblabel->set_padding(0, 0);
		nblabel->set_justify(Gtk::JUSTIFY_LEFT);
		nblabel->set_line_wrap(false);
		nblabel->set_use_markup(false);
		nblabel->set_selectable(false);
		notebook1->set_flags(Gtk::CAN_FOCUS);
		notebook1->set_show_tabs(true);
		notebook1->set_show_border(true);
		notebook1->set_tab_pos(Gtk::POS_TOP);
		notebook1->set_scrollable(false);
		notebook1->append_page(*frame15, *nblabel);
		notebook1->pages().back().set_tab_label_packing(false, true,
				Gtk::PACK_START);
		align->show();
		label->show();
		frame15->show();
		nblabel->show();
		notebook1->show();

		//vtes[i] = vte_terminal_new();
		vte_terminal_set_background_transparent(VTE_TERMINAL(vtes[i]), FALSE);
    	vte_terminal_set_size(VTE_TERMINAL(vtes[i]), 80, 25);
		vte_terminal_set_scroll_on_keystroke(VTE_TERMINAL (vtes[i]), TRUE);
		gtk_container_add(GTK_CONTAINER(align->gobj()), vtes[i]);
		gtk_widget_show(vtes[i]);

//		vte_terminal_fork_command(VTE_TERMINAL(vtes[i]), NULL, NULL, NULL,
//						NULL, FALSE, FALSE, FALSE);

/*		int idx = 0, j = 0, pos;
		std::string args = this->args;
		pos = args.find_first_of(' ', idx);
		while (pos != std::string::npos) {
			std::string val = args.substr(idx, pos - idx);
			argv[j + 1] = (char *) malloc(256 * sizeof(char));
			if (val.compare("%i") == 0) {
				sprintf(argv[j + 1], "%d", i);
			} else if (val.compare("%n") == 0) {
				sprintf(argv[j + 1], "%d", nn);
			} else {
				sprintf(argv[j + 1], "%s", val.c_str());
			}
			idx = pos + 1;
			pos = args.find_first_of(' ', idx);

			j++;
		}
		argv[j + 1] = 0;
		fprintf(stderr, "argv[%d]=0\n", i);
		//pids[i] = vte_terminal_fork_command(VTE_TERMINAL(vtes[i]), command.c_str(), argv, NULL,
		//s.c_str(), FALSE, FALSE, FALSE);
	*/
	}
	used = nn;
	created = nn;
}
