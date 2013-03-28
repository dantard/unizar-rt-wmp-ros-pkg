/*------------------------------------------------------------------------
 *---------------------           WMPSNIFFER          --------------------
 *------------------------------------------------------------------------
 *                                                         V7.0B  11/05/10
 *
 *
 *  File: wmpSniffer.cc
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

#include <sys/types.h>
#include <unistd.h>
using namespace std;

//#include <config.h>
#include <gtkmm/main.h>

#include "main_window.hh"
#include "bridge.hh"
#include "misc.h"
#include "BoundedHash.h"
#include <glibmm/keyfile.h>
#include <signal.h>
#include <pcap_layer.h>

char iface[16];
char sniff_iface[16];

void sig_hand(int n) {
	Gtk::Main::quit();
}

int main(int argc, char **argv) {

	char * param = NULL;
	if (argc > 2) {
		fprintf(
				stderr,
				"wmpSniffer v0.1\nUsage: %s [sniff iface]\nSet recording parameters in %s.conf\n",
				argv[0], argv[0]);
		exit(0);
	} else if (argc == 2) {
		param = argv[1];
	}

	pcap_init(strdup("lo"), 3);

	pid_t pID = vfork(); //smarter fork, pauses parent process till done
	if (pID == 0) {

		setresuid(getuid(), getuid(), getuid()); //set all UID's
		setresgid(getgid(), getgid(), getgid()); //set all GID's

		g_thread_init(NULL);
		gdk_threads_init();

		Gtk::Main m(&argc, &argv);
		main_window *main_window = new class main_window(param);
		Gtk::Main::signal_quit().connect(
				sigc::mem_fun(main_window, &main_window::on_exit));
		signal(SIGINT, sig_hand);

		gdk_threads_enter();
		try {
			m.run(*main_window);
		} catch (...) {

		}
		gdk_threads_leave();

		delete main_window;
		_exit(0);
	} else{
		return 0;
	}
}
