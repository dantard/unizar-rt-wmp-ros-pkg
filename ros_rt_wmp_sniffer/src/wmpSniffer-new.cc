// generated 2012/12/15 17:58:03 CET by danilo@dm1.(none)
// using glademm V2.6.0
//
// newer (non customized) versions of this file go to wmpSniffer-new.cc_new

// This file is for your program, I won't touch it again!

#include <config.h>
#include <gtkmm/main.h>

#include "main_window.hh"
#include "rec_param_dlg.hh"
#include "window1.hh"
#include "window2.hh"
#include "window3.hh"

int main(int argc, char **argv)
{  
   
   Gtk::Main m(&argc, &argv);

main_window *main_window = new class main_window();

rec_param_dlg *rec_param_dlg = new class rec_param_dlg();

window1 *window1 = new class window1();

window2 *window2 = new class window2();

window3 *window3 = new class window3();
   m.run(*main_window);
delete main_window;
delete rec_param_dlg;
delete window1;
delete window2;
delete window3;
   return 0;
}
