// generated 2012/12/16 11:18:14 CET by danilo@dm1.(none)
// using glademm V2.6.0
//
// DO NOT EDIT THIS FILE ! It was created using
// glade-- wmpSniffer.glade
// for gtk 2.24.10 and gtkmm 2.24.2
//
// Please modify the corresponding derived classes in ./src/window1.cc


#if defined __GNUC__ && __GNUC__ < 3
#error This program will crash if compiled with g++ 2.x
// see the dynamic_cast bug in the gtkmm FAQ
#endif //
#include "config.h"
#include <gtkmmconfig.h>
#if GTKMM_MAJOR_VERSION==2 && GTKMM_MINOR_VERSION>2
#define GMM_GTKMM_22_24(a,b) b
#else //gtkmm 2.2
#define GMM_GTKMM_22_24(a,b) a
#endif //
#include "window1_glade.hh"
#include <gdk/gdkkeysyms.h>
#include <gtkmm/accelgroup.h>
#include <gtkmm/radiobutton.h>

window1_glade::window1_glade(
)
{  window1 = this;
   gmm_data = new GlademmData(get_accel_group());
   treeview1 = Gtk::manage(new class Gtk::TreeView());
   scrolledwindow4 = Gtk::manage(new class Gtk::ScrolledWindow());
   
   Gtk::RadioButton::Group _RadioBGroup_token_rb;
   token_rb = Gtk::manage(new class Gtk::RadioButton(_RadioBGroup_token_rb, "Token"));
   aut_rb = Gtk::manage(new class Gtk::RadioButton(_RadioBGroup_token_rb, "Auth"));
   message_rb = Gtk::manage(new class Gtk::RadioButton(_RadioBGroup_token_rb, "Message"));
   drop_rb = Gtk::manage(new class Gtk::RadioButton(_RadioBGroup_token_rb, "Drop"));
   all_rb = Gtk::manage(new class Gtk::RadioButton(_RadioBGroup_token_rb, "All"));
   hbox10 = Gtk::manage(new class Gtk::HBox(true, 0));
   alignment5 = Gtk::manage(new class Gtk::Alignment());
   entry1 = Gtk::manage(new class Gtk::Entry());
   button10 = Gtk::manage(new class Gtk::Button(Gtk::StockID("gtk-clear")));
   button11 = Gtk::manage(new class Gtk::Button(Gtk::StockID("gtk-find")));
   button12 = Gtk::manage(new class Gtk::Button(Gtk::StockID("gtk-quit")));
   hbox11 = Gtk::manage(new class Gtk::HBox(false, 0));
   vbox7 = Gtk::manage(new class Gtk::VBox(false, 0));
   treeview1->set_flags(Gtk::CAN_FOCUS);
   scrolledwindow4->set_flags(Gtk::CAN_FOCUS);
   scrolledwindow4->set_shadow_type(Gtk::SHADOW_IN);
   scrolledwindow4->set_policy(Gtk::POLICY_NEVER, Gtk::POLICY_AUTOMATIC);
   scrolledwindow4->add(*treeview1);
   token_rb->set_flags(Gtk::CAN_FOCUS);
   token_rb->set_mode(true);
   aut_rb->set_flags(Gtk::CAN_FOCUS);
   aut_rb->set_mode(true);
   message_rb->set_flags(Gtk::CAN_FOCUS);
   message_rb->set_mode(true);
   drop_rb->set_flags(Gtk::CAN_FOCUS);
   drop_rb->set_mode(true);
   all_rb->set_flags(Gtk::CAN_FOCUS);
   all_rb->set_mode(true);
   hbox10->unset_flags(Gtk::CAN_FOCUS);
   hbox10->pack_start(*token_rb, Gtk::PACK_SHRINK, 0);
   hbox10->pack_start(*aut_rb, Gtk::PACK_SHRINK, 0);
   hbox10->pack_start(*message_rb, Gtk::PACK_SHRINK, 0);
   hbox10->pack_start(*drop_rb, Gtk::PACK_SHRINK, 0);
   hbox10->pack_start(*all_rb, Gtk::PACK_SHRINK, 0);
   alignment5->unset_flags(Gtk::CAN_FOCUS);
   alignment5->add(*hbox10);
   entry1->set_flags(Gtk::CAN_FOCUS);
   button10->set_flags(Gtk::CAN_FOCUS);
   button11->set_flags(Gtk::CAN_FOCUS);
   button12->set_flags(Gtk::CAN_FOCUS);
   hbox11->unset_flags(Gtk::CAN_FOCUS);
   hbox11->pack_start(*button10);
   hbox11->pack_start(*button11);
   hbox11->pack_start(*button12);
   vbox7->unset_flags(Gtk::CAN_FOCUS);
   vbox7->pack_start(*scrolledwindow4);
   vbox7->pack_start(*alignment5, Gtk::PACK_SHRINK, 0);
   vbox7->pack_start(*entry1, Gtk::PACK_SHRINK, 0);
   vbox7->pack_start(*hbox11, Gtk::PACK_SHRINK, 0);
   window1->unset_flags(Gtk::CAN_FOCUS);
   window1->set_title("window3");
   window1->add(*vbox7);
   treeview1->show();
   scrolledwindow4->show();
   token_rb->show();
   aut_rb->show();
   message_rb->show();
   drop_rb->show();
   all_rb->show();
   hbox10->show();
   alignment5->show();
   entry1->show();
   button10->show();
   button11->show();
   button12->show();
   hbox11->show();
   vbox7->show();
   window1->show();
   button10->signal_clicked().connect(sigc::mem_fun(*this, &window1_glade::on_button10_clicked), false);
   button11->signal_clicked().connect(sigc::mem_fun(*this, &window1_glade::on_button11_clicked), false);
   button12->signal_clicked().connect(sigc::mem_fun(*this, &window1_glade::on_button12_clicked), false);
}

window1_glade::~window1_glade()
{  delete gmm_data;
}
