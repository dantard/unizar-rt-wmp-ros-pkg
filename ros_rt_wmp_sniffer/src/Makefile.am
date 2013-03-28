# generated 2007/12/12 12:34:18 CET by danilo@robot22.(none)
# using glademm V2.6.0
#
# newer (non customized) versions of this file go to Makefile.am_new
CXX=g++-4.4
bin_PROGRAMS = wmpSniffer
wmpSniffer_SOURCES = \
	wmpSniffer.cc\
	main_window_glade.cc \
	main_window.cc \
	area.cc \
	bridge.cc\
	MyArea.cc \
	rec_param_dlg.cc \
	rec_param_dlg_glade.cc \
	wmp_specific.cc \
	marte_layer.cc\
	window1.cc\
	window1_glade.cc\
	window2.cc\
        window2_glade.cc\
        pcap_layer.cc\
	statistics.cc\
	prim.cc \
	buffer_layer.cc \
	basic_io.cc \
	gct.cc \
	enhanced_io.cc \
	time.cc \
	stats.cc \
	Plotter.cc\
	shmem_layer.cc\
	window3.cc\
	window3_glade.cc
#if WMP_COND_MATLAB
#wmpSniffer_SOURCES += matlab.cc
#else
#wmpSniffer_SOURCES += fake_graphs.cc
#endif

noinst_HEADERS = \
	main_window_glade.hh \
	main_window.hh \
	area.h \
	bridge.hh \
	wmp_specific.hh\
	window1.hh\
	window1_glade.hh\
	window2.hh\
	window2_glade.hh

#if WMP_COND_MATLAB
#LD_ADD= -L/opt/matlab/bin/glnx86  -L$WMP_WITH_RT_WMP_DIR/lib-linux_us -leng -lmx -lm -lut -lut -lmx -leng -lmex -lmat -lz -lrt-wmp
#endif

AM_CXXFLAGS = @GTKMM_CFLAGS@ -pg -I@WMP_WITH_RT_WMP_DIR@/src/ -Wno-deprecated-declarations -g -ggdb -Wno-deprecated
#AM_CXXFLAGS = @GTKMM_CFLAGS@ -g -ggdb -pg -I../../rt-wmp/src/ -I/opt/matlab

wmpSniffer_LDADD = @LIBS@ @GTKMM_LIBS@   -lpcap -lgthread-2.0 -lz -lvte
#-lrt-wmp

# WARNING!!! SET ->  export LD_LIBRARY_PATH="/opt/matlab/bin/glnx86:$LD_LIBRARY_PATH" in .bashrc  //DANI