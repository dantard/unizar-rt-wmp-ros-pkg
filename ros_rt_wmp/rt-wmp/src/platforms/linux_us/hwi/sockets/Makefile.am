SUBDIRS = .
AM_CPPFLAGS = @WMP_CPPFLAGS@ -I$(top_srcdir)/src -I$(top_srcdir)/src/platforms/linux_us

lib_LIBRARIES = libwmp_linux_us_sockets.a
libwmp_linux_us_sockets_a_SOURCES =  ll_com.c 
LDADD= -lncurses -lm -lpthread

	
