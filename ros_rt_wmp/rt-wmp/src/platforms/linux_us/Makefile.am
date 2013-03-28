SUBDIRS = . hwi
AM_CPPFLAGS = @WMP_CPPFLAGS@ -I$(top_srcdir)/src -I$(top_srcdir)/src/platforms/linux_us

lib_LIBRARIES = libwmp_linux_us.a
libwmp_linux_us_a_SOURCES =  wmpInit.c wmp_misc.c
LDADD= -lncurses -lm -lpthread

	
