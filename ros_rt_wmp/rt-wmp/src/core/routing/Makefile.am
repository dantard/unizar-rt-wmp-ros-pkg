AM_CPPFLAGS = @WMP_CPPFLAGS@ -I$(top_srcdir)/src/ -I$(top_srcdir)/src/platforms/$(WMP_WITH_PLATFORM)

lib_LIBRARIES = libwmp_routing.a
libwmp_routing_a_SOURCES = manage.c aura.c

if WMP_ROUTING_BASIC
SUBDIRS = basic
endif

if WMP_ROUTING_TREE
SUBDIRS = tree
endif

