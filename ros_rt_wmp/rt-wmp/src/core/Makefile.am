SUBDIRS = routing 
AM_CPPFLAGS = @WMP_CPPFLAGS@ -I$(top_srcdir)/src/ -I$(top_srcdir)/src/platforms/$(WMP_WITH_PLATFORM)

lib_LIBRARIES = libwmp_core.a
libwmp_core_a_SOURCES = wmp_utils.c frame_compress.c ml_com.c prim.c dijkstra_alg.c dijkstra.c  interface.c   nstat.c   rssi_average.c lqm.c MobileAverage.c  queues.c  RT-WMP.c  wmp_com.c wmp_misc.c flow_control.c queue_core.c queue_tx.c queue_rx.c task_timing.c


