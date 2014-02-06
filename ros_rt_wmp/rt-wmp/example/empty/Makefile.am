ARGV1=-1
ARGV2=-1
AM_CPPFLAGS = @WMP_CPPFLAGS@ -I$(TOP_DIR)/src -I$(TOP_DIR)/src/platforms/$(WMP_WITH_PLATFORM) -I$(TOP_DIR)/src/platforms/$(WMP_WITH_PLATFORM)/interface -DARGV1=$(ARGV1) -DARGV2=$(ARGV2)

bin_PROGRAMS = empty
empty_SOURCES = main.c 
empty_DEPENDENCIES = recompile

recompile:
	make -C $(TOP_DIR)


LDADD=-L$(TOP_DIR)/lib-$(WMP_WITH_PLATFORM) -lm -lz

if WMP_PLATFORM_USLINUX
LDADD+= -lpthread -lrt -lrt-wmp -lrt -lpcap -lm
endif

if WMP_PLATFORM_KSLINUX
LDADD+= -lpthread -lrt -lrt-wmp-ks
endif



