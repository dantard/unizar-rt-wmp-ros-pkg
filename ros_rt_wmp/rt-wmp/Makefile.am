SHELL=/bin/bash
SUBDIRS = src

install:
if WMP_PLATFORM_KSLINUX
	obj-m	:= rtwmp.o
	rtwmp-y	+= src/core/RT-WMP.o

else
	cp lib-linux-us/librt-wmp.a /usr/local/lib
endif
#	( cd src && make install )

if WMP_PLATFORM_KSLINUX
insmod:
	./replace_modules
endif
#	-modprobe -a mac80211 $mod \
#	-rmmod rt_wmp_interface rt_wmp 
#	insmod modules-$(ARCHITECTURE)/rt_wmp.ko
#	insmod modules-$(ARCHITECTURE)/rt_wmp_interface.ko
#	( cd src && make install )
