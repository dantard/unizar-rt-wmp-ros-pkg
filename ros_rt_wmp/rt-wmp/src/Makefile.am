### LINUX-US
if WMP_PLATFORM_KSLINUX

if WMP_ROUTING_BASIC
export WMP_ROUTING_basic=foo
endif

if WMP_ROUTING_TREE
export WMP_ROUTING_tree=foo
endif

if WMP_COND_KS_INTERFACE
export WMP_KS_INTERFACE=foo
endif

all:
	make -C $(KERNEL_HEADERS_LOCATION) M=$(PWD)/platforms/linux_ks/hwi/mac80211_mod/module/kernel-$(shell uname -r | cut -c 1-3) modules $(COMPILE_ARCH)
	cp platforms/linux_ks/hwi/mac80211_mod/module/kernel-$(shell uname -r | cut -c 1-3)/Module.symvers .
	make -C $(KERNEL_HEADERS_LOCATION) M=$(PWD) modules $(COMPILE_ARCH)
	( cd platforms/linux_ks/lib && make )
	mkdir -p ../modules-$(ARCHITECTURE)
	mv rt_wmp.ko platforms/linux_ks/net_device/rt_wmp_interface.ko ../modules-$(ARCHITECTURE)
	mv platforms/linux_ks/hwi/mac80211_mod/module/kernel-$(shell uname -r | cut -c 1-3)/mac80211.ko ../modules-$(ARCHITECTURE)	        

if COND_ARCH_X86
install: 
	make -C $(KERNEL_HEADERS_LOCATION) M=$(PWD) modules_install INSTALL_MOD_DIR=kernel/drivers/rt-wmp/
	depmod -a
	cp platforms/linux_ks/start_rt_wmp /usr/sbin
	chmod +x /usr/sbin/start_rt_wmp
	cp platforms/linux_ks/stop_rt_wmp /usr/sbin
	chmod +x /usr/sbin/stop_rt_wmp
	( cd platforms/linux_ks/lib && make install )
endif
	
clean:
	make -C $(KERNEL_HEADERS_LOCATION) M=$(PWD) clean
	( cd platforms/linux_ks/lib && make clean )

### LINUX-US
else

SUBDIRS = core platforms  

.PHONY : clean

all: compress copy

copy:
	-cp `find -name *.a` ../lib-$(WMP_WITH_PLATFORM)

compress:
	-rm ../lib/*
	ar cru ../lib-$(WMP_WITH_PLATFORM)/librt-wmp.a `find -name *.o`
	ranlib ../lib-$(WMP_WITH_PLATFORM)/librt-wmp.a
	-if [ ! -d ~/.rt-wmp ] ; then mkdir ~/.rt-wmp ; fi
	-if [ -f platforms/linux_us/hwi/ath5k_raw/module/ath5k_raw.ko ] ; then cp platforms/linux_us/hwi/ath5k_raw/module/ath5k_raw.ko ~/.rt-wmp ; fi
cleanobj:
	-rm `find . -name *.o`
	-rm `find . -name *.a`

lib: cleanobj all copy makelib

makelib:
	ar cru ../lib-$(WMP_WITH_PLATFORM)/librt-wmp.a `find -name *.o`
	ranlib ../lib-$(WMP_WITH_PLATFORM)/librt-wmp.a

that: cleanobj all
endif
