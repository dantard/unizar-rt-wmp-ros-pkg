all: lib copy #test_ioctl #envio recepcion
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

lib: objs
	ar cru librt-wmp-ks.a wmp_interface.o
	ranlib librt-wmp-ks.a

objs: wmp_interface.c ../../../core/interface/wmp_interface.h ../include/ioctl_interface.h
	gcc -Wall -I.. -I../../.. -I../../src -c wmp_interface.c

install:
#	mkdir -p /lib/modules/`uname -r`/kernel/drivers/rt-wmp
#	cp rt_wmp_interface.ko /lib/modules/`uname -r`/kernel/drivers/rt-wmp
#	make -C /lib/modules/`uname -r`/build M=$(PWD) modules_install INSTALL_MOD_DIR=kernel/drivers/rt-wmp/
	cp librt-wmp-ks.a /usr/local/lib

#test_ioctl: test_ioctl.c ioctl_interface.h
#	gcc -o test_ioctl test_ioctl.c -Wall -I.. -I../../.. -L. -lrt-wmp-ks

#envio: envio.c ioctl_interface.h
#	gcc -o envio envio.c -Wall -I.. -I../../.. -L. -lrt-wmp

#recepcion: recepcion.c ioctl_interface.h
#	gcc -o recepcion recepcion.c -Wall -I.. -I../../.. -L. -lrt-wmp

clean:
#	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f test_ioctl *.o *.a
copy:
	mkdir -p ../../../../lib-linux_ks
	cp librt-wmp-ks.a ../../../../lib-linux_ks
