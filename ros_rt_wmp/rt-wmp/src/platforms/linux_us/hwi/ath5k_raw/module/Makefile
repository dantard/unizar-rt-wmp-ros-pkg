ath5k_raw-y				+= caps.o
ath5k_raw-y				+= mac80211.o
ath5k_raw-y				+= initvals.o
ath5k_raw-y				+= eeprom.o
ath5k_raw-y				+= desc.o
ath5k_raw-y				+= dma.o
ath5k_raw-y				+= qcu.o
ath5k_raw-y				+= pcu.o
ath5k_raw-y				+= phy.o
ath5k_raw-y				+= reset.o
ath5k_raw-y				+= attach.o
ath5k_raw-y				+= base.o
ath5k_raw-y				+= debug.o
obj-m				+= ath5k_raw.o

all: test_ioctl
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

test_ioctl: test_ioctl.c ath5k_interface.h
	gcc -o test_ioctl test_ioctl.c

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f modules.order
	rm -f test_ioctl
