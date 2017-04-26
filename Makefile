obj-m  += gpio-ts.o

.PHONY: all

all: modules

.PHONY:modules


all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	rm -f *.o *.mod.c .*.o .*.ko .*.mod.c .*.cmd *~
	rm -f Module.symvers Module.markers modules.order
	rm -rf .tmp_versions

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f *.o *.ko *.mod.c .*.o .*.ko .*.mod.c .*.cmd *~
	rm -f Module.symvers Module.markers modules.order
	rm -rf .tmp_versions

