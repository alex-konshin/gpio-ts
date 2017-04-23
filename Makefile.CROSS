obj-m  = gpio-ts.o

#CC_PREFIX = arm-linux-gnueabihf-
#KERNEL_DIR = /data/projects/raspberrypi/linux
#TOOLCHAIN = /data/projects/raspberrypi/toolchain/bin

.PHONY: all

all: modules

.PHONY:modules

modules:
	${MAKE} CROSS_COMPILE=${CC_PREFIX} ARCH=arm -C ${KERNEL_DIR} M=${PWD} modules
	cp ${PWD}/*.ko ${BUILD_OUTPUT_DIR}
	rm -f *.o *.ko *.mod.c .*.o .*.ko .*.mod.c .*.cmd *~
	rm -f Module.symvers Module.markers modules.order
	rm -rf .tmp_versions

clean:
	${MAKE} CROSS_COMPILE=${CC_PREFIX} ARCH=arm -C ${KERNEL_DIR} M=${PWD} clean
	rm -f *.o *.ko *.mod.c .*.o .*.ko .*.mod.c .*.cmd *~
	rm -f Module.symvers Module.markers modules.order
	rm -rf .tmp_versions
