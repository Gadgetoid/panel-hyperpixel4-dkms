obj-m := panel-pimoroni-hyperpixel4.o
KVERSION := $(shell uname -r)
KERNELDIR := /lib/modules/${KVERSION}/build

all:
	$(MAKE) -C ${KERNELDIR} M=$(PWD)/src modules

clean:
	$(MAKE) -C ${KERNELDIR} M=$(PWD)/src clean
