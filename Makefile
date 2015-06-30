MODULE_NAME = gslx680_ts_acpi

#CROSS_COMPILE ?= arm-linux-gnueabihf-
#ARCH ?= arm
ARCH := $(shell uname -m | sed -e s/i.86/i386/)
KVER := $(shell uname -r)
KSRC := /lib/modules/$(KVER)/build

obj-m += gslx680_ts.o

.PHONY: all modules clean

all: modules

modules:
	make -C $(KSRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules

clean:
	make -C $(KSRC) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) clean
