#
## Makefile for the Internal DPRAM Driver
#
#

MOD_NAME = wtlfota_idpram
obj-m := $(MOD_NAME).o
$(MOD_NAME)-y := wtlfota_idpram_main.o uio.o


all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean:
	rm -rf *.o *.ko *.mod.c *.mod.o *~ .*.cmd .*.o.cmd Module.symvers modules.order .tmp_versions

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install

