PROJECT=$(shell pwd)
ROOT=$(PROJECT)/../../..
ARCH=arm
UBOOT=u-boot-2009.08
ZLIB=zlib-1.2.3
LZO=lzo-2.03
UUID=e2fsprogs-1.42
MTD=mtd-utils-d37fcc0
BUSYBOX=busybox-1.15.0
LINUX_DIR=$(PROJECT)/$(LINUX)
UBOOT_DIR=$(PROJECT)/$(UBOOT)
BOOT_FOLDER=$(PROJECT)/boot
MTD_INSTALL=$(PROJECT)/fs_mtd_install
FILESYSTEM=$(PROJECT)/fs
FILESYSTEM_FOR_NFS=$(PROJECT)/fs_for_nfs
FILESYSTEM_FOR_FLASHING=$(PROJECT)/fs_for_flashing
BOOTLETS=$(PROJECT)/imx-bootlets-src-10.12.01
ELF2SB=$(PROJECT)/elftosb-10.12.01
L2_IMX_SCRIPTS=$(PROJECT)/L2.6.35_10.12.01_SDK_scripts
KOBS=$(PROJECT)/kobs-ng-10.12.01
BUSYBOX_DIR=$(PROJECT)/$(BUSYBOX)
NANDTEST=nand-test-11-10-2008
NANDTEST_DIR=$(PROJECT)/nand-tests
ifndef CROSS_COMPILE
	CROSS_COMPILE=arm-none-linux-gnueabi-
endif
CROSS=$(CROSS_COMPILE)
WITHOUT_XATTR=1

export PATH:=$(HOME)/arm-2009q1/bin:/opt/arm-2009q1/bin:$(UBOOT_DIR)/tools:$(ELF2SB)/bld/linux:$(PATH)
export TARGET CROSS_COMPILE ARCH LINUX_DIR CROSS WITHOUT_XATTR FILESYSTEM

KERNELRELEASE=$(shell cd $(LINUX); make -s kernelrelease)
NAND_TEST_DRIVERS=$(shell find nand-tests/ -name "*.ko")
MODULE_DEP_PREFIX=/lib/modules/$(KERNELRELEASE)

.PHONY: nand-test modules

all: uboot kerneloldconfig kernel build_modules busybox mtd kobs bootlets nand-test

include $(ROOT)/tools/build/rules.mk

clean: assert-root
	rm -fr $(LINUX) $(UBOOT) $(ZLIB) $(LZO) $(UUID) $(MTD) \
		$(MTD_INSTALL) $(FILESYSTEM) $(FILESYSTEM_FOR_NFS) \
		$(BOOTLETS) $(ELF2SB) $(L2_IMX_SCRIPTS) $(KOBS) \
		$(FILESYSTEM_FOR_FLASHING) $(BUSYBOX_DIR)

version:
	cd $(LINUX); make -s kernelversion

release:
	@cd $(LINUX); make -s kernelrelease

freescale-patch: L2.6.35_10.12_SDK_5V_SUPPLY_PATCH.tar.gz assert-nonroot
	tar xf $<

uboot: $(UBOOT).tar.bz2 \
	001_UBOOT_configuration_for_target_board.patch \
	assert-nonroot
	if (!(test -d $(UBOOT))); then \
		tar xf $<; \
		patch -p0 < 001_UBOOT_configuration_for_target_board.patch; \
		cd $(UBOOT); \
		make mx28_evk_config; \
	fi
	cd $(UBOOT); make

kerneloldconfig: \
	$(LINUX).tar.bz2 \
	freescale-patch \
	assert-nonroot \
	001_rfd_addons_$(LINUX).patch \
	002_free_gpio98_for_export_$(LINUX).patch \
	003_adding_gpios_$(LINUX).patch \
	004_optimize_nand_drivers_for_micron_$(LINUX).patch \
	005_mxs_frame_break_support_for_mbus_$(LINUX).patch \
	0004-MX28-kernel-5V-only-configuration-bug-fixes-and-improvement.patch \
	006_DDR2_for_target_board_$(LINUX).patch \
	007_enlarge_spidev_bufsize_$(LINUX).patch \
	008_mbus_enhancment_$(LINUX).patch \
	009_fixing_ethernet_init_$(LINUX).patch
	if (test -d $(LINUX_DIR)); then \
		echo "###################################################################"; \
		echo "# CANNOT RUN kerneloldconfig. PLEASE REMOVE $(LINUX) FOLDER OR RUN:"; \
		echo "# make kernel                                                     #"; \
		echo "###################################################################"; \
	else \
		echo "Unpacking kernel tar..."; \
		tar xf $<; \
		patch -p0 < 001_rfd_addons_$(LINUX).patch; \
		patch -p0 < 002_free_gpio98_for_export_$(LINUX).patch; \
		patch -p0 < 003_adding_gpios_$(LINUX).patch; \
		patch -p0 < 004_optimize_nand_drivers_for_micron_$(LINUX).patch; \
		patch -p0 < 005_mxs_frame_break_support_for_mbus_$(LINUX).patch; \
		patch -p0 < 007_enlarge_spidev_bufsize_$(LINUX).patch; \
		patch -p0 < 008_mbus_enhancment_$(LINUX).patch; \
		patch -p0 < 009_fixing_ethernet_init_$(LINUX).patch; \
		cd $(LINUX_DIR); \
		patch -p1 < ../0004-MX28-kernel-5V-only-configuration-bug-fixes-and-improvement.patch; \
		cd $(PROJECT); \
		patch -p0 < 006_DDR2_for_target_board_$(LINUX).patch; \
		cp kernel.config $(LINUX)/.config; \
		cd $(LINUX); \
		make oldconfig; \
	fi

config_measure_usb:
	@if (!(grep "CONFIG_MEASURE_USB_THROUGHPUT_GADGET_ETHERNET_CONTROLLER=" $(LINUX)/.config)); then \
		patch -p0 < measure_usb_speed_from_gagdet_ethernet_driver.patch; \
		cd $(LINUX); \
		sed "s;CONFIG_USB_GADGET=y;CONFIG_USB_GADGET=y\nCONFIG_MEASURE_USB_THROUGHPUT_GADGET_ETHERNET_CONTROLLER=5;" -i .config; \
		make silentoldconfig; \
		echo "########################################################"; \
		echo "#Configuration is done, please run 'make build_modules'#"; \
		echo "########################################################"; \
	else \
		echo "#############################################"; \
		echo "#Configuration is already exists, no changes#"; \
		echo "#############################################"; \
	fi

kernel: assert-nonroot
	cd $(LINUX); make uImage -j 4

kernel_distclean: assert-nonroot
	cd $(LINUX); make distclean

kernel_clean: assert-nonroot
	cd $(LINUX); make clean

clean_modules: assert-nonroot
	cd modules; make clean

config: assert-nonroot
	cd $(LINUX); make menuconfig

zlib: ../$(ZLIB).tar.bz2 assert-nonroot
	mkdir -p $(MTD_INSTALL)
	if (!(test -d $(ZLIB))); then \
		tar xf $<; \
		cd $(ZLIB); \
		./configure --prefix=$(MTD_INSTALL); \
		sed "s;CC=gcc;CC=arm-none-linux-gnueabi-gcc;" -i Makefile; \
		sed "s;CPP=gcc;CPP=arm-none-linux-gnueabi-gcc;" -i Makefile; \
		sed "s;LDSHARED=gcc;LDSHARED=arm-none-linux-gnueabi-gcc;" -i Makefile; \
		sed "s;RANLIB=ranlib;RANLIB=arm-none-linux-gnueabi-ranlib;" -i Makefile; \
		sed "s;AR=ar;AR=arm-none-linux-gnueabi-ar;" -i Makefile; \
	fi
	cd $(ZLIB); make
	cd $(ZLIB); make install

lzo: ../$(LZO).tar.gz assert-nonroot
	mkdir -p $(MTD_INSTALL)
	if (!(test -d $(LZO))); then \
		tar xf $<; \
		cd $(LZO); \
		./configure --host=arm-none-linux-gnueabi --prefix=$(MTD_INSTALL); \
	fi
	cd $(LZO); make
	cd $(LZO); make install

uuid: ../$(UUID).tar.gz assert-nonroot
	if (!(test -d $(UUID))); then \
		tar xf $<; \
		cd $(UUID); \
		./configure --host=arm-none-linux-gnueabi CC=arm-none-linux-gnueabi-gcc \
			--prefix=$(MTD_INSTALL); \
	fi
	cd $(UUID); make
	cd $(UUID); make install
	cd $(UUID)/lib/uuid; make
	cd $(UUID)/lib/uuid; make install

mtd: ../$(MTD).tar.gz zlib lzo uuid assert-nonroot
	if (!(test -d $(MTD))); then \
		tar xf $<; \
		cd $(MTD); \
		sed "s;PREFIX=/usr;PREFIX=$(MTD_INSTALL);" -i common.mk; \
		sed 's;CFLAGS ?=;\
		ZLIBCPPFLAGS=-I$$(PREFIX)/include \n \
		LZOCPPFLAGS=-I$$(PREFIX)/include/lzo \n \
		ZLIBLDFLAGS=-L$$(PREFIX)/lib \n \
		LZOLDFLAGS=-L$$(PREFIX)/lib \n \
		LDFLAGS += $$(ZLIBLDFLAGS) $$(LZOLDFLAGS) \n \
		CFLAGS ?= $$(ZLIBCPPFLAGS) $$(LZOCPPFLAGS);' -i common.mk; \
		sed "s;flash_otp_info;flash_otp_info flash_otp_write flash_otp_lock;" -i Makefile; \
		sed "s;install:: $${BINS} $${SCRIPTS};install::;" -i Makefile; \
		sed "s;flash_eraseall;../flash_eraseall;" -i Makefile; \
		sed "s;install::;install::\n#install::;" -i Makefile; \
		sed 's:install -m:cd $$(BUILDDIR); install -m:' -i Makefile; \
	fi
	cd $(MTD); make 
	cd $(MTD); make install

kobs: $(KOBS).tar.gz assert-nonroot
	if (!(test -d $(KOBS))); then \
		tar xf $<; \
		cd $(KOBS); \
		./configure --host=arm-none-linux-gnueabi --prefix=$(MTD_INSTALL); \
	fi
	cd $(KOBS); make
	cd $(KOBS); make install

elf2sb: $(ELF2SB).tar.gz assert-nonroot
	if (!(test -d $(ELF2SB))); then \
		tar xf $<; \
		sed "s;LIBS =     -lstdc++;LIBS =     -lstdc++ -lm;" -i $(ELF2SB)/makefile.rules; \
	fi
	cd $(ELF2SB); make all

l2-imx-scripts: $(L2_IMX_SCRIPTS).tar.gz mk_mx28_sd.patch assert-nonroot
	if (!(test -d $(L2_IMX_SCRIPTS))); then \
		tar xf $<; \
		mkdir -p $(L2_IMX_SCRIPTS)/rootfs; \
		patch -p0 < mk_mx28_sd.patch; \
		ln -s $(BOOTLETS) $(L2_IMX_SCRIPTS)/rootfs/boot; \
	fi

bootlets: \
	$(BOOTLETS).tar.gz \
	freescale-patch \
	0001-MX28-imx-bootlets-adds-the-most-robust-support-for-VDD5V-only-configuration.patch \
	0002-MX28-imx-bootlets-Fix-repeated-boot-issue-with-5V-only-configuration.patch \
	0003-MX28-imx-bootlets-5V-only-configuration-bug-fixes-and-improvement.patch \
	elf2sb \
	l2-imx-scripts \
	bootlets_0001_path_fix.patch \
	bootlets_0002_boot_fix.patch \
	bootlets_0003_cmdline_walkaround.patch \
	assert-nonroot
	if (!(test -d $(BOOTLETS))); then \
		tar xf $<; \
		cd $(BOOTLETS); \
		patch -p1 < ../0001-MX28-imx-bootlets-adds-the-most-robust-support-for-VDD5V-only-configuration.patch; \
		patch -p1 < ../0002-MX28-imx-bootlets-Fix-repeated-boot-issue-with-5V-only-configuration.patch; \
		patch -p1 < ../0003-MX28-imx-bootlets-5V-only-configuration-bug-fixes-and-improvement.patch; \
		cd $(PROJECT); \
		patch -p0 < bootlets_0001_path_fix.patch; \
		patch -p0 < bootlets_0002_boot_fix.patch; \
		patch -p0 < bootlets_0003_cmdline_walkaround.patch; \
	fi
	cd $(LINUX_DIR); make -j 4 uImage
	cd $(LINUX_DIR); make zImage
	mkdir -p $(BOOTLETS)/boot
	cp $(LINUX_DIR)/arch/arm/boot/zImage $(BOOTLETS)/boot/
	cp $(UBOOT_DIR)/u-boot $(BOOTLETS)/boot/
	cd $(BOOTLETS); DEV_IMAGE=$(BOOTLETS) BOARD=iMX28_EVK make boot_prep
	cd $(BOOTLETS); DEV_IMAGE=$(BOOTLETS) BOARD=iMX28_EVK make power_prep
	cd $(BOOTLETS); DEV_IMAGE=$(BOOTLETS) BOARD=iMX28_EVK make gen_bootstream

build_modules: assert-nonroot
	cd modules; make
	cd $(LINUX); make modules -j 4

busybox: ../$(BUSYBOX).tar.bz2 busybox-makefile-fix.patch
	if (!(test -d $(BUSYBOX_DIR))); then \
		tar xf $<; \
		patch -p0 < busybox-makefile-fix.patch; \
		cp busybox.config $(BUSYBOX_DIR)/.config; \
		cd $(BUSYBOX_DIR); \
		make oldconfig; \
	fi
	cd $(BUSYBOX_DIR); make

nand-test: ../$(NANDTEST).tar.bz2 nand-test.patch assert-nonroot
	if (!(test -d $(NANDTEST_DIR))); then \
		tar xf $<; \
		patch -p0 < nand-test.patch; \
	fi
	cd $(NANDTEST_DIR); make $(ARGUMENTS)

create-fs: default_rootfs.tar.bz2 assert-root
	rm -fr $(FILESYSTEM)
	mkdir $(FILESYSTEM)
	cd $(FILESYSTEM); tar xf ../default_rootfs.tar.bz2
	cp -a $(MTD_INSTALL)/bin $(FILESYSTEM)/
	cp -a $(MTD_INSTALL)/sbin $(FILESYSTEM)/
	cp -a $(MTD_INSTALL)/etc $(FILESYSTEM)/
	cp -a etc $(FILESYSTEM)/
	cp -a $(TARGET_ROOTFS)/* $(FILESYSTEM)
	rm -fr $(FILESYSTEM)/lib/modules/*
	rm -fr $(FILESYSTEM)/dev/*
	cd $(LINUX); INSTALL_MOD_PATH=$(FILESYSTEM) make modules_install
	cd modules; INSTALL_MOD_PATH=$(FILESYSTEM) make install
	cd $(BUSYBOX); make CONFIG_PREFIX=$(FILESYSTEM) install
	depmod -a -b $(FILESYSTEM) $(KERNELRELEASE)
	sed "s;kernel;$(MODULE_DEP_PREFIX)/kernel;g" -i $(FILESYSTEM)/lib/modules/$(KERNELRELEASE)/modules.dep

create-fs-for-nfs: assert-root 
	rm -fr $(FILESYSTEM_FOR_NFS)
	mkdir $(FILESYSTEM_FOR_NFS)
	cd $(FILESYSTEM_FOR_NFS); tar xf ../nfs_rootfs.tar.bz2
	cp -fr etc $(FILESYSTEM_FOR_NFS)/
	cp -fr $(TARGET_ROOTFS)/* $(FILESYSTEM_FOR_NFS)
	cd $(LINUX); INSTALL_MOD_PATH=$(FILESYSTEM_FOR_NFS) make modules_install
	cd modules; INSTALL_MOD_PATH=$(FILESYSTEM_FOR_NFS) make install
	depmod -a -b $(FILESYSTEM_FOR_NFS) $(KERNELRELEASE)
	sed "s;kernel;$(MODULE_DEP_PREFIX)/kernel;g" -i $(FILESYSTEM_FOR_NFS)/lib/modules/$(KERNELRELEASE)/modules.dep
	cd $(BUSYBOX); make CONFIG_PREFIX=$(FILESYSTEM_FOR_NFS) install
	rm -fr $(FILESYSTEM_FOR_NFS)/boot/*
	cp -fr $(BOOTLETS)/* $(FILESYSTEM_FOR_NFS)/boot/
	cp -fr $(NAND_TEST_DRIVERS) $(FILESYSTEM_FOR_NFS)/opt
	cp -fr $(LINUX)/arch/arm/boot/uImage $(FILESYSTEM_FOR_NFS)/

create-fs-for-flashing: create-fs
	rm -fr $(FILESYSTEM_FOR_FLASHING)
	mkdir $(FILESYSTEM_FOR_FLASHING)
	cd $(FILESYSTEM_FOR_FLASHING); tar xf ../nfs_rootfs.tar.bz2
	cp -a $(BOOT_FOLDER)/rc.local $(FILESYSTEM_FOR_FLASHING)/etc/rc.d/
	cd $(FILESYSTEM); tar czf $(FILESYSTEM_FOR_FLASHING)/rootfs-nand.tar.gz .
	cp -a $(LINUX)/arch/arm/boot/uImage $(FILESYSTEM_FOR_FLASHING)/
	mkdir -p $(FILESYSTEM_FOR_FLASHING)/boot
	rm -fr $(FILESYSTEM_FOR_FLASHING)/boot/*
	cp -fr $(BOOTLETS)/* $(FILESYSTEM_FOR_FLASHING)/boot/
	chown root:root $(FILESYSTEM_FOR_FLASHING) -R

patch-web:
	 cd $(FILESYSTEM_FOR_NFS)/opt; svn co http://192.168.21.8/svn/rfdapps/branches/WEB/IMX28
	 cd $(FILESYSTEM_FOR_NFS)/opt; rm -fr web
	 cd $(FILESYSTEM_FOR_NFS)/opt; mv IMX28 web
