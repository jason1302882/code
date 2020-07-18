PRODUCT_PACKAGES += gslX680.ko
PRODUCT_COPY_FILES += bsp/kernel/kernel4.14/drivers/input/touchscreen/gslX680/init.gslX680.rc:$(PRODUCT_OUT)/system/etc/init/init.gslX680.rc \
			bsp/kernel/kernel4.14/drivers/input/touchscreen/gslX680/gslX680_ts.kl:$(PRODUCT_OUT)/system/usr/keylayout/gslX680_ts.kl \
			bsp/kernel/kernel4.14/drivers/input/touchscreen/gslX680/gslX680_ts.idc:$(PRODUCT_OUT)/system/usr/idc/gslX680_ts.idc
