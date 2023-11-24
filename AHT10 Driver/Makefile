KERNELDIR := /home/huangxiheng/IMX6/linux-imx-4.1.15-2.1.0
CURRENT_PATH := $(shell pwd)

obj-m := aht10.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
