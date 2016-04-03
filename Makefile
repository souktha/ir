
#EXTRA_CFLAGS += -DDEBUG_ON

obj-m	+= cir.o

all:
	make ARCH=arm CROSS_COMPILE=arm-linux- -C $(KERNELDIR) M=`pwd` modules

clean:
	make -C $(KERNELDIR) M=`pwd` clean
