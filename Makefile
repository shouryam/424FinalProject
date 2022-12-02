#the one from canvas
PWD=$(shell pwd)
KERNEL_BUILD=/lib/modules/$(shell uname -r)/build

obj-m+=gpiod_driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
	sudo cp gpiod_driver.ko /lib/modules/$(shell uname -r)/misc/
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
