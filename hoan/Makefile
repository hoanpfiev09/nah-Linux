#Đặt Makefile cùng với thư mục có source code (Lưu ý, dùng Makefile chứ KHÔNG dùng makefile)
# make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- : Sử dụng modules này cho arm64 
# -C /home/hoan/renesas-drivers : đường dẫn đến thư mục Linux kernel, ở đây là renesas-drivers
# M=$(PWD). Nó build ở thư mục naỳ?
# all: đoạn này thực hiện khi gõ make
# clean: đoạn này nó thực hiện khi gõ make clean.
########################################################################
#hoan@hoan-pfiev09:~/renesas-drivers/hoan$ make
#make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C /home/hoan/renesas-drivers M=/home/hoan/renesas-drivers/hoan modules
#make[1]: Entering directory '/home/hoan/renesas-drivers'
#arch/arm64/Makefile:23: ld does not support --fix-cortex-a53-843419; kernel may be susceptible to erratum
#arch/arm64/Makefile:44: Detected assembler with broken .inst; disassembly will be unreliable
#  CC [M]  /home/hoan/renesas-drivers/hoan/hello.o
#  Building modules, stage 2.
#  MODPOST 1 modules
#  CC      /home/hoan/renesas-drivers/hoan/hello.mod.o
#  LD [M]  /home/hoan/renesas-drivers/hoan/hello.ko
#make[1]: Leaving directory '/home/hoan/renesas-drivers'
#hoan@hoan-pfiev09:~/renesas-drivers/hoan$
#
###########################################################################

PWD := $(shell pwd)
obj-m += i2c-rcar.o

all:
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C /home/hoan/linux M=$(PWD) modules
	
clean:
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C /home/hoan/linux M=$(PWD) clean
