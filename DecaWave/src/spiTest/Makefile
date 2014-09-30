export PATH:=/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/:$(PATH)
TARGET = spiTest
CROSS_COMPILE = arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
CFLAGS += -g -Wall -I/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/libc/usr/include
LDFLAGS =
SRC:=$(shell ls *.c)
OBJ:=$(SRC:.c=.o)

all: $(TARGET)


$(TARGET):$(OBJ)
	@echo "linking"
	$(CC) $(LDFLAGS) -o $@ $< 

$(OBJ):$(SRC)
	@echo "building objects"
	@echo $(SRC)
	$(CC) $(CFLAGS) -c $<

clean:
	rm -rf *.o
	rm -rf $(TARGET)
path:
	@echo $(PATH)
