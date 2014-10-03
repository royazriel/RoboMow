export PATH:=/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/:$(PATH)
TARGET = libDw1000
CROSS_COMPILE = arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
AR = $(CROSS_COMPILE)ar
CFLAGS += -g -Wall -I/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/libc/usr/include
LDFLAGS =
SRC:=$(shell ls *.c)
OBJ:=$(SRC:.c=.o)

all: $(TARGET).a

$(TARGET).a:$(OBJ)
	$(AR) rc $(TARGET).a $(OBJ)

$(TARGET):$(OBJ)
	$(CC) $(LDFLAGS) -o $@ $< 

$(OBJ):$(SRC)
	$(CC) $(CFLAGS) -c $(SRC)

clean:
	rm -rf *.o
	rm -rf $(TARGET) $(TARGET).a

path:
	@echo $(PATH)
