PWD = $(shell pwd)
export PATH:=/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/:$(PATH)
TARGET = udpServer

CROSS_COMPILE = arm-linux-gnueabihf-
CC = gcc
LD = ld
CFLAGS += -g -Wall
CFLAGS += 
LDFLAGS =  
SRC:=$(shell ls *.c)
OBJ:=$(SRC:.c=.o)

all:  $(TARGET)


$(TARGET):$(OBJ)
	$(CC) $(OBJ) $(LDFLAGS) -o  $(TARGET)
	
$(OBJ):$(SRC)
	@echo $(SRC)
	$(CC) $(CFLAGS) -c $(SRC)

clean:
	rm -rf *.o
	rm -rf $(TARGET)
path:
	@echo $(PATH)


