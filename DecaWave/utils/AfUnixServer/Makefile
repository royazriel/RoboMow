PWD = $(shell pwd)
export ARCH= arm
export PATH:=/home/roy/ti-dvsdk_dm3730-evm_04_03_00_06/linux-devkit/bin:$(PATH)
TARGET = AfUnixServer
DEVICE = root@192.168.1.26:/home/root

CROSS_COMPILE = arm-arago-linux-gnueabi-
CC = gcc
LD = ld
CFLAGS += -g -Wall
CFLAGS += 
LDFLAGS =  
SRC:=$(shell ls *.c)
OBJ:=$(SRC:.c=.o)

all:  $(TARGET)


$(TARGET):$(OBJ)
	$(CROSS_COMPILE)$(CC) $(OBJ) $(LDFLAGS) -o  $(TARGET)
	
$(OBJ):$(SRC)
	@echo $(SRC)
	$(CROSS_COMPILE)$(CC) $(CFLAGS) -c $(SRC)

clean:
	rm -rf *.o
	rm -rf $(TARGET)
path:
	@echo $(PATH)

install:
	scp $(TARGET) $(DEVICE)


