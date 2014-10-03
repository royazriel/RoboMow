PWD = $(shell pwd)
export PATH:=/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/:$(PATH)
TARGET = Deca
LIB_SPI_DRIVER_FILE = libSpiDriver.a
LIB_DW_DRIVER_FILE = libDw1000.a
SPI_DRIVER_PATH = $(PWD)/../spiDriver
DW_DRIVER_PATH = $(PWD)/../dwDriver

CROSS_COMPILE = arm-linux-gnueabihf-
CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
CFLAGS += -g -Wall -I/opt/rpi/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/arm-linux-gnueabihf/libc/usr/include 
CFLAGS += -I$(SPI_DRIVER_PATH) -I$(DW_DRIVER_PATH)
LDFLAGS = -L$(SPI_DRIVER_PATH) -lSpiDriver  -L$(DW_DRIVER_PATH) -lDw1000 
SRC:=$(shell ls *.c)
OBJ:=$(SRC:.c=.o)

all: $(LIB_SPI_DRIVER_FILE) $(LIB_DW_DRIVER_FILE) $(TARGET)

$(LIB_SPI_DRIVER_FILE):
	cd $(SPI_DRIVER_PATH); make clean; make $@

$(LIB_DW_DRIVER_FILE):
	cd $(DW_DRIVER_PATH); make clean; make $@

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

mv2target:
	scp $(TARGET) pi@192.168.1.30:/home/pi

