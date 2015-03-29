#!/bin/sh

GPIO_DIR="/sys/class/gpio"

if [ ! -d "$GPIO_DIR/gpio140" ]; then
echo 140 > /sys/class/gpio/export
fi

if [ ! -d "$GPIO_DIR/gpio141" ]; then
echo 141 > /sys/class/gpio/export
fi

if [ ! -d "$GPIO_DIR/gpio142" ]; then
echo 142 > /sys/class/gpio/export
fi
./DecaWave -d /dev/spidev2.0 -r 1 -c 0 -p 40010 -i 192.168.1.11 -s 12 -t 3000
