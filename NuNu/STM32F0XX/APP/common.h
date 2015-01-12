#ifndef COMMON_H
#define COMMON_H

#include <stdio.h>
#include <stdarg.h>

#define PI 							3.14159265359f
#define PI_IN_DEG 					180
#define ONE_G_RESOLUTION 			8192.0f
#define WHEEL_RADIUS				0.045f  //45mm
#define WHEEL_PERIMETER				(2*PI*WHEEL_RADIUS)
#define DISTANCE_BETWEEN_WHEELS 	0.21   //210mm
#define RPS_HIGH_SPEED				0.75f
#define RPS_MID_SPEED				0.5f
#define RPS_LOW_SPEED				1.0//0.25f
#define OVERLAP_COVERAGE_SIZE		0.03 //30mm

#endif //COMMON_H
