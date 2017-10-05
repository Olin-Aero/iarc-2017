#ifndef __UTILS_H__
#define __UTILS_H__

#ifndef M_PI
#define M_PI 3.14159265F
#endif

#define MAP_M 24.0F //+-2m
#define MAP_PX 768.0F

float d2r(float d);
float r2d(float r);
float p2m(float p); //pxl->meter
float m2p(float m); //meter->pxl
#endif
