#ifndef __UTILS_H__
#define __UTILS_H__

#ifndef M_PI
#define M_PI 3.14159265F
#endif

float d2r(float d);
float r2d(float r);
float p2m(float p); //pxl->meter
float m2p(float m); //meter->pxl

// dimension conversion factors
extern float MAP_M;
extern float MAP_PX;

#endif
