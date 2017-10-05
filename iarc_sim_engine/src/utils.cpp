#include "utils.h"

float d2r(float d){
    return d*M_PI/180.0;
}
float r2d(float r){
    return r*180.0/M_PI;
}
float m2p(float m){
    return m*MAP_PX/MAP_M;
}
float p2m(float p){
    return p*MAP_M/MAP_PX;
}
