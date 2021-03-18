#ifndef _FUZZY_H
#define _FUZZY_H

#define NB 0
#define NM 1
#define NS 2
#define ZO 3
#define PS 4
#define PM 5
#define PB 6

float fuzzy_kp(float e,float ec);
float fuzzy_ki(float e,float ec);
float fuzzy_kd(float e,float ec);

#endif          //_FUZZY_H