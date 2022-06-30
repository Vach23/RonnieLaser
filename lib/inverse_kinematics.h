#ifndef _INVERSE_KINEMATICS_H_
#define _INVERSE_KINEMATICS_H_

#include <stdio.h>
#include <stdlib.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <float.h>


// Length of individual parts, in paralelogram:
#define L1 20 //[mm]
#define L2 50 //[mm]
#define L3 20 //[mm]

// Initial dimensions of room:
#define Z_HEIGHT -2000

// Functions definiton
float get_phi(float x, float y);
void v_get_phi(float x, float y, float *phi);

float get_theta(float x, float y);
void v_get_theta(float x, float y, float *theta);

void get_angles(float x, float y, float *phi, float *theta);
void get_xyz(float theta, float phi, float *x, float *y, float *z);
void get_z(float x, float y, float z, float *x_final, float *z_final);
float get_final_angle(float x, float z);

float inverse_kinematics(float x, float y, int flip);
float inverse_kinematics_angle(float phi, float theta, int flip);

void v_inverse_kinematics(float x, float y, float *angle, int flip);
void v_inverse_kinematics_angle(float phi, float theta, float *angle, int flip);


void v_linear_planning(int prev_x, int prev_y, int x, int y, int *spacing, int *direction, int *steps);	

void v_linear_move(int prev_x, int prev_y, int x, int y);

int get_direction(int dx, int dy);

#endif