/*
 \file		Pathplanning.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>

#define M_PI 3.1415927
#define MAX_OBSTACLES 25 /* maximum number of obstacles */

int num_obstacles = 13; /* number of obstacles */

double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
{{0.61, 2.743},{0.915, 2.743},{1.219, 2.743},{1.829, 1.219},
{1.829, 1.524},{ 1.829, 1.829}, {1.829, 2.134},{2.743, 0.305},
{2.743, 0.61},{2.743, 0.915},{2.743, 2.743},{3.048, 2.743},
{3.353, 2.743},
{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
{-1,-1},{-1,-1},{-1,-1}};

double start[2] = {0.305, 1.219}; /* start location */
double goal[2] = {3.658, 1.829}; /* goal location */

int main(void)
{
	// INFO This code template works only with recent versions of the API. If TermPrintln is not found,
	//      please update the API or use the "Hello World EV3 Project Old API" as template.

	// Notes: 360 motor rotations = 1 wheel rotation
	// tire r = 0.025 m, c = 2*M_PI*r


	InitEV3();
	//TODO Place here your variables
	double r = 0.025; //radius of wheel in meters
	double c; //circumference of wheel [meters]
	double distance; // distance to travel
	double rotations_motor; // motor rotations needed to travel the distance

	//TODO Place here your code
	c = 2*M_PI*r; // circumference
	distance = 0.30;
	rotations_motor = (360*distance)/c; // rotations needed ?
										// 360 [motor rotations] = circumference(c) [distance traveled]
										// that implies x [distance] = (360*x)/c [rotations] needed


	ResetRotationCount(OUT_B);
	ResetRotationCount(OUT_C);
	while (MotorRotationCount(OUT_B)<(rotations_motor+1) && MotorRotationCount(OUT_C)<(rotations_motor+1)) {
	OnFwdSync(OUT_BC, 5);
	TermPrintf("B: %d, C: %d\n", MotorRotationCount(OUT_B), MotorRotationCount(OUT_C));
	}
	Off(OUT_BC);

	Wait(5000);

	FreeEV3();
	return 0;
}
