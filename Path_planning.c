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

int currAngle = 270;
double currLocation[2]={0.305, 1.219}; //TODO equalize to start

void moveStraight(double distance){
	// Notes: 360 motor rotations = 1 wheel rotation
	// wheels r = 0.0275 m, c = 2*M_PI*r
	double r = 0.0275; //radius of wheel in meters
	double c; //circumference of wheel [meters]
	double rotations_motor; // motor rotations needed to travel the distance

	c = 2*M_PI*r; // circumference
	rotations_motor = (360*distance)/c; // rotations needed ?
										// 360 [motor rotations] = circumference(c) [distance traveled]
										// that implies x [distance] = (360*x)/c [rotations] needed

	ResetRotationCount(OUT_B);
	ResetRotationCount(OUT_C);
	while (MotorRotationCount(OUT_B)<(rotations_motor+1) && MotorRotationCount(OUT_C)<(rotations_motor+1)) {
	OnFwdSync(OUT_BC, 5);
	}
	Off(OUT_BC);
}

void rotateFn(int output, int angle){
	RotateMotor(output, 5, ((420*angle)/90)); // OUT_B - move right, OUT_C - move left, Observation: 420 [<Angle>] = 90 degrees rotation
											  // => (420*x)/90 = x degrees rotation
}

//double angleToRotate(double initial[2], double final[2]){
//	double Rangle = atan((final[1]-initial[1])/(final[0]-initial[0]));
//	TermPrintf("Rotation angle: %f\n", Rangle);
//	Wait(5000);
//	return Rangle;
//}

// move along X-axis
void moveAlongXaxis(double initialLocation[2],double goalLocation[2]){
    double Xdistance = goalLocation[0]-initialLocation[0]; // x2-x1
    TermPrintf("Current angle: %d\n", currAngle);
    TermPrintf("Xdistance: %f\n", Xdistance);

	if (Xdistance>0){ //x2-x1 > 0  => rotate to angle 0 and move straight towards +ve X-axis
            if(currAngle==0){
            	// continue
            }
            else if (currAngle==90){
                rotateFn(OUT_B, 90);
            }
            else if(currAngle==180){
                rotateFn(OUT_B, 180);
            }
            else if(currAngle==270){
                rotateFn(OUT_C, 90);
            }
            else { // i.e. currAngle==360
            	// continue
            }
		currAngle = 0; //update currAngle
        moveStraight(Xdistance);
        currLocation[0]  = goalLocation[0]; // OR i.e. currLocation[0] = currLocation[0] + Xdistance;
	}
    else if (Xdistance<0){ // x2-x1 < 0 => rotate to angle 180 and move straight towards -ve X-axis

        if(currAngle==0){
            rotateFn(OUT_C, 180);
        }
        else if (currAngle==90){
            rotateFn(OUT_C, 90);
        }
        else if(currAngle==180){
            rotateFn(OUT_C, 0);
        }
        else if(currAngle==270){
            rotateFn(OUT_B, 90);
        }
        else { // i.e. currAngle==360
             rotateFn(OUT_C, 180);
        }

		currAngle = 180; //update currAngle
        Xdistance = (-1)* Xdistance;
        moveStraight(Xdistance);
        currLocation[0]  = goalLocation[0]; // OR i.e. currLocation[0] = currLocation[0] + Xdistance;
    }
    else{
        // do nothing
    }
}

// move along Y-axis
void moveAlongYaxis(double initialLocation[2],double goalLocation[2]){
    double Ydistance = goalLocation[1]-initialLocation[1]; // y2-y1
    TermPrintf("Current angle: %d\n", currAngle);
    TermPrintf("Ydistance: %f\n", Ydistance);

	if (Ydistance>0){ //y2-y1 > 0 => rotate to angle 90 and move straight towards +ve Y-axis
            if(currAngle==0){
            	rotateFn(OUT_C, 90);
            }
            else if (currAngle==90){
                // continue
            }
            else if(currAngle==180){
                rotateFn(OUT_B, 90);
            }
            else if(currAngle==270){
                rotateFn(OUT_B, 180);
            }
            else { // i.e. currAngle==360
            	rotateFn(OUT_C, 90);
            }
		currAngle = 90; //update currAngle
        moveStraight(Ydistance);
        currLocation[1]  = goalLocation[1]; // OR i.e. currLocation[1] = currLocation[1] + Ydistance;
	}
    else if (Ydistance<0){ //y2-y1 < 0 => rotate to angle 270 and move straight towards -ve Y-axis

        if(currAngle==0){
            rotateFn(OUT_B, 90);
        }
        else if (currAngle==90){
            rotateFn(OUT_B, 180);
        }
        else if(currAngle==180){
            rotateFn(OUT_C, 90);
        }
        else if(currAngle==270){
            // continue
        }
        else { // i.e. currAngle==360
             rotateFn(OUT_B, 90);
        }

		currAngle = 270; //update currAngle
        Ydistance = (-1)* Ydistance;
        moveStraight(Ydistance);
        currLocation[1]  = goalLocation[1]; // OR i.e. currLocation[1] = currLocation[1] + Ydistance;
    }
    else{
        // do nothing
    }
}


int main(void)
{
	// INFO This code template works only with recent versions of the API. If TermPrintln is not found,
	//      please update the API or use the "Hello World EV3 Project Old API" as template.

	InitEV3();

	//TODO Place here your variables
//	double distance = 0.10; // distance to travel
//	int angle = 90; // rotation in degrees

	//TODO Place here your code
//	moveStraight(distance);
//	rotateFn(OUT_C, angle);


	// Visiblity graph algorithm
	double X1Y1[2] = {0,0};
	double X2Y2[2] = {0.15,-0.15};

    moveAlongXaxis(X1Y1, X2Y2);
    moveAlongYaxis(X1Y1, X2Y2);

	Wait(5000);
	FreeEV3();
//	TermPrintf("Press ENTER to exit");
	return 0;
}
