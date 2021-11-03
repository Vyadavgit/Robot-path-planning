/*
 \file		Pathplanning.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <ev3.h>

#define M_PI 3.1415927
#define MAX_OBSTACLES 25 /* maximum number of obstacles */

int num_obstacles = 10; /* number of obstacles */

double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
 {{0.305*2, 0.305*2},{0.305*4,0.305*2},{0.305*4,0.305*3},{0.305*4,0.305*5},
 {0.305*6,0.305*3},{0.305*6,0.305*4},{0.305*6,0.305*5},{0.305*3,0.305*6},
 {0.305*2,0.305*6},{0.305*1,0.305*6},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
 {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
 {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};


 double start[2] = {0.305*1, 0.305*1}; /* start location */
 double goal[2] = {0.305*5,0.305*6}; /* goal location */


 int currAngle = 270;
 double currLocation[2]={0.305*1, 0.305*1}; //TODO equalize to start

 int OffsetY=0;
 int offsetX=0;

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
	OnFwdSync(OUT_BC, 25);
	}
	Off(OUT_BC);
}

void rotateFn(int output, int angle){
	RotateMotor(output, 13, ((400*angle)/90)); // OUT_B - move right, OUT_C - move left, Observation: 400 [<Angle>] = 90 degrees rotation
											  // => (400*x)/90 = x degrees rotation
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
                OffsetY = 1; // increases offset in Y
            }
            else if(currAngle==180){
                rotateFn(OUT_B, 180);
                OffsetY = 1; // increases offset in Y
            }
            else if(currAngle==270){
                rotateFn(OUT_C, 90);
                OffsetY = -1; // decreses offset in Y
            }
            else { // i.e. currAngle==360
            	// continue
            }
		currAngle = 0; //update currAngle

        // moveStraight(Xdistance);
        moveStraight(Xdistance-0.145*offsetX); // distance between the two wheels = 14.5 cm
                                               // increase Xdistance if offset is -1
                                               // decrease Xdistance if offset is 1
        offsetX = 0; // reset offsetX after adjusted movement in X axis

        currLocation[0]  = goalLocation[0]; // OR i.e. currLocation[0] = currLocation[0] + Xdistance;
	}
    else if (Xdistance<0){ // x2-x1 < 0 => rotate to angle 180 and move straight towards -ve X-axis

        if(currAngle==0){
            rotateFn(OUT_C, 180);
            OffsetY = 1; // increses offset in Y
        }
        else if (currAngle==90){
            rotateFn(OUT_C, 90);
            OffsetY = 1; // increses offset in Y
        }
        else if(currAngle==180){
            rotateFn(OUT_C, 0);
            OffsetY = 1; // increses offset in Y
        }
        else if(currAngle==270){
            rotateFn(OUT_B, 90);
            OffsetY = -1; // decreases offset in Y
        }
        else { // i.e. currAngle==360
             rotateFn(OUT_C, 180);
             OffsetY = 1;// increses offset in Y
        }

		currAngle = 180; //update currAngle
        Xdistance = (-1)* Xdistance;

        // moveStraight(Xdistance);
        moveStraight(Xdistance-0.145*offsetX); // distance between the two wheels = 14.5 cm
                                               // increase Xdistance if offset is -1
                                               // decrease Xdistance if offset is 1
        offsetX = 0; // reset offsetX after adjusted movement in X axis

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
                offsetX = 1;// increases offset in X
            }
            else if (currAngle==90){
                // continue
            }
            else if(currAngle==180){
                rotateFn(OUT_B, 90);
                offsetX = -1;// decreases offset in X
            }
            else if(currAngle==270){
                rotateFn(OUT_B, 180);
                offsetX = -1;// decreases offset in X
            }
            else { // i.e. currAngle==360
            	rotateFn(OUT_C, 90);
                offsetX = 1;// increases offset in X
            }
		currAngle = 90; //update currAngle

        // moveStraight(Ydistance);
        moveStraight(Ydistance-0.145*OffsetY); // distance between the two wheels = 14.5 cm
                                               // increase Ydistance if offset is -1
                                               // decrease Ydistance if offset is 1
        OffsetY = 0; // reset offsetY after adjusted movement in Y axis

        currLocation[1]  = goalLocation[1]; // OR i.e. currLocation[1] = currLocation[1] + Ydistance;
	}
    else if (Ydistance<0){ //y2-y1 < 0 => rotate to angle 270 and move straight towards -ve Y-axis

        if(currAngle==0){
            rotateFn(OUT_B, 90);
            offsetX = 1;// increases offset in X
        }
        else if (currAngle==90){
            rotateFn(OUT_B, 180);
            offsetX = 1;// increases offset in X
        }
        else if(currAngle==180){
            rotateFn(OUT_C, 90);
            offsetX = -1;// decreases offset in X
        }
        else if(currAngle==270){
            // continue
        }
        else { // i.e. currAngle==360
             rotateFn(OUT_B, 90);
             offsetX = 1;// increases offset in X
        }

		currAngle = 270; //update currAngle
        Ydistance = (-1)* Ydistance;

        // moveStraight(Ydistance);
        moveStraight(Ydistance-0.145*OffsetY); // distance between the two wheels = 14.5 cm
                                               // increase Ydistance if offset is -1
                                               // decrease Ydistance if offset is 1
        OffsetY = 0; // reset offsetY after adjusted movement in Y axis

        currLocation[1]  = goalLocation[1]; // OR i.e. currLocation[1] = currLocation[1] + Ydistance;
    }
    else{
        // do nothing
    }
}

// move through X axis first the Y axis
void Xfirst_moveX1Y1toX2Y2(double initialLocation[2],double goalLocation[2]){
	moveAlongXaxis(initialLocation, goalLocation);
	moveAlongYaxis(initialLocation, goalLocation);
}

// move through Y axis first the X axis
void Yfirst_moveX1Y1toX2Y2(double initialLocation[2],double goalLocation[2]){
	moveAlongYaxis(initialLocation, goalLocation);
    moveAlongXaxis(initialLocation, goalLocation);
}


//-------------------------------------path planner program-----------------------------------------------
int alreadyFoundObstacle[MAX_OBSTACLES]; // only stores the rows of obstacle array (rows of obstacles that are found to
                                        // be nearest to start positions during path planning)
int i=0;

double sqrt(double arg);

float calculateDistance(double x1y1[2], double x2y2[2]){
	double d;
	d = sqrt((x2y2[0]-x1y1[0])*(x2y2[0]-x1y1[0])+(x2y2[1]-x1y1[1])*(x2y2[1]-x1y1[1]));
    return d;
}

int obstacleFound(int r){
    int b=0;
    int m;
    for(m=0; m<i; m++){
        if (r == alreadyFoundObstacle[m]){
            b=1;
        }
    }
    return b;
}

void findNearestObstacle(double startPosition[2], double obstaclesPositions[MAX_OBSTACLES][2]){ //finds nearest obstacles in the order: start-->obstacle-->...-->obstacle-->goal
                              // i.e finds obstacle nearest to goal, then next obstacle nearest to current obstacle...and
                              // lastly finds goal location if it is the nearest one
                              // it skips the obstacles already found while looking for another nearest one
    double temp[2];
    double nearestLocation[2];
    int closestCounter;
    double min;
    double lastmin = 100; //arbitrary distance that is greater than any distance between any two coordinates
                          // in the given frame (or greater than the diagonal of the given frame as it is the longest distance in the frame)
    double distanceToGoal;

    int row;
    for (row=0; row<num_obstacles; row++){  // iterate through all obstacles to find the nearest obstacle
                                                // to the start position(the nearest obstacle found is the start position for next iterations through remaining obstacles)

        if (obstacleFound(row)==0){ // look for the closest obstacles only from the set of obstacles that haven't been found as closest before
            temp[0] = obstaclesPositions[row][0];
            temp[1] = obstaclesPositions[row][1];

            min = calculateDistance(startPosition,temp);
            if (min<=lastmin){
                lastmin = min;
                closestCounter = row;
            }
        }
    }
    nearestLocation[0] = obstaclesPositions[closestCounter][0];
    nearestLocation[1] = obstaclesPositions[closestCounter][1];

    // after the nearest obstacle is found, check if the goal is nearer than the nearest obstacle found
    distanceToGoal =  calculateDistance(startPosition, goal);
    if (distanceToGoal<=lastmin){
        nearestLocation[0] = goal[0]; // even if goal is found as the nearest location here it isn't stored as nearest obstacle in alreadyfoundobstacle
        nearestLocation[1] = goal[1]; // just for info: *here while looping for (p<num_obstacles) in pathPlanner fn the final nearestLocation holds either the location of goal or
                                      // the recent obstacle found depending on which one was found to be the nearest
                                      // (hence, nearestLocation array prints the same location i.e goal/theLastOne again and again for every iteration as there is no any other nearest location and it keeps holding the last elements)
    }

    if((nearestLocation[0] != goal[0]) && (nearestLocation[1] != goal[1])){
        alreadyFoundObstacle[i]=closestCounter; // storing rows of the obstacles already found i.e plan the path
        i++; // total obstacles found = i, last obstacle position = i-1
    }

    // printf("Nearest obstacle: (%f, %f)\n",nearestLocation[0],nearestLocation[1]); // *comment above for it printing the same elements again and again at the end (for iteration in pathPlanner function)
}

void pathPlanner(){
	findNearestObstacle(start,obstacle);
    int p;
    double tempArray2[2];
    for (p=1; p<num_obstacles; p++){
        tempArray2[0] = obstacle[alreadyFoundObstacle[i-1]][0]; // x of new start position
        tempArray2[1] = obstacle[alreadyFoundObstacle[i-1]][1]; // y of the new start position
        findNearestObstacle(tempArray2, obstacle); // search from remaining obstacles the nearest to the new start position i.e obstacle pointed by recent/last element of alreadyFoundObstacle array
    }
}

//------------------------change dirn to avoid obstacles-------------------------separate segment---------------
//check if it is obstacle
int isObstacle(double checkArr[2]){
    int is=0;
    int num;
    for(num=0;num<i;num++){
        if((checkArr[0]==obstacle[alreadyFoundObstacle[num]][0]) && (checkArr[1]==obstacle[alreadyFoundObstacle[num]][1])){
            is=1;
        }
    }
    return is;
}

void changeDirn(double startarg[2], double goalarg[2]){

    double Xcheck[2];
    Xcheck[0] = startarg[0]+0.305;
    Xcheck[1] = startarg[1];

    double XcheckNegDir[2];
    XcheckNegDir[0] = startarg[0]-0.305;
    XcheckNegDir[1] = startarg[1];

    double Ycheck[2];
    Ycheck[0] = startarg[0];
    Ycheck[1] = startarg[1]+0.305;

    double YcheckNegDir[2];
    YcheckNegDir[0] = startarg[0];
    YcheckNegDir[1] = startarg[1]-0.305;

    double XXdistance;
    double YYdistance;
    XXdistance = startarg[0]-goalarg[0];
    YYdistance = startarg[1]-goalarg[1];


    if(XXdistance>0 && isObstacle(Xcheck)==1){
        Yfirst_moveX1Y1toX2Y2(startarg,goalarg);
    }
    else if(XXdistance<0 && isObstacle(XcheckNegDir)==1){
        Yfirst_moveX1Y1toX2Y2(startarg,goalarg);
    }
    else if(YYdistance>0 && isObstacle(Ycheck)==1){
        Xfirst_moveX1Y1toX2Y2(startarg,goalarg);
    }
    else if(YYdistance<0 && isObstacle(YcheckNegDir)==1){
        Xfirst_moveX1Y1toX2Y2(startarg,goalarg);
    }
    else{
        Xfirst_moveX1Y1toX2Y2(startarg, goalarg);
    }
}
//-------------------------------------------------------------------------------

void avoidObstacles(double startPosition[2], double obstaclesPositions[MAX_OBSTACLES][2],double goalPosition[2]){
	// 1. find the nearest obstacle from start
    // 2. move to that obstacle(with safety margin 0.61)
    // 1.. repeat i.e. find the next obstacle closest to the new start position
    // 2.. repeat i.e. move to that obstacle(with safety margin 0.61)
    // 1,2.. repeat until you reach the last obstacle that is closest to the new start position (means there are no other obstacles that is
    // closer to the current obstacle than the goal itself)
    // 3. lastly move to the goal

    // 1.
    double tempArr[2];
    int j;
    for(j=0; j<i; j++){
        tempArr[0] =  obstaclesPositions[alreadyFoundObstacle[j]][0]-0.305; //avoid obstacle by .305 offset
        tempArr[1] =  obstaclesPositions[alreadyFoundObstacle[j]][1]-0.305; //avoid obstacle by .305 offset

        changeDirn(startPosition,tempArr); // move through obstacles with safety margin
        startPosition[0]=tempArr[0];
        startPosition[1]=tempArr[1];
    }

        changeDirn(startPosition, goal); // move to goal 
}

//--------------------------------------------------------------------------------------------------------


int main(void)
{
	// INFO This code template works only with recent versions of the API. If TermPrintln is not found,
	//      please update the API or use the "Hello World EV3 Project Old API" as template.

	InitEV3();

	pathPlanner();
	avoidObstacles(start,obstacle,goal);
	Wait(500);

	FreeEV3();
	return 0;
}