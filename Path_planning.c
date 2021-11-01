/*
 \file		Pathplanning.c
 \author	${user}
 \date		${date}
 \brief		Simple Hello World! for the Ev3
*/

#include <stdio.h>
#include <math.h>

#define M_PI 3.1415927
#define MAX_OBSTACLES 25 /* maximum number of obstacles */

// int num_obstacles = 3; /* number of obstacles */

// double obstacle[MAX_OBSTACLES][2] = /* obstacle locations */
// {{0.305*2, 0.305*2},{0.305*3, 0.305*3},{0.305*4, 0.305*4},{-1,-1},{-1,-1},
// {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
// {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
// {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},
// {-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}};

// double start[2] = {0.305*1, 0.305*1}; /* start location */
// double goal[2] = {0.305*5,0.305*4}; /* goal location */

// int currAngle = 270;
// double currLocation[2]={0.305, 1.219}; //TODO equalize to start

//-------------------------------------------------------------------
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
//------------------------------------------------------------------------


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
    int p;
    double tempArray2[2];
    for (p=0; p<num_obstacles; p++){
        tempArray2[0] = obstacle[alreadyFoundObstacle[i-1]][0]; // x of new start position
        tempArray2[1] = obstacle[alreadyFoundObstacle[i-1]][1]; // y of the new start position 
        findNearestObstacle(tempArray2, obstacle); // search from remaining obstacles the nearest to the new start position i.e obstacle pointed by recent/last element of alreadyFoundObstacle array 
    }
}

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
        tempArr[0] =  obstaclesPositions[alreadyFoundObstacle[j]][0];
        tempArr[1] =  obstaclesPositions[alreadyFoundObstacle[j]][1];
        printf("\n%d: (%f, %f) - ",j+1,obstaclesPositions[alreadyFoundObstacle[j]][0],obstaclesPositions[alreadyFoundObstacle[j]][1]);
        
        // moveX1Y1toX2Y2(startPosition,tempArr);
        printf("move from (%f, %f) to (%f, %f)", startPosition[0], startPosition[1], tempArr[0], tempArr[1]);
        startPosition[0]=tempArr[0];
        startPosition[1]=tempArr[1];
    }

    // moveX1Y1toX2Y2(startPosition, goal);
    printf("\n   Move to goal         - move from (%f, %f) to (%f, %f)\n", startPosition[0], startPosition[1], goal[0], goal[1]);
}

int main(void)
{
    pathPlanner();

    // printf("\n");
    // int q;
    // for (q=0; q<i; q++){
    //     printf("already found obstacle %d: (%f, %f)\n", q+1, obstacle[alreadyFoundObstacle[q]][0], obstacle[alreadyFoundObstacle[q]][1]);
    // }

    printf("Path: ");
    avoidObstacles(start,obstacle,goal);

	return 0;
}