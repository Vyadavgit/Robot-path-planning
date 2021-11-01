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
// // double currLocation[2]={0.305, 1.219}; //TODO equalize to start

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
// double currLocation[2]={0.305, 1.219}; //TODO equalize to start
//------------------------------------------------------------------------

//-------------------------free spaces program------------------------------------------------------------
// This program creates a new 2D array of free spaces to be passed to the path planner program segment
int isObstacle(double g, double h, double obst[MAX_OBSTACLES][2]){
    int z=0;
    int a;
    for(a=0; a<num_obstacles; a++){
        if (g == obst[a][0] && h == obst[a][1]){
            z=1;
        }
    }
    return z;
}

int freeNUM = 176; // 16x11 may be the maximum free space coordinates available if no obstacles available
int freeSpaceCounter=0;
double freeSpaces[176][2]; 
void freeSpacesFinder(double obstacleSpaces[MAX_OBSTACLES][2]){
    int frameRow,frameCol;
    // for(k=0; k<freeNUM; k++){
        for(frameRow=0; frameRow<16; frameRow++){
            for(frameCol=0; frameCol<11; frameCol++){
                if(isObstacle(0.305*frameRow, 0.305*frameCol, obstacleSpaces)==0){
                    freeSpaces[freeSpaceCounter][0]= 0.305*frameRow;
                    freeSpaces[freeSpaceCounter][1]= 0.305*frameCol;
                    freeSpaceCounter++;
                }
            }
        }
    // }
}

// void printFreeSpaces(double freeCoordinates[176][2]){
//     int f;
//     printf("\nFree coordinates: \n");
//     for(f=0; f<freeSpaceCounter; f++){
//         printf("%d: (%f, %f)\n", f, freeCoordinates[f][0]/0.305, freeCoordinates[f][1]/0.305);
//     }
// }
//--------------------------------------------------------------------------------------------------------

int alreadyFoundFreeSpace[176]; // 16*10 tiles => 16*11=176 intesections 
                                // only stores the rows of free space array (rows of free spaces that are found to 
                                // be nearest to start positions during path planning)
int i=0;

double sqrt(double arg);

float calculateDistance(double x1y1[2], double x2y2[2]){
	double d;
	d = sqrt((x2y2[0]-x1y1[0])*(x2y2[0]-x1y1[0])+(x2y2[1]-x1y1[1])*(x2y2[1]-x1y1[1]));
    return d;
}

int freeSpaceFound(int r){
    int b=0;
    int m;
    for(m=0; m<i; m++){
        if (r == alreadyFoundFreeSpace[m]){
            b=1;
        }
    }
    return b;
}

void findNearestFreeSpace(double startPosition[2], double freeSpacesPositions[176][2]){ //finds nearest freespace in the order: start-->freespace-->...-->freespace-->goal
                              // i.e finds freespace nearest to goal, then next freespace nearest to current free space...and
                              // lastly finds goal location if it is the nearest one
                              // it skips the freespaces already found while looking for another nearest one
    double temp[2];
    double nearestLocation[2];
    int closestCounter;
    double min;
    double lastmin = 100; //arbitrary distance that is greater than any distance between any two coordinates
                          // in the given frame (or greater than the diagonal of the given frame as it is the longest distance in the frame)
    double distanceToGoal;

    int row;
    for (row=0; row<freeSpaceCounter; row++){  // iterate through all freespaces to find the nearest freespace
                                                // to the start position(the nearest freespace found is the start position for next iterations through remaining freespaces)

        if (freeSpaceFound(row)==0){ // look for the closest freespace only from the set of freespaces that haven't been found as closest before
            temp[0] = freeSpacesPositions[row][0];
            temp[1] = freeSpacesPositions[row][1];

            min = calculateDistance(startPosition,temp);
            if (min<=lastmin){
                lastmin = min;
                closestCounter = row;
            }
        }
    }
    nearestLocation[0] = freeSpacesPositions[closestCounter][0];
    nearestLocation[1] = freeSpacesPositions[closestCounter][1];

    // after the nearest freespace is found, check if the goal is nearer than the nearest freespace found
    distanceToGoal =  calculateDistance(startPosition, goal);
    if (distanceToGoal<=lastmin){
        nearestLocation[0] = goal[0]; // even if goal is found as the nearest location here it isn't stored as nearest freespace in alreadyFoundFreeSpace 
        nearestLocation[1] = goal[1]; // just for info: *here while looping for (p<freeSpaceCounter) in pathPlanner fn the final nearestLocation holds either the location of goal or 
                                      // the recent free space found depending on which one was found to be the nearest 
                                      // (hence, nearestLocation array prints the same location i.e goal/theLastOne again and again for every iteration as there is no any other nearest location and it keeps holding the last elements)
    }

    if((nearestLocation[0] != goal[0]) && (nearestLocation[1] != goal[1])){
        alreadyFoundFreeSpace[i]=closestCounter; // storing rows of the free spaces already found i.e plan the path
        i++; // total free spaces found = i, last free space position = i-1
    }

    // printf("Nearest free space: (%f, %f)\n",nearestLocation[0],nearestLocation[1]); // *comment above for it printing the same elements again and again at the end (for iteration in pathPlanner function)
}

void pathPlanner(){
    int p;
    double tempArray2[2];
    for (p=0; p<freeSpaceCounter; p++){
        tempArray2[0] = freeSpaces[alreadyFoundFreeSpace[i-1]][0]; // x of new start position
        tempArray2[1] = freeSpaces[alreadyFoundFreeSpace[i-1]][1]; // y of the new start position 
        findNearestFreeSpace(tempArray2, freeSpaces); // search from remaining free spaces the nearest to the new start position i.e freespace pointed by recent/last element of alreadyFoundFreeSpace array 
    }
}

void avoidObstacles(double startPosition[2], double freeSpacesPositions[176][2],double goalPosition[2]){
	// 1. find the nearest freespace from start
    // 2. move to that freespace(with safety margin 0.61)
    // 1.. repeat i.e. find the next freespace closest to the new start position
    // 2.. repeat i.e. move to that freespace(with safety margin 0.61)
    // 1,2.. repeat until you reach the last freespace that is closest to the new start position (means there are no other freespaces that is
    // closer to the current freespace than the goal itself)
    // 3. lastly move to the goal

    // 1.
    double tempArr[2];
    int j;
    for(j=0; j<i; j++){
        tempArr[0] =  freeSpacesPositions[alreadyFoundFreeSpace[j]][0];
        tempArr[1] =  freeSpacesPositions[alreadyFoundFreeSpace[j]][1];
        printf("\n%d: (%f, %f) - ",j+1,freeSpacesPositions[alreadyFoundFreeSpace[j]][0],freeSpacesPositions[alreadyFoundFreeSpace[j]][1]);
        
        // moveX1Y1toX2Y2(startPosition,tempArr);
        printf("move from (%f, %f) to (%f, %f)", startPosition[0]/0.305, startPosition[1]/0.305, tempArr[0]/0.305, tempArr[1]/0.305);
        startPosition[0]=tempArr[0];
        startPosition[1]=tempArr[1];
    }

    // moveX1Y1toX2Y2(startPosition, goal);
    printf("\n   Move to goal         - move from (%f, %f) to (%f, %f)\n", startPosition[0]/0.305, startPosition[1]/0.305, goal[0]/0.305, goal[1]/0.305);
}

int main(void)
{

    freeSpacesFinder(obstacle);
    // printFreeSpaces(freeSpaces);

    pathPlanner();

    // printf("\n");
    // int q;
    // for (q=0; q<i; q++){
    //     printf("already found freespace %d: (%f, %f)\n", q+1, freeSpaces[alreadyFoundFreeSpace[q]][0], freeSpaces[alreadyFoundFreeSpace[q]][1]);
    // }

    printf("\nPath: ");
    avoidObstacles(start,freeSpaces,goal);

    

	return 0;
}