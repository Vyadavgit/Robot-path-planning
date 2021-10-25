#include <stdio.h>
#include <math.h>

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


