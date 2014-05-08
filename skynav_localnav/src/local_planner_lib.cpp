
#include <math.h>
#include "local_planner_lib.h"

using namespace geometry_msgs;

//calculate distance between two point with use of pythagoras
double calcDistance(Point a, Point b)
{
	return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
}

