#include <ros/ros.h>
#include "obstacle_detector.h"

using namespace ros;
using namespace std;

ObstacleDetector *obstacleDetector;

int main (int argc, char *argv[])
{
	init (argc, argv, "obstacle_detector");

	obstacleDetector = new ObstacleDetector;

	return obstacleDetector->spin ();
}
