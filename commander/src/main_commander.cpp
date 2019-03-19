#include <ros/ros.h>
#include "../include/commander.h"

using namespace ros;
using namespace std;

Commander *commander;

int main (int argc, char *argv[])
{
	init (argc, argv, "commander");
	commander = new Commander;

	return commander->spin ();
}
