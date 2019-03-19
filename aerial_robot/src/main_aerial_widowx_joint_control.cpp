#include <ros/ros.h>
#include "aerial_widowx_joint_control.h"

using namespace ros;

WidowXJointControl *node;

int main (int argc, char *argv[])
{
	init (argc, argv, "aerial_widowx_joint_control");

	node = new WidowXJointControl;

	return node->spin ();
}
