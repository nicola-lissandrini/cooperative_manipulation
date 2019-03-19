#include <ros/ros.h>
#include "mpc_wrapper.h"

using namespace ros;
using namespace std;

#define NODE_NAME "mpc_wrapper_test"

MPCWrapper *mpcWrapper;

int main (int argc, char *argv[])
{
	init (argc, argv, NODE_NAME);

	mpcWrapper = new MPCWrapper;

	return mpcWrapper->spin ();
}
