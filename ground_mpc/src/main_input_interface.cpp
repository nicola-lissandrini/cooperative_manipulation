#include "input_interface.h"

using namespace ros;

InputInterface *inputInterface;

int main (int argc, char *argv[])
{
	init (argc, argv, "input_interface");

	inputInterface = new InputInterface;

	return inputInterface->spin ();
}
