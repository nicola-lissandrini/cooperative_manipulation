#include "output_interface.h"

using namespace ros;

OutputInterface *outputInterface;

int main (int argc, char *argv[])
{
	init (argc, argv, "output_interface");

	outputInterface = new OutputInterface;

	return outputInterface->spin ();
}
