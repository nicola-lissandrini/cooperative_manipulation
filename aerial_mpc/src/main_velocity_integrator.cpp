#include "velocity_integrator.h"

using namespace ros;

VelocityIntegrator *velocityIntegrator;

int main (int argc, char *argv[])
{
	init (argc, argv, "velocity_integrator");

	velocityIntegrator = new VelocityIntegrator;

	return velocityIntegrator->spin ();
}
