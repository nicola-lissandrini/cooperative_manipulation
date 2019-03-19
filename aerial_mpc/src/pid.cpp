#include "pid.h"

#include <iostream>

using namespace Eigen;
using namespace std;

Pid::Pid (int _stateCount, double _sampleTime,
	 double _kp, double _ki,
	 double _kd):
	kp(_kp), ki(_ki), kd(_kd),
	sampleTime (_sampleTime),
	stateCount(_stateCount),
	integratorState(_stateCount),
	lastError(_stateCount)
{
	std::cout << "quqaaaa" << std::endl;
}

VectorXd Pid::getControl (const VectorXd &refVel, const VectorXd &state)
{

	VectorXd error = refVel - state;
	VectorXd intError = integratorState;
	VectorXd derivError = (error - lastError) / sampleTime;

	integratorState += error * sampleTime;
	lastError = error;

	return kp * error + ki * intError + kd * derivError;
}
