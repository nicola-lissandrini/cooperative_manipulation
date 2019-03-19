#ifndef PID_H
#define PID_H

#include <eigen3/Eigen/Dense>

class Pid
{
	 double ki, kp, kd;
	 double sampleTime;
	 int stateCount;
	 Eigen::VectorXd integratorState;
	 Eigen::VectorXd lastError;

public:
	Pid (int _stateCount,
		 double _sampleTime,
		 double _kp,
		 double _ki,
		 double _kd);

	Eigen::VectorXd getControl (const Eigen::VectorXd &refVel, const Eigen::VectorXd &state);
};

#endif // PID_H
