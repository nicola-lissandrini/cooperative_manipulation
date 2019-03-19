#ifndef MPC_H
#define MPC_H

#include <XmlRpc.h>
#include <eigen3/Eigen/Dense>
#include <ros/assert.h>

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#define QUA ROS_INFO("Reached %d:%s", __LINE__, __FILE__);

enum MPCStatus {
	MPC_NOT_READY = -1,
	MPC_INITIALIZED,
	MPC_READY,
	MPC_SPINNING_NEW,
	MPC_SPINNING_OLD,
	MPC_STOPPED,
	MPC_DEAD
};

extern const char *mpcStatusStr[7];

#define GET_MPC_STATUS_STR(s) (mpcStatusStr[s-1])

enum MPCError {
	MPC_ERROR_NONE = 0,
	MPC_ERROR_SIZE_MISMATCH,
	MPC_ERROR_ILLEGAL_STATUS
};

typedef Eigen::VectorXd DVector;
typedef Eigen::MatrixXd DMatrix;

class MPC
{
	MPCStatus status;
	int openLoopIndex;

	struct Params {
		DVector xInit;
		DVector zInit;
		DVector uInit;
		DVector bValues;
		DMatrix w;
		DMatrix wN;

		int nIter;
	} params;


	inline bool checkSizes (const DMatrix &a, const DMatrix &b);
	void solve ();
	void solverGenerateOutput (int &solverStatus, int &sumIter, double cpuTime);
	void solverCompute (int &solverStatus, int &sumIter, double &kkt);
	void solverSetVars ();
	void initSetVars ();
	bool predictedControlAvailable ();

	static DVector vectorFromParam (XmlRpc::XmlRpcValue &param);
	static DMatrix matrixFromParam (XmlRpc::XmlRpcValue &param);
	static void repcol2acado (double *mat, const DVector &col, int rows, int cols);
	static void mat2acado (double *dest, const DMatrix &src, int rows, int cols);
	static void acado2mat (DMatrix &dest, double *src, int rows, int cols);
	static void acadocpy (double *dest, double *src, int size);

public:
	struct InputData {
		DMatrix state;
		DMatrix refWindow;
		DMatrix refTerminal;
		DMatrix onlineData;

		InputData ():
			state(ACADO_NX, 1),
			refWindow(ACADO_NY, ACADO_N),
			refTerminal(ACADO_NYN, 1),
			onlineData(ACADO_NOD, ACADO_N + 1)
		{}

		InputData (const InputData &_cpy):
			state(_cpy.state),
			refWindow(_cpy.refWindow),
			refTerminal(_cpy.refTerminal),
			onlineData(_cpy.onlineData)
		{}
	};

	struct OutputData {
		DMatrix controlTraj;
		DMatrix stateTraj;
		int openLoopIndex;
		double kktTol;
		double solverStatus;
		double cpuTime;
		int nIter;
		double objVal;

		inline DVector getControl () {
			return controlTraj.col(openLoopIndex);
		}

		inline bool isOpenLoop () {
			return openLoopIndex > 0;
		}

		OutputData ():
			stateTraj(ACADO_NX, ACADO_N + 1),
			controlTraj(ACADO_NU, ACADO_N)
		{}
	};

	MPC ();

	MPCError init (XmlRpc::XmlRpcValue &_yamlParams);
	MPCError start ();
	MPCError tick ();

	MPCError updateInput (const InputData &newInputData);
	MPCError initInput (const InputData &newInputData);
	const OutputData &getMPCOutput ();

	MPCStatus statusInfo ();
	bool isSpinning ();

	int windowLength ();
	int stateCount ();
	int referenceCount ();
	int controlCount ();

private:
	InputData inputData;
	OutputData outputData;

	MPCError setInput (const InputData &newInputData);
};

#endif // MPC_H
