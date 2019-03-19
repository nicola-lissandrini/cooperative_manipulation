#include "mpc.h"

using namespace XmlRpc;
using namespace Eigen;
#include <iostream> // for debug
using namespace std;
#include <stdio.h>

#define STATUS_CHECK(s) {if (status != (s)) return MPC_ERROR_STATUS}

#define VARYING_BOUNDS 0
#define RESET_WHEN_NAN 1   // no reset ( = 0 ), reset with previous traj ( = 1 )
#define KKT_MIN -1
#define KKT_MAX 1e15

#include "acado_auxiliary_functions.h"

void dumpAcado ();

const char *mpcStatusStr[7] = {
	"MPC_NOT_READY",
	"MPC_INITIALIZED",
	"MPC_READY",
	"MPC_SPINNING_NEW",
	"MPC_SPINNING_OLD",
	"MPC_STOPPED",
	"MPC_DEAD"};

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

MPC::MPC ():
	status(MPC_NOT_READY),
	openLoopIndex(0)
{
}

bool MPC::checkSizes (const DMatrix &a, const DMatrix &b) {
	return (a.cols() == b.cols()) || (a.rows () == b.rows ());
}

DVector MPC::vectorFromParam (XmlRpcValue &param)
{
	DVector vec;

	ROS_ASSERT (param.getType () == XmlRpcValue::TypeArray);

	vec.resize (param.size ());

	for (int i = 0; i < param.size (); i++) {
		ROS_ASSERT (param[i].getType () == XmlRpcValue::TypeDouble);

		vec(i) = (double) param[i];
	}

	return vec;
}

// yaml representation must be line by line
DMatrix MPC::matrixFromParam (XmlRpcValue &param)
{
	DMatrix mat;

	ROS_ASSERT (param.getType () == XmlRpcValue::TypeArray);

	mat.resize (param.size (), 1);

	for (int i = 0; i < param.size (); i++) {
		XmlRpcValue paramRow = param[i];
		ROS_ASSERT (paramRow.getType () == XmlRpcValue::TypeArray);

		if (i == 0)
			mat.resize (NoChange, paramRow.size ());
		else
			ROS_ASSERT (mat.cols() == paramRow.size ());

		for (int j = 0; j < mat.cols (); j++)
			mat(i,j) = (double) paramRow[j];
	}

	return mat;
}

MPCError MPC::init (XmlRpcValue &yamlParams)
{
	if (status != MPC_NOT_READY)
		return MPC_ERROR_ILLEGAL_STATUS;

	// Initial state
	params.xInit = vectorFromParam (yamlParams["x_init"]);
	// Initial algebraic state - if any
	params.zInit = vectorFromParam (yamlParams["z_init"]);
	// Initial control input
	params.uInit = vectorFromParam (yamlParams["u_init"]);
	// Running cost matrix
	DVector diagW = vectorFromParam (yamlParams["W"]);
	params.w = diagW.asDiagonal ();
	// Terminal cost matrix
	DVector diagWN = vectorFromParam (yamlParams["WN"]);
	params.wN = diagWN.asDiagonal ();
	// Variable boundary matrix - if any
	params.bValues = vectorFromParam (yamlParams["b_values"]);
	// Solver iterations
	params.nIter = (int) yamlParams["n_iter"];

	status = MPC_INITIALIZED;
	ROS_INFO ("MPC Initialized");

	return MPC_ERROR_NONE;
}

void MPC::repcol2acado (double *mat, const DVector &col, int rows, int cols)
{
	int i, j;

	assert (rows == col.size());

	for (i = 0; i < cols; i++) {
		for (j = 0; j < rows; ++j)
			mat[i*rows+j] = col(j);
	}
}

void MPC::mat2acado (double *dest, const DMatrix &src, int rows, int cols)
{
	int i, j;

	assert ((rows == src.rows()) && (cols <= src.cols ()));

	for (i = 0; i < cols; i++) {
		for (j = 0; j < rows; j++)
			dest[i*rows + j] = src(j, i);
	}
}

void MPC::acado2mat (DMatrix &dest, double *src, int rows, int cols)
{
	int i, j;

	assert ((rows == dest.rows()) && (cols == dest.cols ()));

	for (i = 0; i < cols; i++) {
		for (j = 0; j < rows; j++)
			dest(j, i) = src[i*rows + j];
	}
}

void MPC::acadocpy (double *dest, double *src, int size) {
	memcpy (dest, src, size);
}

void MPC::initSetVars ()
{
	if (params.xInit.size () != ACADO_NX) {
		ROS_FATAL ("Wrong xInit size in params. Aborting.");
		exit (-1);
	}

	if (params.uInit.size () != ACADO_NU)  {
		ROS_FATAL ("Wrong uInit size in params. Aborting.");
		exit (-1);
	}

	if (params.w.rows () != ACADO_NY &&
			params.w.cols () != ACADO_NY)  {
		ROS_FATAL ("Wrong running cost matrix size in params. Aborting.");
		exit (-1);
	}

	if (params.wN.rows () != ACADO_NYN &&
			params.wN.cols () != ACADO_NYN)  {
		ROS_FATAL ("Wrong terminal cost matrix size in params. Aborting.");
		exit (-1);
	}

	// Init state
	repcol2acado (acadoVariables.x, params.xInit, ACADO_NX, ACADO_N + 1);
	// Init control input
	repcol2acado (acadoVariables.u, params.uInit, ACADO_NU, ACADO_N);
	// Init algebraic state - if any
#if ACADO_NXA > 0
	repcol2acado (acadoVariables.z, params.zInit, ACADO_NXA, ACADO_N);
#endif
	// Init output reference
	mat2acado (acadoVariables.y, inputData.refWindow, ACADO_NY, ACADO_N);
	// Init terminal reference
	mat2acado (acadoVariables.yN, inputData.refTerminal, ACADO_NYN, 1);
	// Init online data
	mat2acado (acadoVariables.od, inputData.onlineData, ACADO_NOD, ACADO_N + 1);
	// Init running cost
	mat2acado (acadoVariables.W, params.w, ACADO_NY, ACADO_NY);
	// Init terminal cost
	mat2acado (acadoVariables.WN, params.wN, ACADO_NYN, ACADO_NYN);

	// TODO: implement VARYING_BOUNDS
}

bool MPC::predictedControlAvailable() {
	return openLoopIndex < ACADO_N;
}

MPCError MPC::start ()
{
	if (status != MPC_READY)
		return MPC_ERROR_ILLEGAL_STATUS;

	initSetVars ();

	acado_initializeSolver ();
	//acado_initializeNodesByForwardSimulation ();
	acado_preparationStep ();

	// Random init
	for (int i = 0; i < 10; i++) {
		tick ();
	}

	status = MPC_SPINNING_NEW;
	return MPC_ERROR_NONE;
}

MPCError MPC::setInput (const InputData &newInputData)
{
	if (!checkSizes (newInputData.state, inputData.state))
		return MPC_ERROR_SIZE_MISMATCH;

	if (!checkSizes (newInputData.refWindow, inputData.refWindow))
		return MPC_ERROR_SIZE_MISMATCH;

	if (!checkSizes (newInputData.onlineData, inputData.onlineData))
		return MPC_ERROR_SIZE_MISMATCH;

	// Update data
	inputData = newInputData;

	// New data received: reset openLoopCounter
	openLoopIndex = 0;
}

MPCError MPC::updateInput (const InputData &newInputData)
{
	if (status != MPC_SPINNING_NEW &&
			status != MPC_SPINNING_OLD)
		return MPC_ERROR_ILLEGAL_STATUS;

	setInput (newInputData);

	// Shift the state and control trajectories:
	acado_shiftStates (0, NULL, NULL);
	acado_shiftControls (NULL);

	// ACADO reset step
	acado_preparationStep ();

	status = MPC_SPINNING_NEW;

	return MPC_ERROR_NONE;
}

MPCError MPC::initInput (const InputData &newInputData)
{
	if (status != MPC_INITIALIZED)
		return MPC_ERROR_ILLEGAL_STATUS;

	setInput (newInputData);

	status = MPC_READY;

	return MPC_ERROR_NONE;
}

MPCError MPC::tick ()
{
	switch (status)
	{
	case MPC_SPINNING_NEW:
		solve ();

		status = MPC_SPINNING_OLD;
		return MPC_ERROR_NONE;
	case MPC_SPINNING_OLD:
		QUA;
		if (predictedControlAvailable ())
			openLoopIndex++;
		else {
			status = MPC_DEAD;

			return MPC_ERROR_NONE;
		}
	default:
		return MPC_ERROR_ILLEGAL_STATUS;
	}
}

void MPC::solverSetVars ()
{
	// Acado init x0
	mat2acado (acadoVariables.x0, inputData.state, ACADO_NX, 1);
	// Acado init reference output
	mat2acado (acadoVariables.y, inputData.refWindow, ACADO_NY, ACADO_N);
	// Acado init terminal reference output
	mat2acado (acadoVariables.yN, inputData.refTerminal, ACADO_NYN, 1);
	// Online data
	mat2acado (acadoVariables.od, inputData.onlineData, ACADO_NOD, ACADO_N + 1);
	// Acado init running cost -- fixed for now
	mat2acado (acadoVariables.W, params.w, ACADO_NY, ACADO_NY);
	// Init terminal cost -- fixed for now
	mat2acado (acadoVariables.WN, params.wN, ACADO_NYN, ACADO_NYN);

	// TODO: varying bounds

}

void MPC::solverCompute (int &solverStatus, int &sumIter, double &kkt)
{
	// Compute first feedback step
	solverStatus = acado_feedbackStep ();
	sumIter = (int) acado_getNWSR ();


	// Perform subsequent iterations
	// Possible TODO: iterate until desired KKT
	for (int i = 1; i < params.nIter; i++) {
		acado_preparationStep ();
		solverStatus = acado_feedbackStep ();
		sumIter += (int) acado_getNWSR ();
	}
}

void MPC::solverGenerateOutput (int &solverStatus, int &sumIter, double cpuTime)
{
	acado2mat (outputData.stateTraj, acadoVariables.x, ACADO_NX, ACADO_N + 1);
	acado2mat (outputData.controlTraj, acadoVariables.u, ACADO_NU, ACADO_N);

	outputData.openLoopIndex = 0;
	outputData.kktTol = acado_getKKT ();
	outputData.solverStatus = solverStatus;
	outputData.cpuTime = cpuTime;
	outputData.nIter = sumIter;
	outputData.objVal = acado_getObjective ();
}

void MPC::solve ()
{
#if RESET_WHEN_NAN
	double x_prev[(ACADO_N+1)*ACADO_NX];
# if ACADO_NXA >0
	double z_prev[ACADO_N*ACADO_NXA];
# endif // ACADO_NXA
	double u_prev[ACADO_N*ACADO_NU];
#endif
	// Start solver timer
	acado_timer timer;

	acado_tic (&timer);

	// Initialize ACADO variables for current step
	solverSetVars ();

#if RESET_WHEN_NAN
	acadocpy (x_prev, acadoVariables.x, (ACADO_N+1)*ACADO_NX);
# if ACADO_NXA > 0
	acadocpy (z_prev, acadoVariables.z, ACADO_N * ACADO_NXA);
# endif // ACADO_NXA
	acadocpy (u_prev, acadoVariables.u, ACADO_N*ACADO_NU);
#endif // RESET_WHEN_NAN

	int solverStatus, sumIter;
	double kkt;

	// Compute MPC step
	solverCompute (solverStatus, sumIter, kkt);

#if RESET_WHEN_NAN
	kkt = acado_getKKT ();
	if (kkt < KKT_MIN || kkt > KKT_MAX || kkt != kkt) {
		acadocpy (acadoVariables.x, x_prev, (ACADO_N+1)*ACADO_NX);
# if ACADO_NXA > 0
		acadocpy (z_prev, acadoVariables.z, ACADO_N * ACADO_NXA);
# endif // ACADO_NXA
		acadocpy (acadoVariables.u, u_prev, ACADO_N*ACADO_NU);

		solverStatus = -30;
	}
#endif // RESET_WHEN_NAN

	double cpuTime = acado_toc (&timer);

	// Generate output structure
	solverGenerateOutput (solverStatus, sumIter, cpuTime);
}

bool MPC::isSpinning () {
	return (status == MPC_SPINNING_NEW) || (status == MPC_SPINNING_OLD);
}

const MPC::OutputData &MPC::getMPCOutput () {
	return outputData;
}

MPCStatus MPC::statusInfo() {
	return status;
}

int MPC::windowLength () {
	return ACADO_N;
}

int MPC::stateCount () {
	return ACADO_NX;
}

int MPC::referenceCount () {
	return ACADO_NY;
}

int MPC::controlCount () {
	return ACADO_NU;
}




void dumpAcado ()
{
	int i;
	printf ("ACADO Variables:\n");

	printf ("   x\n");
	for (i = 0; i < (ACADO_N +1)*ACADO_NX; i++) {
		printf ("   %3.3lg\n", acadoVariables.x[i]);
	}

	printf ("   x0\n");
	for (i = 0; i < ACADO_NX; i++) {
		printf ("   %3.3lg\n", acadoVariables.x0[i]);
	}

	printf ("   y\n");
	for (i = 0; i < (ACADO_N)*ACADO_NY; i++) {
		printf ("   %3.3lg\n", acadoVariables.y[i]);
	}

	printf ("   yN\n");
	for (i = 0; i < ACADO_NYN; i++) {
		printf ("   %3.3lg\n", acadoVariables.yN[i]);
	}

	printf ("   u\n");
	for (i = 0; i < (ACADO_N)*ACADO_NU; i++) {
		printf ("   %3.3lg\n", acadoVariables.u[i]);
	}

	printf ("   W\n");
	for (i = 0; i < (ACADO_NY)*ACADO_NY; i++) {
		printf ("   %3.3lg\n", acadoVariables.W[i]);
	}

	printf ("   WN\n");
	for (i = 0; i < (ACADO_NYN)*ACADO_NYN; i++) {
		printf ("   %3.3lg\n", acadoVariables.WN[i]);
	}

	printf ("   od\n");
	for (i = 0; i < (ACADO_NY)*ACADO_NY; i++) {
		printf ("   %3.3lg\n", acadoVariables.od[i]);
	}

	printf ("------ DUMP END ------\n");
}








