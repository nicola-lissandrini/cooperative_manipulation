#include "mpc_wrapper.h"

#include <acado/acado_gnuplot.hpp>
#include <acado/curve/curve.hpp>
#include <acado/acado_toolkit.hpp>
#include <fstream>

using namespace std;
using namespace ros;
using namespace Eigen;
using namespace XmlRpc;
using namespace mpc_wrapper; //for msg;

void MPCWrapper::initParams () {
	try {
		nh.getParam ("mpc_wrapper_config",params);
	} catch (XmlRpc::XmlRpcException e) {
		ROS_ERROR ("Error loading params: %s\n",e.getMessage ().c_str ());
	}
}

void MPCWrapper::initROS ()
{
	rate = new Rate (double (params["rate"]));

	inputSub = nh.subscribe ((string)params["input_topic"], 1, inputCallback);

	controlPub = nh.advertise<OutputDataMsg> ((string)params["control_topic"], 1);
}

MPCWrapper::MPCWrapper ()
{
	initParams ();
	initROS ();

	MPCError initError = mpc.init (params["mpc_config"]);

	if (initError == MPC_ERROR_ILLEGAL_STATUS)
		WARN_ILLEGAL_STATUS (mpc.statusInfo ());
}

void MPCWrapper::sendControl ()
{
	OutputDataMsg controlMsg;
	double *controlTrajData = mpcOutput.controlTraj.data ();
	double *stateTrajData = mpcOutput.stateTraj.data ();
	double *currentData = mpcOutput.controlTraj.col (mpcOutput.openLoopIndex).data ();

	// Debug write output
	ofstream outData;

	outData.open ("/home/nicola/ros_ws/outData", std::ofstream::app | std::ofstream::out);

	for (int i = 0; i < mpcOutput.stateTraj.cols (); i++) {
		outData << mpcOutput.stateTraj(0, i) << ", " <<  mpcOutput.stateTraj(1,i) << "\n";
	}

	outData.close ();
	// Debug write output
	outData.open ("/home/nicola/ros_ws/controlOut", std::ofstream::app | std::ofstream::out);

	//for (int i = 0; i < mpcOutput.stateTraj.cols (); i++) {
		outData << mpcOutput.controlTraj(0, 0)  << "\n";
	//}

	outData.close ();

	controlMsg.controlTrajectory.layout.dim.resize (2);
	controlMsg.controlTrajectory.layout.dim[0].label = "Controls";
	controlMsg.controlTrajectory.layout.dim[0].size = mpcOutput.controlTraj.rows ();
	controlMsg.controlTrajectory.layout.dim[0].stride = mpcOutput.controlTraj.rows () * mpcOutput.controlTraj.cols ();
	controlMsg.controlTrajectory.layout.dim[1].label = "Samples";
	controlMsg.controlTrajectory.layout.dim[1].size = mpcOutput.controlTraj.cols ();
	controlMsg.controlTrajectory.layout.dim[1].stride = mpcOutput.controlTraj.cols ();
	controlMsg.controlTrajectory.data = vector<double> (controlTrajData, controlTrajData + mpcOutput.controlTraj.size());

	controlMsg.currentControl.layout.dim.resize (1);
	controlMsg.currentControl.layout.dim[0].label = "Current control";
	controlMsg.currentControl.layout.dim[0].size = mpcOutput.controlTraj.rows ();
	controlMsg.currentControl.layout.dim[0].stride = mpcOutput.controlTraj.rows ();
	controlMsg.currentControl.data = vector<double> (currentData, currentData + mpcOutput.controlTraj.rows ());

	controlMsg.stateTrajectory.layout.dim.resize (2);
	controlMsg.stateTrajectory.layout.dim[0].label = "States";
	controlMsg.stateTrajectory.layout.dim[0].size = mpcOutput.stateTraj.rows ();
	controlMsg.stateTrajectory.layout.dim[0].stride = mpcOutput.stateTraj.rows () * mpcOutput.stateTraj.cols ();
	controlMsg.stateTrajectory.layout.dim[1].label = "Samples";
	controlMsg.stateTrajectory.layout.dim[1].size = mpcOutput.stateTraj.cols ();
	controlMsg.stateTrajectory.layout.dim[1].stride = mpcOutput.stateTraj.cols ();
	controlMsg.stateTrajectory.data = vector<double> (stateTrajData, stateTrajData + mpcOutput.stateTraj.size());

	controlMsg.openLoopIndex = mpcOutput.openLoopIndex;
	controlMsg.kktTol = mpcOutput.kktTol;
	controlMsg.solverStatus = mpcOutput.solverStatus;
	controlMsg.cpuTime = mpcOutput.cpuTime;
	controlMsg.nIter = mpcOutput.nIter;
	controlMsg.objVal = mpcOutput.objVal;

	controlPub.publish (controlMsg);
}

void MPCWrapper::checkForSolverErrors (int err)
{
	// From qpOASES manual
	switch (err)
	{
	case 0:
		// QP was solved
		break;
	case 1:
		ROS_WARN ("QP could not be solved within the given number of iterations.");
		break;
	case -1:
		ROS_WARN ("QP could not be solved due to an internal error.");
		break;
	case -2:
		ROS_WARN ("QP is infeasible and thus could not be solved.");
		break;
	case -3:
		ROS_WARN ("QP is unbounded and thus could not be solved.");
		break;
	case -30:
		ROS_WARN ("QP causes nan KKT");
		break;
	default:
		ROS_WARN ("Unexpected QP error %d, kkt %lg", err, mpcOutput.kktTol);
		break;
	}
}

int MPCWrapper::spin ()
{
	int counter = 0;

	while (ok()) {
		if (mpc.isSpinning ()) {
			counter++;
#if SYNC == 1
			// Update every step only in synchronous mode
			mpc.updateInput (currentInputData);
#endif
			// Trigger an MPC computation step;
			MPCError tickError = mpc.tick ();

			if (tickError == MPC_ERROR_ILLEGAL_STATUS) {
				WARN_ILLEGAL_STATUS (mpc.statusInfo ());
			} else {
				// Get computed data
				mpcOutput = mpc.getMPCOutput ();

				checkForSolverErrors (mpcOutput.solverStatus);

				// Publish mpc output
				sendControl ();
			}
		}

		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

void MPCWrapper::updateInputData (const MPC::InputData &inputData)
{
	// If not started yet, initialize MPC
	if (mpc.statusInfo () == MPC_INITIALIZED) {
		mpc.initInput (inputData);
		currentInputData = inputData;

		// Automatically start when first data is available
		mpc.start ();
		ROS_INFO ("MPC Started");
	} else {
#if SYNC == 1
		// Latch input data for now, to avoid sync problems
		currentInputData = inputData;
#elif SYNC == 0
		// If not synced the MPC can run open loop
		// Then send it directly to the MPC when available
		mpc.updateInput (inputData);
#endif
	}
}

void MPCWrapper::inputCallback(const InputDataMsg &inputDataMsg)
{
	MPC::InputData inputData;
	vector<double> stateData = inputDataMsg.state.data;
	vector<double> refWindowData = inputDataMsg.refWindow.data;
	vector<double> refTerminalData = inputDataMsg.refTerminal.data;
	vector<double> onlineData = inputDataMsg.onlineData.data;

	if (inputDataMsg.state.layout.dim.size () != 1 ||
			inputDataMsg.refWindow.layout.dim.size () != 2 ||
			inputDataMsg.refTerminal.layout.dim.size () != 1 ||
			inputDataMsg.onlineData.layout.dim.size () != 2) {
		ROS_WARN ("Received invalid input message. Skipping.");
		return;
	}

	if (inputData.state.rows () != inputDataMsg.state.layout.dim[0].size ||
			inputData.state.rows () * inputData.state.cols () != inputDataMsg.state.data.size ()) {
		ROS_WARN("Input message state size mismatch. Skipping");
		return;
	}

	inputData.state << Map<MatrixXd> (stateData.data (), inputDataMsg.state.layout.dim[0].size, 1);

	if (inputData.refWindow.rows () != inputDataMsg.refWindow.layout.dim[0].size ||
			inputData.refWindow.cols () != inputDataMsg.refWindow.layout.dim[1].size ||
			inputData.refWindow.rows () * inputData.refWindow.cols () != inputDataMsg.refWindow.data.size ()) {
		ROS_WARN("Input message window reference size mismatch. Skipping");
		return;
	}

	inputData.refWindow << Map<MatrixXd> (refWindowData.data (), inputDataMsg.refWindow.layout.dim[0].size,
			inputDataMsg.refWindow.layout.dim[1].size);


	if (inputData.refTerminal.rows () != inputDataMsg.refTerminal.layout.dim[0].size ||
			inputData.refTerminal.rows ()  != inputDataMsg.refTerminal.data.size ()) {
		ROS_WARN("Input message terminal reference size mismatch. Skipping");
		return;
	}


	inputData.refTerminal << Map<MatrixXd> (refTerminalData.data (), inputDataMsg.refTerminal.layout.dim[0].size, 1);

	if (inputData.onlineData.rows () != inputDataMsg.onlineData.layout.dim[0].size ||
			inputData.onlineData.cols () != inputDataMsg.onlineData.layout.dim[1].size ||
			inputData.onlineData.rows () * inputData.onlineData.cols () != inputDataMsg.onlineData.data.size ()) {
		ROS_WARN("Input message onlineData size mismatch. Skipping");
		return;
	}

	inputData.onlineData << Map<MatrixXd> (onlineData.data (), inputDataMsg.onlineData.layout.dim[0].size,
			inputDataMsg.onlineData.layout.dim[1].size);

	mpcWrapper->updateInputData (inputData);
}








