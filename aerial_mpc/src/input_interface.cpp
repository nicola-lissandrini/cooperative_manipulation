#include "input_interface.h"
#include <sensor_msgs/JointState.h>
#include "mpc.h"


using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace mpc_wrapper;
using namespace tf;

InputInterface::InputInterface ():
	started(false),
	mode(MODE_CONSTANT)
{
	initParams ();
	initROS ();
	FILE *out = fopen ("/home/nicola/stateLogAerial","w");
	fclose (out);
}

void InputInterface::initParams ()
{
	try {
		nh.getParam ("mpc_input_interface", params);
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s\n",e.getMessage ().c_str ());
	}
}

void InputInterface::initROS ()
{
	rate = new Rate (double (params["rate"]));
	
	baseSub = nh.subscribe ((string) params["base_state_topic"], 1, baseCallback);
	jointsSub = nh.subscribe ((string) params["joints_state_topic"], 1, jointsCallback);
	linksSub = nh.subscribe ((string) params["end_effector_topic"], 1, eefOdometryCallback);
	refSub = nh.subscribe ((string) params["reference_topic"], 1, referenceCallback);
	trajSub = nh.subscribe ((string) params["trajectory_topic"], 1, trajectoryCallback);
	obstSub = nh.subscribe ((string) params["obstacle_topic"], 1, obstacleCallback);
	
	inputPub = nh.advertise<InputDataMsg> ((string) params["mpc_input_topic"], 1);
}

void InputInterface::baseCallback (const nav_msgs::Odometry &baseOdometry)
{
	Quaternion baseQuat;
	Matrix3x3 rotm;
	Vector basePose(BASE_PO_N);
	double r, p, y;
	
	quaternionMsgToTF (baseOdometry.pose.pose.orientation, baseQuat);
	rotm.setRotation (baseQuat.normalized ());
	rotm.getEulerYPR (y, p, r);
	
	//ROS_INFO ("CUrr z %lg", baseOdometry.pose.pose.position.z);
	
	basePose[0] = baseOdometry.pose.pose.position.x;
	basePose[1] = baseOdometry.pose.pose.position.y;
	basePose[2] = baseOdometry.pose.pose.position.z;
	basePose[3] = y;
	
	inputInterface->setRobotsBasePos (basePose);
}

void InputInterface::jointsCallback (const std_msgs::Float32MultiArray &jointState)
{
	vector<double> jointsVals(2);
	jointsVals[0] = jointState.data[0];
	jointsVals[1] = jointState.data[1];

	inputInterface->setRobotJoints (jointsVals);
}

void InputInterface::eefOdometryCallback (const nav_msgs::Odometry &eefOdometry)
{
	Quaternion eefQuat;
	Matrix3x3 eefRotm;
	Vector eefPos(EEF_P_N), eefRot(EEF_O_N);
	
	geometry_msgs::Pose eefPose = eefOdometry.pose.pose;
	quaternionMsgToTF (eefPose.orientation, eefQuat);
	eefRotm.setRotation (eefQuat);
	
	eefPos[0] = eefPose.position.x;
	eefPos[1] = eefPose.position.y;
	eefPos[2] = eefPose.position.z;

	for (int i = 0; i < 3; i++) {
		eefRot[i*3] = eefRotm.getColumn (i).x ();
		eefRot[i*3 + 1] = eefRotm.getColumn (i).y ();
		eefRot[i*3 + 2] = eefRotm.getColumn (i).z ();
	}

	
	inputInterface->setRobotEef (eefPos, eefRot);
}

void InputInterface::obstacleCallback (const ground_mpc::ObstaclesArray &obstacle)
{
	Vector obstData(OBST_SZ * obstacle.obstacles.size ());
	
	for (int i = 0; i < obstacle.obstacles.size(); i++) {
		obstData[i*OBST_SZ + 0] = obstacle.obstacles[i].center.x;
		obstData[i*OBST_SZ + 1] = obstacle.obstacles[i].center.y;
		obstData[i*OBST_SZ + 2] = obstacle.obstacles[i].center.z;
		obstData[i*OBST_SZ + 3] = obstacle.obstacles[i].axes.x;
		obstData[i*OBST_SZ + 4] = obstacle.obstacles[i].axes.y;
		obstData[i*OBST_SZ + 5] = obstacle.obstacles[i].axes.z;
	}
	
	inputInterface->setObstacles (obstData);
}

Vector InputInterface::getStackedState ()
{
	Vector stacked;
	int global = 0;
	int i;
	stacked.resize (EEF_P_N + EEF_O_N + JOINT_N + BASE_PO_N);

	for (i = 0; i < EEF_P_N; i++)
		stacked[i] = robotState.eefPos[i];
	global += i;
	for (i = 0; i < EEF_O_N; i++) {
		stacked[global + i] = robotState.eefRot[i];
	}
	global += i;
	
	for (i = 0; i < JOINT_N; i++) {
		stacked[global + i] = robotState.jointsVals[i];
	}
	global += i;
	
	for (i = 0; i < BASE_PO_N; i++) {
		stacked[global + i] = robotState.basePose[i];
	}
	global += i;


	FILE *out = fopen ("/home/nicola/stateLogAerial","a");
	fprintf (out, "%lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg\n",robotState.eefPos[0],
			robotState.eefPos[1],robotState.eefPos[2],  robotState.jointsVals[0],
			robotState.jointsVals[1],
			 robotState.basePose[0],  robotState.basePose[1],  robotState.basePose[2], robotState.basePose[3]);
	fclose (out);

	return stacked;
}

Vector InputInterface::getMPCRefConstant ()
{
	Vector mpcRef;
	Vector term = getMPCRefTermConstant ();
	int i;
	
	mpcRef.resize (term.size () + CONTROLS);
	
	for (i = 0; i < term.size (); i++)
		mpcRef[i] = term[i];
	for (; i < mpcRef.size (); i++)
		mpcRef[i] = 0;

	return mpcRef;
}


Vector InputInterface::getMPCRefTermConstant ()
{
	Vector mpcRef;
	int i;
	
	mpcRef.resize (EEF_REF_TERM);
	
	for (i = 0; i < EEF_P_N; i++)
		mpcRef[i] = robotReference.eefPosRefConstant[i];
	for (; i < EEF_REF_TERM; i++)
		mpcRef[i] = 0;

	return mpcRef;
}

Vector InputInterface::getOnlineDataConstant ()
{
	const int eefRotSize = robotReference.eefRotRefConstant.size ();
	const int obstSize = robotReference.obstacles.size ();
	Vector stackedOnlineData(eefRotSize);

	for (int i = 0; i < eefRotSize; i++)
		stackedOnlineData[i] = robotReference.eefRotRefConstant[i];
	/*for (int i = 0; i < obstSize; i++)
		stackedOnlineData[eefRotSize + i] = robotReference.obstacles[i];*/

	return stackedOnlineData;
}

Vector InputInterface::getMPCRefTrajectory ()
{
	Vector stackedPosTraj(EEF_P_TRAJ_N);

	for (int i = 0; i < robotReference.eefPosRefTrajectory.size () - 1; i++) {
		int j;

		for (j = 0; j < EEF_P_N; j++)
			stackedPosTraj[i * EEF_REF + j] = robotReference.eefPosRefTrajectory[i][j];
		for (; j < EEF_REF; j++)
			stackedPosTraj[i * EEF_REF + j] = 0; // controls and others
	}

	return stackedPosTraj;
}

Vector InputInterface::getMPCRefTermTrajectory ()
{
	Vector refTerm(EEF_REF_TERM);
	int j;

	for (j = 0; j < EEF_P_N; j++)
		refTerm[j] = robotReference.eefPosRefTrajectory[WINDOW_N][j];
	for (; j < EEF_REF_TERM; j++)
		refTerm[j] = 0;

	return refTerm;
}

Vector InputInterface::getOnlineDataTrajectory ()
{
	Vector stackedOnline(EEF_O_TRAJ_N);

	for (int i = 0; i < WINDOW_N + 1; i++) {
		for (int j = 0; j < EEF_O_N; j++) {
			stackedOnline[i * EEF_O_N + j] = robotReference.eefRotRefTrajectory[i][j];
		}
	}

	return stackedOnline;
}

void InputInterface::checkCollisions ()
{
	double x = robotState.eefPos[0] - robotReference.obstacles[0];
	double y = robotState.eefPos[1] - robotReference.obstacles[1];
	double z = robotState.eefPos[2] - robotReference.obstacles[2];
	double dist = x*x + y*y + z*z;
	
	if (dist < robotReference.obstacles[4])
		ROS_WARN ("End effector collision detected");
	
}

int InputInterface::spin ()
{
	while (ok ()) {
		if (started)
			sendInput ();
		checkCollisions ();
		spinOnce ();
		
		rate->sleep ();
	}
	
	return 0;
}

template<typename T>
vector<T> vectorFromParam (XmlRpcValue &param)
{
	vector<T> vec;

	if (param.getType () != XmlRpcValue::TypeArray) {
		ROS_ERROR ("Error parsing array. Type %d found", param.getType ());
	}

	vec.resize (param.size ());

	for (int i = 0; i < param.size (); i++) {
		vec[i] = (T) param[i];
	}

	return vec;
}

void InputInterface::setRobotJoints(const Vector &jointsVals)
{
	vector<int> jointSigns = vectorFromParam<int> (params["joint_signs"]);

	for (int i = 0; i < jointsVals.size (); i++)
		robotState.jointsVals[i] = jointsVals[i] * double (jointSigns[i]);
}

void InputInterface::setRobotRefConstant(const Vector &eefPosRef, const Vector &eefRotRef)
{
	mode = MODE_CONSTANT;
	robotReference.eefPosRefConstant = eefPosRef;
	robotReference.eefRotRefConstant = eefRotRef;
}

void InputInterface::setRobotRefTrajectory(const vector<Vector> &eefPosRef, const vector<Vector> &eefRotRef)
{
	mode = MODE_TRAJECTORY;
	robotReference.eefPosRefTrajectory = eefPosRef;
	robotReference.eefRotRefTrajectory = eefRotRef;
}

Vector positionToVector (const geometry_msgs::Pose &pose)
{
	Vector posRef(EEF_P_N);

	posRef[0] = pose.position.x;
	posRef[1] = pose.position.y;
	posRef[2] = pose.position.z;

	return posRef;
}

Vector rotationMatrixToVector (const geometry_msgs::Pose &pose)
{
	Quaternion quat;
	Matrix3x3 rotm;
	Vector rotRef(EEF_O_N);

	quaternionMsgToTF (pose.orientation, quat);
	rotm.setRotation (quat);

	for (int i = 0; i < 3; i++) {
		rotRef[i*3] = rotm.getColumn (i).x ();
		rotRef[i*3 + 1] = rotm.getColumn (i).y ();
		rotRef[i*3 + 2] = rotm.getColumn (i).z ();
	}

	return rotRef;
}

void InputInterface::referenceCallback (const geometry_msgs::Pose &ref)
{
	Vector eefPosRef(EEF_P_N, 0);
	Vector eefRotRef(EEF_O_N, 0);
	
	if (!inputInterface->isStarted ())
		inputInterface->start ();

	eefPosRef = positionToVector (ref);

#ifdef BASE_ONLY
	eefRotRef[0] = ref.orientation.x;
#else
	eefRotRef = rotationMatrixToVector (ref);
#endif
	
	inputInterface->setRobotRefConstant (eefPosRef, eefRotRef);
}

void InputInterface::trajectoryCallback (const aerial_mpc::Trajectory &ref)
{
	vector<Vector> posTrajectory(WINDOW_N + 1);
	vector<Vector> rotTrajectory(WINDOW_N + 1);
	Vector currPos(EEF_P_N);
	Vector currRot(EEF_O_N);

	if (ref.trajectory.size () != WINDOW_N + 1) {
		ROS_WARN ("Aerial input interface: invalid trajectory length. Skipping");
		return;
	}

	for (int i = 0; i < WINDOW_N + 1; i++) {
		currPos = positionToVector (ref.trajectory[i]);
		currRot = rotationMatrixToVector (ref.trajectory[i]);

		posTrajectory[i].resize ( EEF_O_N);
		posTrajectory[i] = currPos;

		rotTrajectory[i].resize (EEF_P_N);
		rotTrajectory[i] = currRot;
	}

	inputInterface->setRobotRefTrajectory (posTrajectory, rotTrajectory);
}

void vector2MultiArray (std_msgs::Float64MultiArray &dest, const vector<double> &src, string label = "")
{
	dest.layout.dim.resize (1);
	dest.layout.dim[0].size = src.size ();
	dest.layout.dim[0].stride = src.size ();
	dest.layout.dim[0].label = label;
	
	dest.data = src;
}

void vector2MultiArrayMatrix (std_msgs::Float64MultiArray &dest, const vector<double> &src, int rows, int cols)
{
	assert (rows * cols == src.size ());
	dest.layout.dim.resize (2);
	dest.layout.dim[0].size =  rows;
	dest.layout.dim[0].stride = src.size ();
	dest.layout.dim[0].label = "";

	dest.layout.dim[1].size = cols;
	dest.layout.dim[1].stride = cols;

	dest.data = src;
}

void rep2MultiArray (std_msgs::Float64MultiArray &dest, const vector<double> &srcBase, int cols)
{
	const int rows = srcBase.size ();
	dest.layout.dim.resize(2);
	// Rows
	dest.layout.dim[0].size = rows;
	dest.layout.dim[0].stride = rows * cols;
	dest.layout.dim[0].label = "Rows";
	// Cols
	dest.layout.dim[1].size = cols;
	dest.layout.dim[1].stride = cols;
	dest.layout.dim[1].label = "Cols";
	
	dest.data.resize (rows * cols);
	
	for (int i = 0; i < cols; i++) {
		for (int j = 0; j < rows; j++)
			dest.data[i*rows + j] = srcBase[j];
	}
}

void InputInterface::sendInput ()
{
	InputDataMsg inputMsg;
	Vector state = getStackedState ();

	//ROS_INFO ("State eef %lg %lg %lg", state[0], state[1], state[2]);
	
	switch (mode)
	{
	case MODE_CONSTANT: {
		Vector mpcRef = getMPCRefConstant ();
		Vector mpcRefTerm = getMPCRefTermConstant ();
		Vector onlineData = getOnlineDataConstant ();
		rep2MultiArray (inputMsg.refWindow, mpcRef, WINDOW_N);
		vector2MultiArray (inputMsg.refTerminal, mpcRefTerm);
		rep2MultiArray (inputMsg.onlineData, onlineData, WINDOW_N + 1);
		break;
	}
	case MODE_TRAJECTORY: {
		Vector mpcRef = getMPCRefTrajectory ();
		Vector mpcRefTerm = getMPCRefTermTrajectory ();
		Vector onlineData = getOnlineDataTrajectory ();

		vector2MultiArrayMatrix (inputMsg.refWindow, mpcRef, EEF_REF, WINDOW_N);
		vector2MultiArray (inputMsg.refTerminal, mpcRefTerm);
		vector2MultiArrayMatrix (inputMsg.onlineData, onlineData, EEF_O_N, WINDOW_N+1);

		break;
	}
	}

	vector2MultiArray (inputMsg.state, state);

	inputPub.publish (inputMsg);
}




























