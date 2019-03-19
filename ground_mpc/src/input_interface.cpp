#include "input_interface.h"

#include "mpc.h"

using namespace std;
using namespace ros;
using namespace XmlRpc;
using namespace mpc_wrapper;
using namespace tf;

InputInterface::InputInterface ():
	started(false)
{
	initParams ();
	initROS ();
	FILE *out = fopen ("/home/nicola/stateLog","w");
	fclose (out);
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

std::vector<double> InputInterface::getEefFixRpy() const
{
	return eefFixRpy;
}

void InputInterface::initParams ()
{
	try {
		nh.getParam ("mpc_input_interface", params);
		jointsSigns = vectorFromParam<int> (params["joint_signs"]);
		jointsOffsets = vectorFromParam<double> (params["joint_offsets"]);

		eefFixRpy = vectorFromParam<double> (params["fix_end_effector_rpy"]);

	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s\n",e.getMessage ().c_str ());
	}
}

void InputInterface::initROS ()
{
	rate = new Rate (double (params["rate"]));

	baseSub = nh.subscribe ((string) params["base_state_topic"], 1, baseCallback);
	jointsSub = nh.subscribe ((string) params["joints_state_topic"], 1, jointsCallback);
	eefSub = nh.subscribe ((string) params["end_effector_topic"], 1, eefCallback);
	refSub = nh.subscribe ((string) params["reference_topic"], 1, referenceCallback);
	obstSub = nh.subscribe ((string) params["obstacle_topic"], 1, obstacleCallback);
	objectSub = nh.subscribe((string) params["object_topic"], 1, objectCallback);

	inputPub = nh.advertise<InputDataMsg> ((string) params["mpc_input_topic"], 1);
	fixedEefPub = nh.advertise<geometry_msgs::PoseStamped> ((string) params["fixed_end_effector_pose"], 1);
	fixedEefOdomPub = nh.advertise<nav_msgs::Odometry> ((string) params["fixed_end_effector_odom"], 1);
}

void InputInterface::baseCallback (const nav_msgs::Odometry &baseOdometry)
{
	Quaternion baseQuat;
	Matrix3x3 rotm;
	Vector basePose(BASE_PO_N);
	double r, p, y;

	quaternionMsgToTF (baseOdometry.pose.pose.orientation, baseQuat);
	rotm.setRotation (baseQuat);
	rotm.getEulerYPR (y, p, r);

	basePose[0] = baseOdometry.pose.pose.position.x;
	basePose[1] = baseOdometry.pose.pose.position.y;
	basePose[2] = y;

	inputInterface->setRobotsBasePos (basePose);
}

void InputInterface::jointsCallback (const std_msgs::Float32MultiArray &jointState)
{
	vector<double> jointVals(JOINT_N);
	for (int i = 0; i < JOINT_N; i++)
		jointVals[i] = jointState.data[i];

	inputInterface->setRobotJoints (jointVals);
}

void InputInterface::eefCallback (const nav_msgs::Odometry &eefOdom)
{
	Quaternion eefQuat, eefQuatFix, eefQuatFixed;
	Matrix3x3 eefRotm;
	Vector eefPos(EEF_P_N), eefRot(EEF_O_N);

	geometry_msgs::Pose eefPose = eefOdom.pose.pose;

	quaternionMsgToTF (eefPose.orientation, eefQuat);
	eefQuatFix.setEuler (inputInterface->getEefFixRpy()[2],
			inputInterface->getEefFixRpy()[1],
			inputInterface->getEefFixRpy()[0]);
	eefQuatFixed = eefQuat *  eefQuatFix;
	eefRotm.setRotation (eefQuatFixed);

	eefPos[0] = eefPose.position.x;
	eefPos[1] = eefPose.position.y;
	eefPos[2] = eefPose.position.z;

	for (int i = 0; i < 3; i++) {
		eefRot[i*3] = eefRotm.getColumn (i).x ();
		eefRot[i*3 + 1] = eefRotm.getColumn (i).y ();
		eefRot[i*3 + 2] = eefRotm.getColumn (i).z ();
	}

	geometry_msgs::PoseStamped eefFixedMsg;

	eefFixedMsg.header.frame_id = "qualisys";
	eefFixedMsg.pose.position.x = eefPose.position.x;
	eefFixedMsg.pose.position.y = eefPose.position.y;
	eefFixedMsg.pose.position.z = eefPose.position.z;

	quaternionTFToMsg (eefQuatFixed,eefFixedMsg.pose.orientation);

	inputInterface->pubFixed (eefFixedMsg);
	inputInterface->setRobotEef (eefPos, eefRot);
}

void InputInterface::pubFixed (const geometry_msgs::PoseStamped &fixedMsg) {
	nav_msgs::Odometry odom;

	odom.pose.pose = fixedMsg.pose;
	fixedEefPub.publish (fixedMsg);
	fixedEefOdomPub.publish (odom);
}

void InputInterface::obstacleCallback (const ground_mpc::ObstaclesArray &obstacle)
{
	Vector obstData(OBST_SZ);

	for (int i = 0; i < obstacle.obstacles.size(); i++) {
		obstData[i*OBST_SINGLE + 0] = obstacle.obstacles[i].center.x;
		obstData[i*OBST_SINGLE + 1] = obstacle.obstacles[i].center.y;
		obstData[i*OBST_SINGLE + 2] = obstacle.obstacles[i].center.z;
		obstData[i*OBST_SINGLE + 3] = obstacle.obstacles[i].axes.x;
		obstData[i*OBST_SINGLE + 4] = obstacle.obstacles[i].axes.y;
		obstData[i*OBST_SINGLE + 5] = obstacle.obstacles[i].axes.z;
	}

	inputInterface->setObstacles (obstData);
}

void InputInterface::objectCallback (const nav_msgs::Odometry &object) {
	inputInterface->setObjectPos (Vector({object.pose.pose.position.x,
										  object.pose.pose.position.y,
										  object.pose.pose.position.z}));
}

Vector InputInterface::getStackedState ()
{
	Vector stacked;
	int global = 0;
	int i;

	stacked.resize (EEF_P_N + EEF_O_N + JOINT_N + BASE_PO_N);

	for (i = 0; i < EEF_P_N; i++) {
		stacked[global + i] = robotState.eefPos[i];
	}
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


	FILE *out = fopen ("/home/nicola/stateLog","a");
	fprintf (out, "%lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg\n",robotState.eefPos[0],robotState.eefPos[1],robotState.eefPos[2],  robotState.jointsVals[0],  robotState.jointsVals[1],  robotState.jointsVals[2],  robotState.jointsVals[3],
			robotState.basePose[0],  robotState.basePose[1],  robotState.basePose[2]);
	fclose (out);

	return stacked;
}

Vector InputInterface::getMPCRef ()
{
	Vector mpcRef;
	Vector term = getMPCRefTerm ();
	int i;

	mpcRef.resize (term.size () + CONTROLS);

	for (i = 0; i < term.size (); i++)
		mpcRef[i] = term[i];
	for (; i < mpcRef.size (); i++)
		mpcRef[i] = 0;

	return mpcRef;
}

Vector InputInterface::getMPCRefTerm ()
{
	Vector mpcRef;
	int i;

	mpcRef.resize (EEF_P_N + EEF_O_N + COLL_AVOID);

	for (i = 0; i < EEF_P_N; i++)
		mpcRef[i] = robotReference.eefPosRef[i];
	for (; i < mpcRef.size (); i++)
		mpcRef[i] = 0;

	return mpcRef;
}

Vector InputInterface::getOnlineData ()
{
	const int eefRotSize = robotReference.eefRotRef.size ();
	const int obstSize = robotReference.obstacles.size ();
	const int keyptsSize = robotState.objectPos.size ();
	Vector stackedOnlineData(eefRotSize + obstSize + keyptsSize);

	for (int i = 0; i < eefRotSize; i++)
		stackedOnlineData[i] = robotReference.eefRotRef[i];
	for (int i = 0; i < obstSize; i++)
		stackedOnlineData[eefRotSize + i] = robotReference.obstacles[i];
	for (int i = 0; i < keyptsSize; i++)
		stackedOnlineData[eefRotSize + obstSize + i] = robotState.objectPos[i];
	return stackedOnlineData;
}

void InputInterface::checkCollisions ()
{
	// Not implemented yet.
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

void InputInterface::setRobotJoints(const Vector &jointsVals) {

	for (int i = 0; i < JOINT_N; i++) {
		// only needed in some cases for shoulder joint in the real device
		if (i == 0 && (abs (jointsOffsets[i]) > 0.1)) {
			const double thSensor = double (jointsSigns[i]) * jointsVals[i];
			robotState.jointsVals[i] = atan2 (-cos(thSensor), sin(thSensor));
			// else we are in simulation -- HORRIBLE WORKAROUND
		} else
			robotState.jointsVals[i] = double (jointsSigns[i]) * jointsVals[i] + jointsOffsets[i];
#ifdef DEBUG_JOINTS
		ROS_INFO ("i %d, % d * %lg + %lg = total %lg",
				  i,
				  jointsSigns[i],
				  jointsVals[i],
				  jointsOffsets[i],
				  robotState.jointsVals[i]);
#endif
	}
}


void InputInterface::referenceCallback (const geometry_msgs::Pose &ref)
{
	Quaternion quat;
	Matrix3x3 rotm;
	Vector eefPosRef(EEF_P_N);
	Vector eefRotRef(EEF_O_N);

	if (!inputInterface->isStarted ())
		inputInterface->start ();

	eefPosRef[0] = ref.position.x;
	eefPosRef[1] = ref.position.y;
	eefPosRef[2] = ref.position.z;

	quaternionMsgToTF (ref.orientation, quat);
	rotm.setRotation (quat);

	for (int i = 0; i < 3; i++) {
		eefRotRef[i*3] = rotm.getColumn (i).x ();
		eefRotRef[i*3 + 1] = rotm.getColumn (i).y ();
		eefRotRef[i*3 + 2] = rotm.getColumn (i).z ();
	}

	inputInterface->setRobotRef (eefPosRef, eefRotRef);
}

void vector2MultiArray (std_msgs::Float64MultiArray &dest, const vector<double> &src, string label = "")
{
	dest.layout.dim.resize (1);
	dest.layout.dim[0].size = src.size ();
	dest.layout.dim[0].stride = src.size ();
	dest.layout.dim[0].label = label;

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
	Vector mpcRef = getMPCRef ();
	Vector mpcRefTerm = getMPCRefTerm ();
	Vector onlineData = getOnlineData ();

	//ROS_INFO ("State eef %lg %lg %lg", state[0], state[1], state[2]);

	vector2MultiArray (inputMsg.state, state);
	rep2MultiArray (inputMsg.refWindow, mpcRef, WINDOW_N);
	vector2MultiArray (inputMsg.refTerminal, mpcRefTerm);
	rep2MultiArray (inputMsg.onlineData, onlineData, WINDOW_N + 1);

	inputPub.publish (inputMsg);
}




























