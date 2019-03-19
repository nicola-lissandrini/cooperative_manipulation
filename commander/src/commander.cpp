#include "commander.h"
#include <std_msgs/Float64.h>

using namespace ros;
using namespace std;
using namespace tf;
using namespace XmlRpc;

#define INT_ERROR_MSG "Commander: internal fatal error while starting object approach"

void ArenaEntity::initROS (NodeHandle *node, const string &odomTopic)
{
	rosNode = node;
	rosOdomSub = rosNode->subscribe (odomTopic, 1, &ArenaEntity::odometryCallback, this);
}

void ArenaEntity::odometryCallback (const nav_msgs::Odometry &newOdom) {
	entityOdom = newOdom;
}

const nav_msgs::Odometry &ArenaEntity::odometry() {
	return entityOdom;
}

Pose ArenaEntity::pose() {
	Pose pose;
	poseMsgToTF (entityOdom.pose.pose, pose);
	return pose;
}

Vector3 ArenaEntity::linearVel() {
	Vector3 vel;
	vector3MsgToTF (entityOdom.twist.twist.linear, vel);
	return vel;
}

Vector3 ArenaEntity::angularVel() {
	Vector3 vel;
	vector3MsgToTF (entityOdom.twist.twist.angular, vel);
	return vel;
}

string ArenaEntity::name() {
	return entityName;
}

void Leader::initROS (NodeHandle *node, const string &cmdTopic, const string &odomTopic, const string &predTopic)
{
	Agent::initROS (node, cmdTopic, odomTopic);

	predSub = node->subscribe (predTopic, 1, &Leader::predictionCallback, this);
}

aerial_mpc::Trajectory Leader::getPrediction() {
	return prediction;
}

void Leader::predictionCallback (const aerial_mpc::Trajectory &newPrediction) {
	prediction = newPrediction;
}

void Leader::commandObjectPose(const Pose &objectPose)
{
	Pose groundPose = objectPose * relativeTransform;

	commandPose (groundPose);
}

void Agent::initROS (ros::NodeHandle *node, const string &cmdTopic, const string &odomTopic)
{
	ArenaEntity::initROS (node, odomTopic);

	rosCmdPub = rosNode->advertise<geometry_msgs::Pose> (cmdTopic, 1);
}

void Agent::setParams(const Pose &gripPose,
					  const Pose &terminationPose,
					  const string &gripperLinkName,
					  double angularEps, double linearEps,
					  double velocityEps)
{
	defaultPoses.agentGripPose = gripPose;
	defaultPoses.terminationPose = terminationPose;
	agentGripperLink = gripperLinkName;
	threshold.angular = angularEps;
	threshold.linear = linearEps;
	threshold.velocity = velocityEps;
}

Role Agent::role() const {
	return agentRole;
}

string Agent::gripperLink() const {
	return agentGripperLink;
}

Pose Agent::gripPose() const {
	return defaultPoses.agentGripPose;
}

Pose Agent::terminationPose() const {
	return defaultPoses.terminationPose;
}

void Agent::commandPose (const Pose &cmd)
{
	geometry_msgs::Pose poseMsg;

	currentSetpoint.setOrigin (cmd.getOrigin ());
	currentSetpoint.setRotation (cmd.getRotation ());
	poseTFToMsg (cmd, poseMsg);
	rosCmdPub.publish (poseMsg);
}

// Metrics
double riemannDist (const Quaternion &a, const Quaternion &b)
{
	Quaternion diff;

	diff = a.inverse () * b;


	return diff.getAngle ();
}

double riemannDist (const Pose &a, const Pose &b) {
	return riemannDist (a.getRotation (), b.getRotation ());
}

double norm (const Vector3 &a) {
	return a.dot(a);
}

double dist (const Vector3 &a, const Vector3 &b) {
	Vector3 diff = a - b;

	return norm(diff);
}

double linearDist (const Pose &a, const Pose &b) {
	return dist (a.getOrigin (), b.getOrigin ());
}

bool Agent::setpointReached ()
{
	 ROS_INFO("rot %lg  lin %lg vel %lg",riemannDist (pose (), currentSetpoint),linearDist (pose (), currentSetpoint) ,norm (linearVel ()));
	ROS_INFO ("riee %lg %lg %lg %lg",currentSetpoint.getRotation ().getX (),currentSetpoint.getRotation ().getY (),currentSetpoint.getRotation ().getZ (),currentSetpoint.getRotation ().getW ());
	ROS_INFO ("riee %lg %lg %lg %lg",pose ().getRotation ().getX (),
			  pose ().getRotation ().getY (),pose ().getRotation ().getZ (),
			  pose ().getRotation ().getW ());

	if (linearDist (pose (), currentSetpoint) < threshold.linear) {
		if (norm (linearVel ()) < threshold.velocity) {
			return true;
		}else
			return false;
	}
	else
		return false;
}

void Object::initROS (NodeHandle *node, const string odomTopic)
{
	ArenaEntity::initROS (node, odomTopic);
}

void Object::setParams (const string &linkName) {
	objectLinkName = linkName;
}

string Object::linkName() const {
	return objectLinkName;
}

void GripperController::initROS (NodeHandle *node, const string &gripperTopic)
{
	rosNode = node;
	gripperCmdPub = rosNode->advertise<easy_gripper::GripCommand> (gripperTopic, 1);
}

void GripperController::command (const string &gripperLink, const string &objectLink, GripCommand cmd)
{
	easy_gripper::GripCommand gripMsg;

	gripMsg.gripper_link = gripperLink;
	gripMsg.object_link = objectLink;
	gripMsg.command = (int) cmd;

	gripperCmdPub.publish (gripMsg);
}

void GripperController::attach (const string &parent, const string &child) {
	command (parent, child, GRIP_ATTACH);
}

void GripperController::detach (const string &parent, const string &child) {
	command (parent, child, GRIP_DETACH);
}

void GripperController::info () {
	command ("","", GRIP_INFO);
}

Commander::Commander():
	status(COMMANDER_NOT_READY),
	aerialAgent("aerial"),
	groundAgent("ground"),
	object("object")
{
	initParams ();
	initROS ();
}

double Commander::paramDouble (XmlRpcValue &param)
{
	switch (param.getType ())
	{
	case XmlRpcValue::TypeInt:
		return (double) int (param);
	case XmlRpcValue::TypeDouble:
		return double (param);
	default:
		ROS_ERROR ("Invaild datatype %d, expecting %d",param.getType (), XmlRpcValue::TypeDouble);
		throw XmlRpcException ("Invalid datatype, expecting double");
	}
}

string Commander::paramString (XmlRpcValue &param)
{
	if (param.getType () != XmlRpcValue::TypeString)
		throw XmlRpcException ("Invalid datatype, expecting string");
	return string (param);
}

Pose Commander::paramPose (XmlRpcValue &param)
{
	Pose ret;

	ret.setOrigin (Vector3 (
					   paramDouble (param["pos"]["x"]),
			paramDouble (param["pos"]["y"]),
			paramDouble (param["pos"]["z"])));
	ret.setRotation (Quaternion (
						 paramDouble (param["rot"]["x"]),
			paramDouble (param["rot"]["y"]),
			paramDouble (param["rot"]["z"]),
			paramDouble (param["rot"]["w"])));
	return ret;
}

void Commander::initParams()
{
	try {
		nh.getParam ("commander", params);

		aerialAgent.setParams (paramPose (params["aerial"]["grip_pose"]),
				paramPose (params["aerial"]["termination_pose"]),
				paramString (params["aerial"]["gripper_link"]),
				paramDouble (params["aerial"]["reach_threshold"]["angular"]),
				paramDouble (params["aerial"]["reach_threshold"]["linear"]),
				paramDouble (params["aerial"]["reach_threshold"]["velocity"]));

		groundAgent.setParams (paramPose (params["ground"]["grip_pose"]),
				paramPose (params["ground"]["termination_pose"]),
				paramString (params["ground"]["gripper_link"]),
				paramDouble (params["ground"]["reach_threshold"]["angular"]),
				paramDouble (params["ground"]["reach_threshold"]["linear"]),
				paramDouble (params["ground"]["reach_threshold"]["velocity"]));

		object.setParams (paramString (params["object"]["gripper_link"]));
		flags.attachEnabled = (bool) params["attach_enable"];

	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s\n",e.getMessage ().c_str ());
	}
}

void Commander::initROS ()
{
	rate = new Rate (double (params["rate"]));

	aerialAgent.initROS (&nh, paramString (params["aerial"]["cmd_topic"]),
			paramString (params["aerial"]["odom_topic"]),
			paramString (params["aerial"]["trajectory_topic"]),
			paramString (params["aerial"]["trajectory_topic"]) + "_print");

	groundAgent.initROS (&nh, paramString (params["ground"]["cmd_topic"]),
			paramString (params["ground"]["odom_topic"]),
			paramString (params["ground"]["prediction_topic"]));

	object.initROS (&nh, paramString (params["object"]["odom_topic"]));

	gripper.initROS (&nh, paramString (params["gripper_topic"]));

	gripper.setObject (object);

	targetPoseSub = nh.subscribe (paramString (params["target_pose_topic"]), 1, targetPoseCallback);
	commandSub = nh.subscribe (paramString (params["command_topic"]), 1, commandCallback);

	enablePub = nh.advertise<std_msgs::String> (paramString (params["mpc_enable_topic"]), 1);
	disablePub = nh.advertise<std_msgs::String> (paramString (params["mpc_disable_topic"]), 1);
	errorPub = nh.advertise<std_msgs::Float64> ("/aerial_tracking_error", 1);
}

void Commander::targetPoseCallback (const geometry_msgs::Pose &pose) {
	commander->setTargetPose (pose);
}

void Commander::commandCallback (const std_msgs::Int8 &cmd) {
	commander->setCommand ((CommanderCommand) cmd.data);
}

void Commander::setCommand (CommanderCommand cmd) {
	switch (cmd)
	{
	case COMMAND_START:
		start ();
		break;
	case COMMAND_ABORT:
		abort ();
		break;
	}
}

Pose Commander::getGripPose (const Agent &agent) {
	return object.pose () * agent.gripPose ();
}

void Commander::startObjectApproach (Agent &agent)
{
	Pose startTargetPose = getGripPose (agent);

	agent.commandPose (startTargetPose);
}

void Commander::startTakeoff (Agent &agent)
{
	Pose hoveringPose;

	Quaternion hoveringOrientation(-0.706328108495,
									0.0331753394794,
								   -0.0331753394794,
									0.706328108495);

	//hoveringPose.setOrigin (Vector3 (0.47, -2.07, 0.8));

	Pose agentPose = agent.pose ();
	hoveringPose.setOrigin (agentPose.getOrigin () + Vector3 (0, 0, 0.5));
	hoveringPose.setRotation (hoveringOrientation);

	agent.commandPose (hoveringPose);
}


void Commander::startLanding (Agent &agent)
{
	Pose landingPose;

	Pose agentPose = agent.pose ();
	Vector3 pos = agentPose.getOrigin ();

	landingPose.setOrigin (Vector3 (pos.getX (), pos.getY (), -0.1));

	agent.commandPose (landingPose);
}

bool Commander::checkReached () {
	return aerialAgent.setpointReached () &&
			groundAgent.setpointReached ();
}

void Commander::handshake ()
{
	// Agents exchange information about relative position
	Pose leaderPose = groundAgent.pose ();
	Pose followerPose = aerialAgent.pose ();
	Pose objectPose = object.pose ();

	Transform objectToLeader = objectPose.inverse () * leaderPose;
	Transform leaderToFollower = leaderPose.inverse () * followerPose;

	aerialAgent.setRelativeTransform (leaderToFollower);
	groundAgent.setRelativeTransform (objectToLeader);
}

void Commander::cooperativeTaskSpin ()
{
#define COOPERATIVE
#ifdef COOPERATIVE
	// Get prediction from leader
	aerial_mpc::Trajectory prediction = groundAgent.getPrediction ();
	// Send trajectory to follower

	aerialAgent.commandTrajectory (prediction);

	Pose aerialPose = aerialAgent.pose ();
	geometry_msgs::Pose  aerialRef = prediction.trajectory[1];

	double e_x = aerialRef.position.x - aerialPose.getOrigin ().getX ();
	double e_y = aerialRef.position.y - aerialPose.getOrigin ().getY ();
	double e_z = aerialRef.position.z - aerialPose.getOrigin ().getZ ();
	std_msgs::Float64 msg;

	msg.data = sqrt (e_x*e_x + e_y*e_y + e_z*e_z);

	errorPub.publish (msg);
#else
	Transform objectToLeader = groundAgent.getRelativeTransform ();
	Transform leaderToFollower = aerialAgent.getRelativeTransform ();
	aerialAgent.commandPose (targetPose * objectToLeader * leaderToFollower);
#endif
}

void Follower::initROS (NodeHandle *node, const string &cmdTopic, const string &odomTopic, const string &trajTopic, const string &referenceTopic)
{
	Agent::initROS (node, cmdTopic, odomTopic);

	trajPub = node->advertise<aerial_mpc::Trajectory> (trajTopic, 1);
	aerialTrajectoryPub = node->advertise<aerial_mpc::OutputDataMsg> (referenceTopic, 1);
}

void Follower::commandTrajectory (const aerial_mpc::Trajectory &traj)
{
	aerial_mpc::Trajectory trajAerialFrame;
	Transform groundFrame;
	const int trajSize = traj.trajectory.size ();// Debug only: fake outputmsg for prediction printer
	// State only
	aerial_mpc::OutputDataMsg outputTraj;
	poseMsgToTF (traj.trajectory[0], groundFrame);
	Transform leaderToFollower = groundFrame;

	/*ROS_INFO ("T_G_A %lg %lg %lg", leaderToFollower.getOrigin ().x(), leaderToFollower.getOrigin ().y(),leaderToFollower.getOrigin ().z());
	Matrix3x3 rotm;
	rotm.setRotation (leaderToFollower.getRotation ());
	ROS_INFO ("%lg %lg %lg", rotm.transpose ().getColumn (0)[0], rotm.transpose ().getColumn (0)[1], rotm.transpose ().getColumn (0)[2]);
	ROS_INFO ("%lg %lg %lg", rotm.transpose ().getColumn (1)[0], rotm.transpose ().getColumn (1)[1], rotm.transpose ().getColumn (1)[2]);
	ROS_INFO ("%lg %lg %lg", rotm.transpose ().getColumn (2)[0], rotm.transpose ().getColumn (2)[1], rotm.transpose ().getColumn (2)[2]);
*/

	outputTraj.stateTrajectory.layout.dim.resize (2);
	outputTraj.stateTrajectory.layout.dim[0].size = 3; // just the eef position
	outputTraj.stateTrajectory.layout.dim[1].size = trajSize;
	outputTraj.stateTrajectory.data.resize (trajSize * 3);

	trajAerialFrame.trajectory.resize (trajSize);

	// Convert trajectory into aerial frame
	for (int i = 0; i < trajSize; i++) {
		Transform aerialFrame;
		poseMsgToTF (traj.trajectory[i], groundFrame);
		aerialFrame = groundFrame * relativeTransform;
		poseTFToMsg (aerialFrame, trajAerialFrame.trajectory[i]);

		//ROS_INFO ("%lg %lg %lg -- q %lg %lg %lg %lg", aerialFrame.getOrigin ().x (), aerialFrame.getOrigin ().z (), aerialFrame.getOrigin ().z (),
		//		  aerialFrame.getRotation().getX (), aerialFrame.getRotation().getY (), aerialFrame.getRotation().getZ (), aerialFrame.getRotation().getW ());

		outputTraj.stateTrajectory.data[i * 3 + 0] = aerialFrame.getOrigin ().x ();
		outputTraj.stateTrajectory.data[i * 3 + 1] = aerialFrame.getOrigin ().y ();
		outputTraj.stateTrajectory.data[i * 3 + 2] = aerialFrame.getOrigin ().z ();
	}

	aerialTrajectoryPub.publish (outputTraj);
	trajPub.publish (trajAerialFrame);
}

bool Commander::checkTaskCompleted () {
	return groundAgent.setpointReached ();
}

void Commander::startTerminationTask ()
{
	// Detach gripper
	/*if (flags.attachEnabled) {
		gripper.detachAgent (aerialAgent);
		sleep (1);
		gripper.detachAgent (groundAgent);
		sleep (1);
	}*/
	// Get back from object
	//aerialAgent.commandPose (aerialAgent.pose ());
	//groundAgent.commandPose (groundAgent.pose ());
}

void Commander::dispatchActions ()
{
	if (flags.abortFlag)
		status = COMMANDER_ABORT;

	switch (status)
	{
	case COMMANDER_NOT_READY:
		if (flags.startFlag) {
			flags.startFlag = false;
			ROS_INFO ("Commander: starting aerial take off");
			enabledMPCTraj (false);
			startTakeoff (aerialAgent);

			status = COMMANDER_AERIAL_TAKEOFF;
		}
		break;
	case COMMANDER_AERIAL_TAKEOFF:
		if (aerialAgent.setpointReached ()) {
			ROS_INFO ("Commander: aerial robot took off. Waiting for start.");
			status = COMMANDER_IDLE;
		}
		break;
	case COMMANDER_IDLE:
		if (true) {
			flags.startFlag = false;
			ROS_INFO ("Commander: Starting approach task");
			startObjectApproach (groundAgent);
			sleep(1);
			startObjectApproach (aerialAgent);
			status = COMMANDER_WAIT_POSE;
		}
		break;
	case COMMANDER_APPROACHING:
		if (checkReached ())
			status = COMMANDER_REACHED;
		// else keep spinning
		break;
	case COMMANDER_REACHED:
		ROS_INFO ("Commander: robots approached. Attaching");

		ROS_INFO ("Commander: Attaching completed");
		status = COMMANDER_WAIT_POSE;
		break;
	case COMMANDER_WAIT_POSE: {
		Pose startTargetPose = getGripPose (aerialAgent);
		Pose aerialPose = aerialAgent.pose ();
		double err = (startTargetPose.getOrigin () - aerialPose.getOrigin ()).length ();
		std_msgs::Float64 msg;

		msg.data = err;

		errorPub.publish (msg);

		if (flags.commandReceived) {
			flags.commandReceived = false;
			ROS_INFO ("Commander: target pose command set. Attaching");
			enabledMPCReg (false);
			enabledMPCTraj (true);
			if (flags.attachEnabled) {
				gripper.attachAgent (groundAgent);
				sleep(1);
				//gripper.attachAgent (aerialAgent);
				sleep(1);
			}
			status = COMMANDER_COOPERATIVE_TASK_HANDSHAKE;
		} else {
			startObjectApproach (groundAgent);
		}
	}
		break;
	case COMMANDER_COOPERATIVE_TASK_HANDSHAKE:
		ROS_INFO ("Commander: performing initial handshake");
		// Perform initial handshake on relative position
		handshake ();
		// Leader computes its trajectory independently
		groundAgent.commandObjectPose (targetPose);
		ROS_INFO ("Commander: starting task");
		status = COMMANDER_COOPERATIVE_TASK;
		break;
	case COMMANDER_COOPERATIVE_TASK:
		// Perform one spin of the cooperative task
		cooperativeTaskSpin ();
		/*if (checkTaskCompleted ()) {
			sleep (1);
			status = COMMANDER_TASK_COMPLETE;
		}*/
		break;
	case COMMANDER_TASK_COMPLETE:
		ROS_INFO ("Commander: cooperative task completed. Starting termination task");
		enabledMPCTraj (false);
		enabledMPCReg (true);
		sleep(1);
		startTerminationTask ();
		status = COMMANDER_END;
		break;
	case COMMANDER_END:
		if (checkReached ()) {
			ROS_INFO ("Commander: experiment finished.");
			status = COMMANDER_IDLE;
		}
		break;
	case COMMANDER_ABORT:
		startLanding (aerialAgent);
		gripper.detachAgent (aerialAgent);
		sleep (0.5);
		gripper.detachAgent (groundAgent);
		status = COMMANDER_NOT_READY;
		break;
	}
}

int Commander::spin ()
{
	while (ok ()) {
		dispatchActions ();
		spinOnce ();
		rate->sleep ();
	}

	return 0;
}
