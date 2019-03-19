#ifndef COMMANDER_H
#define COMMANDER_H

#define QUA ROS_INFO("Reached %d:%s", __LINE__, __FILE__);

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <xmlrpcpp/XmlRpc.h>
#include <eigen3/Eigen/Dense>
#include "easy_gripper/GripCommand.h"
#include "aerial_mpc/Trajectory.h"
#include "aerial_mpc/OutputDataMsg.h"


enum CommanderStatus {
	COMMANDER_NOT_READY,
	COMMANDER_AERIAL_TAKEOFF,
	COMMANDER_IDLE,
	COMMANDER_APPROACHING,
	COMMANDER_REACHED,
	COMMANDER_WAIT_POSE,
	COMMANDER_COOPERATIVE_TASK_HANDSHAKE,
	COMMANDER_COOPERATIVE_TASK,
	COMMANDER_TASK_COMPLETE,
	COMMANDER_END,
	COMMANDER_ABORT
};

enum CommanderCommand {
	COMMAND_START,
	COMMAND_ABORT
};

enum Role {
	ROLE_LEADER,
	ROLE_FOLLOWER
};

class ArenaEntity
{
	ros::Subscriber rosOdomSub;
	nav_msgs::Odometry entityOdom;
	std::string entityName;

protected:
	ros::NodeHandle *rosNode;

public:
	ArenaEntity (const std::string &name):
		entityName(name)
	{}

	void initROS (ros::NodeHandle *node, const std::string &odomTopic);

	void odometryCallback (const nav_msgs::Odometry &newOdom);

	inline const nav_msgs::Odometry &odometry ();
	inline tf::Pose pose();
	inline tf::Vector3 linearVel ();
	inline tf::Vector3 angularVel ();
	inline std::string name();
};

class Agent : public ArenaEntity
{
	ros::Publisher rosCmdPub;

	Role agentRole;
	tf::Pose currentSetpoint;
	struct DefaultPoses {
		tf::Pose agentGripPose;
		tf::Pose terminationPose;
		// tf::Pose idlePose; -- Not used yet
	} defaultPoses;
	std::string agentGripperLink;

	struct {
		double angular;
		double linear;
		double velocity;
	} threshold;

public:
	Agent (const std::string &name, Role _role):
		ArenaEntity( name),
		agentRole(_role)
	{}

	void commandPose (const tf::Pose &cmd);
	bool setpointReached ();

	void initROS (ros::NodeHandle *node, const std::string &cmdTopic, const std::string &odomTopic);

	inline void setParams (const tf::Pose &gripPose, const tf::Pose &terminationPose,
						   const std::string &gripperLinkName,
						   double angularEps,
						   double linearEps,
						   double velocityEps);
	inline Role role () const;
	inline std::string gripperLink() const;
	inline tf::Pose gripPose () const;
	inline tf::Pose terminationPose () const;
	tf::Pose getCurrentSetpoint () const {
		return currentSetpoint;
	}
};

class Leader : public Agent
{
	ros::Subscriber predSub;
	aerial_mpc::Trajectory prediction;
	// Object to ground eef transform
	tf::Transform relativeTransform;

public:
	Leader (const std::string &name):
		Agent (name, ROLE_LEADER)
	{}

	void initROS (ros::NodeHandle *node,
				  const std::string &cmdTopic,
				  const std::string &odomTopic,
				  const std::string &predTopic);

	aerial_mpc::Trajectory getPrediction ();
	void predictionCallback (const aerial_mpc::Trajectory &newPrediction);
	void commandObjectPose (const tf::Pose &objectPose);
	inline void setRelativeTransform (const tf::Transform &transform) {
		relativeTransform = transform;
	}
	inline tf::Transform getRelativeTransform () {
		return relativeTransform;
	}
};

class Follower : public Agent
{
	ros::Publisher trajPub;
	ros::Publisher aerialTrajectoryPub;
	// Ground to aerial frame transform
	tf::Transform relativeTransform;

public:
	Follower (const std::string &name):
		Agent (name, ROLE_FOLLOWER)
	{}

	void initROS  (ros::NodeHandle *node,
				   const std::string &cmdTopic,
				   const std::string &odomTopic,
				   const std::string &trajTopic, const std::string &referenceTopic);
	// Relative to ground eef
	inline void setRelativeTransform (const tf::Transform &transform) {
		relativeTransform = transform;
	}
	inline tf::Transform getRelativeTransform () {
		return relativeTransform;
	}
	void commandTrajectory (const aerial_mpc::Trajectory &traj);
};

class Object : public ArenaEntity
{
	std::string objectLinkName;

public:
	Object (const std::string &name):
		ArenaEntity (name)
	{}

	void initROS (ros::NodeHandle *node, const std::string odomTopic);
	void setParams (const std::string &linkName);
	std::string linkName () const;
};

class GripperController
{
	ros::NodeHandle *rosNode;

	ros::Publisher gripperCmdPub;

	enum GripCommand {
		GRIP_ATTACH = 0,
		GRIP_DETACH = 1,
		GRIP_INFO = 2 // Debug only
	};

	void command (const std::string &parent, const std::string &child, GripCommand cmd);

public:
	GripperController ()
	{}

	void initROS (ros::NodeHandle *node, const std::string &gripperTopic);
	void attach (const std::string &parent, const std::string &child);
	void detach (const std::string &parent, const std::string &child);
	void info ();
};

class ObjectGripperController : public GripperController
{
	std::string objectLinkName;

public:
	ObjectGripperController()
	{}

	void setObject (const Object &object) {
		objectLinkName = object.linkName ();
	}
	void attachAgent (const Agent &agent) {
		attach (agent.gripperLink (), objectLinkName);
	}
	void detachAgent (const Agent &agent) {
		detach (agent.gripperLink (), objectLinkName);
	}
};

class Commander
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	ros::Subscriber commandSub;
	ros::Subscriber targetPoseSub;
	ros::Publisher enablePub;
	ros::Publisher disablePub;
	ros::Publisher errorPub;
	XmlRpc::XmlRpcValue params;

	Leader groundAgent;
	Follower aerialAgent;
	Object object;
	ObjectGripperController gripper;

	tf::Pose targetPose;

	CommanderStatus status;
	struct Flags {
		bool startFlag;
		bool abortFlag;
		bool commandReceived;
		bool attachEnabled;

		Flags ():
			abortFlag(false),
			startFlag(false),
			commandReceived(false),
			attachEnabled(false)
		{}
	} flags;

	double paramDouble (XmlRpc::XmlRpcValue &param);
	tf::Pose paramPose (XmlRpc::XmlRpcValue &param);
	std::string paramString (XmlRpc::XmlRpcValue &param);

	void initROS ();
	void initParams ();

	void enabledMPCReg (bool val) {
		std_msgs::String msg;

		msg.data = "/neo11/aerial_mpc";

		if (val)
			enablePub.publish (msg);
		else
			disablePub.publish (msg);
		sleep (0.1);
	}

	void enabledMPCTraj (bool val) {
		std_msgs::String msg;

		msg.data = "/neo11/aerial_mpc_traj";

		if (val)
			enablePub.publish (msg);
		else
			disablePub.publish (msg);
		sleep (0.1);
	}
	void dispatchActions ();
	void startObjectApproach (Agent &agent);
	void startTakeoff (Agent &agent);
	void startLanding (Agent &agent);
	tf::Pose getGripPose(const Agent &agent);

	void handshake ();
	void cooperativeTaskStart ();
	void cooperativeTaskSpin ();
	void startTerminationTask ();

	bool checkReached ();
	bool checkTaskCompleted ();
	void pubError ();

public:
	Commander ();

	int spin ();
	static void objectOdomCallback (const nav_msgs::Odometry &odom);
	static void targetPoseCallback (const geometry_msgs::Pose &pose);
	static void commandCallback (const std_msgs::Int8 &cmd);

	inline void start () {
		flags.startFlag = true;
	}
	inline void abort () {
		flags.abortFlag = true;
		flags.startFlag = false;
	}
	inline void setTargetPose (const geometry_msgs::Pose &pose) {
		flags.commandReceived = true;
		tf::poseMsgToTF (pose, targetPose);
	}
	inline void setCommand (CommanderCommand cmd);
};

extern Commander *commander;

#endif // COMMANDER_H
