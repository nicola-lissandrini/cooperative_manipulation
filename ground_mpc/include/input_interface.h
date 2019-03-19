#ifndef CLOSED_LOOP_H
#define CLOSED_LOOP_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/tf.h>

#include "mpc_wrapper/InputDataMsg.h"
#include "ground_mpc/ObstaclesArray.h"
#include "acado_common.h"

typedef std::vector<double> Vector;

#define COLL_AVOID 1
#define EEF_P_N 3
#define EEF_O_N 9
#define JOINT_N 4
#define BASE_PO_N 3
#define OBST_SINGLE 6
#define OBST_N 2
#define OBST_SZ (OBST_SINGLE * OBST_N)
#define OBJ_P_N 3
#define CONTROLS ACADO_NU
#define WINDOW_N ACADO_N

class InputInterface
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber baseSub;
	ros::Subscriber jointsSub;
	ros::Subscriber eefSub;
	ros::Subscriber refSub;
	ros::Subscriber obstSub;
	ros::Subscriber objectSub;
	ros::Publisher inputPub;
	ros::Publisher fixedEefPub;
	ros::Publisher fixedEefOdomPub;
	std::vector<double> jointsOffsets;
	std::vector<int> jointsSigns;
	std::vector<double> eefFixRpy;

	bool started;

	void initParams ();
	void initROS ();
	void sendInput ();
	void checkCollisions ();

	struct RobotState {
		Vector eefPos;
		Vector eefRot;
		Vector jointsVals;
		Vector basePose;
		Vector objectPos;

		RobotState ():
			eefPos(EEF_P_N, 0),
			eefRot(EEF_O_N, 0),
			jointsVals(JOINT_N, 0),
			basePose(BASE_PO_N, 0),
			objectPos(OBJ_P_N, 0)
		{}

	} robotState;

	struct RobotReference {
		Vector eefPosRef;
		Vector eefRotRef;
		Vector obstacles;

		RobotReference():
			eefPosRef(EEF_P_N, 0),
			eefRotRef(EEF_O_N, 0),
			obstacles(OBST_SZ, 0)
		{}

	} robotReference;

	Vector getStackedState ();
	Vector getMPCRef ();
	Vector getMPCRefTerm ();
	Vector getOnlineData ();

	void pubFixed (const geometry_msgs::PoseStamped &fixedMsg);

public:
	InputInterface ();

	int spin ();
	inline void setRobotEef (const Vector &eefPos, const Vector &eefRot) {
		robotState.eefPos = eefPos;
		robotState.eefRot = eefRot;
	}
	inline void setRobotJoints (const Vector &jointsVals);
	inline void setRobotsBasePos (const Vector &basePose) {
		robotState.basePose = basePose;
	}
	inline void setRobotRef (const Vector &eefPosRef, const Vector &eefRotRef) {
		robotReference.eefPosRef = eefPosRef;
		robotReference.eefRotRef = eefRotRef;
	}
	inline void setObstacles (const Vector &obstacle) {
		robotReference.obstacles = obstacle;
	}
	inline void setObjectPos (const Vector &pos) {
		robotState.objectPos = pos;
	}
	inline std::string getJointName (int i) {
		return (std::string) params[(std::string)"joints_names"][i];
	}

	inline bool isStarted () {
		return started;
	}
	inline void start () {
		started = true;
	}

	static void baseCallback (const nav_msgs::Odometry &baseOdometry);
	static void jointsCallback (const std_msgs::Float32MultiArray &jointState);
	static void eefCallback (const nav_msgs::Odometry &eefOdom);
	static void referenceCallback (const geometry_msgs::Pose &ref);
	static void obstacleCallback (const ground_mpc::ObstaclesArray &obstacle);
	static void objectCallback (const nav_msgs::Odometry &object);
	std::vector<double> getEefFixRpy() const;
};

extern InputInterface *inputInterface;

#endif // CLOSED_LOOP_H
