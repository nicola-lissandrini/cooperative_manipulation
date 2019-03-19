#ifndef CLOSED_LOOP_H
#define CLOSED_LOOP_H

#include <ros/ros.h>
#include <XmlRpc.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/tf.h>
#include <std_msgs/Float32MultiArray.h>

#include "mpc_wrapper/InputDataMsg.h"
#include "ground_mpc/ObstaclesArray.h"
#include "aerial_mpc/Trajectory.h"
#include "acado_common.h"

typedef std::vector<double> Vector;


#define COLL_AVOID 1
#define EEF_P_N 3
#define EEF_O_N 9
#define JOINT_N 2
#define BASE_PO_N 4
#define OBST_SINGLE 6
#define OBST_N 2
#define OBST_SZ (OBST_SINGLE * OBST_N)
#define STATES ACADO_NX
#define CONTROLS ACADO_NU
#define WINDOW_N ACADO_N
#define EEF_REF_TERM (EEF_P_N + EEF_O_N + 1)
#define EEF_REF (EEF_REF_TERM + CONTROLS)
#define EEF_P_TRAJ_N (EEF_REF * WINDOW_N)
#define EEF_O_TRAJ_N (EEF_O_N * (WINDOW_N +1))


class InputInterface
{
	ros::NodeHandle nh;
	ros::Rate *rate;
	XmlRpc::XmlRpcValue params;
	ros::Subscriber baseSub;
	ros::Subscriber jointsSub;
	ros::Subscriber linksSub;
	ros::Subscriber refSub;
	ros::Subscriber trajSub;
	ros::Subscriber obstSub;
	ros::Publisher inputPub;

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

		RobotState ():
			eefPos(EEF_P_N, 0),
			eefRot(EEF_O_N, 0),
			jointsVals(JOINT_N, 0),
			basePose(BASE_PO_N, 0)
		{}
	} robotState;

	struct RobotReference {
		Vector eefPosRefConstant;
		Vector eefRotRefConstant;
		std::vector<Vector> eefPosRefTrajectory;
		std::vector<Vector> eefRotRefTrajectory;
		Vector obstacles;

		RobotReference():
			eefPosRefConstant(EEF_P_N, 0),
			eefRotRefConstant(EEF_O_N, 0),
			eefPosRefTrajectory(WINDOW_N, Vector(EEF_P_N, 0)),
			eefRotRefTrajectory(WINDOW_N, Vector(EEF_O_N, 0)),
			obstacles(OBST_SZ, 0)
		{}
	} robotReference;

	enum InputMode
	{
		MODE_CONSTANT,
		MODE_TRAJECTORY
	} mode;

	Vector getStackedState ();
	Vector getMPCRefConstant ();
	Vector getMPCRefTermConstant ();
	Vector getOnlineDataConstant ();
	Vector getMPCRefTrajectory ();
	Vector getMPCRefTermTrajectory ();
	Vector getOnlineDataTrajectory ();

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
	inline void setRobotRefConstant (const Vector &eefPosRef, const Vector &eefRotRef);
	inline void setRobotRefTrajectory (const std::vector<Vector> &eefPosRef, const std::vector<Vector> &eefRotRef);
	inline void setObstacles (const Vector &obstacle) {
		robotReference.obstacles.resize (obstacle.size ());
		robotReference.obstacles = obstacle;
	}
	inline std::string getJointName (int i) {
		return (std::string) params[(std::string)"joints_names"][i];
	}

	inline bool isStarted () const {
		return started;
	}
	inline void start () {
		started = true;
	}

	static void baseCallback (const nav_msgs::Odometry &baseOdometry);
	static void jointsCallback (const std_msgs::Float32MultiArray &jointState);
	static void eefOdometryCallback (const nav_msgs::Odometry &eefOdometry);
	static void referenceCallback (const geometry_msgs::Pose &ref);
	static void trajectoryCallback (const aerial_mpc::Trajectory &ref);
	static void obstacleCallback (const ground_mpc::ObstaclesArray &obstacle);
};

extern InputInterface *inputInterface;

#endif // CLOSED_LOOP_H
