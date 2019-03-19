#ifndef EASY_GRIPPER_H
#define EASY_GRIPPER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <thread>

#include "easy_gripper/GripCommand.h"
#include "easy_gripper/WrenchArray.h"


namespace gazebo {

enum GripperCommand {
	GRIPPER_ATTACH = 0,
	GRIPPER_DETACH,
	GRIPPER_PRINT
};

typedef std::pair<physics::EntityPtr, physics::EntityPtr> GraspPair;

struct Grasp {
	GraspPair graspPair;
	physics::JointPtr fixedJoint;

	bool operator == (const Grasp &b) {
		return graspPair == b.graspPair;
	}

	Grasp (const physics::EntityPtr &first, const physics::EntityPtr &second):
		graspPair (first, second)
	{}
};

class EasyGripper : public ModelPlugin
{
	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber rosSub;
	ros::Publisher wrenchPub;
	std::thread rosQueueThread;

	physics::ModelPtr model;
	std::vector<Grasp> grasps;

	void attach (const physics::EntityPtr &gripper,
				 const physics::EntityPtr &object);
	void detach (const physics::EntityPtr &gripper,
				 const physics::EntityPtr &object);

	void printGrasps ();
	void pubWrenches ();

public:
	EasyGripper ():
		ModelPlugin ()
	{}

	void Load (physics::ModelPtr parent, sdf::ElementPtr sdf);

	void onRosMsg (const easy_gripper::GripCommandConstPtr &msg);

private:
	void queueThread ();
};
}
#endif // EASY_GRIPPER_H
