#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <gazebo/physics/ode/ODEBallJoint.hh>

#include "easy_gripper.h"
#include "easy_gripper/GripCommand.h"

using namespace ros;
using namespace std;
using namespace gazebo;
using namespace physics;
using namespace ignition::math;


void EasyGripper::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	model = parent;

	FILE *fp = fopen ("/home/nicola/forceOut","w");
	fclose (fp);

	if (!ros::isInitialized ())
	{
		int argc = 0;
		char ** argv = NULL;

		ros::init (argc, argv,"gazebo_client");
	}

	rosNode = new NodeHandle ("gazebo_client");
	rosNode->setCallbackQueue (&rosQueue);

	SubscribeOptions so =
			SubscribeOptions::create<easy_gripper::GripCommand> ("/easy_gripper/grip_command", 1000,
																 boost::bind(&EasyGripper::onRosMsg, this, _1),
																 ros::VoidPtr(), &rosQueue);
	rosSub = rosNode->subscribe(so);
	wrenchPub = rosNode->advertise<easy_gripper::WrenchArray> ("/easy_gripper/wrenches", 1000);

	rosQueueThread = thread (bind (&EasyGripper::queueThread, this));
}

void EasyGripper::queueThread() {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		pubWrenches ();
		rosQueue.callAvailable (WallDuration(timeout));
	}
}

void EasyGripper::pubWrenches ()
{
	const int count = grasps.size ();
	easy_gripper::WrenchArray msg;

	msg.wrenches.resize (count);
	msg.parent_names.resize (count);
	msg.child_names.resize (count);

	for (int i = 0; i < count; i++) {
		ODEJointPtr odeJoint = boost::dynamic_pointer_cast<ODEJoint> (grasps[i].fixedJoint);

		odeJoint->SetProvideFeedback (true);
		Vector3d force = odeJoint->GetForceTorque (0).body1Force;
		Vector3d torque = odeJoint->GetForceTorque (0).body1Torque;
		Pose3d pos = odeJoint->GetParent ()->DirtyPose ();

		msg.wrenches[i].force.x = force[0];
		msg.wrenches[i].force.y = force[1];
		msg.wrenches[i].force.z = force[2];

		msg.wrenches[i].torque.x = torque[0];
		msg.wrenches[i].torque.y = torque[1];
		msg.wrenches[i].torque.z = torque[2];

		msg.parent_names[i] = grasps[i].graspPair.first->GetName ();
		msg.child_names[i] = grasps[i].graspPair.second->GetName ();
		if (i == 1) {
			FILE *fp = fopen ("/home/nicola/forceOut","a");
			fprintf (fp, "%lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg, %lg\n",
					 pos.Pos ()[0], pos.Pos ()[1], pos.Pos ()[2],
					 force[0], force[1], force[2], torque[0], torque[1], torque[2]);
			fclose (fp);
		}
	}


	wrenchPub.publish (msg);
}

void EasyGripper::attach (const EntityPtr &gripper, const EntityPtr &object)
{
	// Check whether a grasp roselement already exists
	Grasp currGrasp(gripper, object);

	auto result = find (grasps.begin (), grasps.end (), currGrasp);

	if (result != grasps.end ()) {
		ROS_INFO ("Attempting to reattach an existing grasp. Skipping");
		return;
	}


	// Create fixed joint
	Pose3d diff = object->WorldPose () - gripper->WorldPose ();
	LinkPtr gripperLink = boost::dynamic_pointer_cast<Link> (gripper);
	LinkPtr objectLink =  boost::dynamic_pointer_cast<Link> (object);

	PhysicsEnginePtr physics = model->GetWorld ()->Physics ();

	currGrasp.fixedJoint = physics->CreateJoint ("fixed");


	currGrasp.fixedJoint->Load (gripperLink, objectLink, diff);
	currGrasp.fixedJoint->SetParent (gripperLink);
	currGrasp.fixedJoint->AddChild (objectLink);
	currGrasp.fixedJoint->Init ();
	ODEJointPtr odeJoint = boost::dynamic_pointer_cast<ODEJoint> (currGrasp.fixedJoint);
	odeJoint->SetCFM (100.10);

	grasps.push_back (currGrasp);
	ROS_INFO ("Done");
}

void EasyGripper::detach (const EntityPtr &gripper, const EntityPtr &object)
{
	// Check whether a grasp element already exists
	Grasp findGrasp(gripper, object);
	vector<Grasp>::iterator result = find (grasps.begin (), grasps.end (), findGrasp);

	if (result == grasps.end ()) {
		ROS_INFO ("Attempting to detach an unexisting grasp. Skipping");
		return;
	}

	result->fixedJoint->Detach ();
	result->fixedJoint->RemoveChildren ();

	grasps.erase (result);

	ROS_INFO ("Done");
}

void EasyGripper::printGrasps ()
{
	if (grasps.size() > 0)
		ROS_INFO ("Active grasps:");
	else
		ROS_INFO ("No active grasps.");

	for (int i = 0; i < grasps.size (); i++) {
		Grasp curr = grasps[i];

		ROS_INFO ("%d: %s -- %s", i, curr.graspPair.first->GetName ().c_str (), curr.graspPair.second->GetName ().c_str ());
	}
}

void EasyGripper::onRosMsg(const easy_gripper::GripCommandConstPtr &msg)
{
	WorldPtr world = model->GetWorld ();
	EntityPtr gripper, object;

	if (world.get() == NULL) {
		gzerr << "EasyGripper: world is NULL" << endl;
		return;
	}

	ROS_INFO ("EasyGripper: Grasp from %s to %s", msg->gripper_link.c_str(), msg->object_link.c_str());

	gripper = world->EntityByName (msg->gripper_link);

	if (gripper.get() == NULL) {
		ROS_WARN ("EasyGripper: entity %s does not exists", msg->gripper_link.c_str());
		return;
	}

	object = world->EntityByName (msg->object_link);

	if (object.get () == NULL) {
		ROS_WARN ("EasyGripper: entity %s does not exist", msg->gripper_link.c_str());
		return;
	}

	switch (msg->command)
	{
	case GRIPPER_ATTACH:
		ROS_INFO ("Attaching");
		attach (gripper, object);
		break;
	case GRIPPER_DETACH:
		ROS_INFO ("Detaching");
		detach (gripper, object);
		break;
	case GRIPPER_PRINT:
		printGrasps ();
		break;
	default:
		ROS_INFO ("Invalid command");
		break;
	}
}

GZ_REGISTER_MODEL_PLUGIN(EasyGripper)























