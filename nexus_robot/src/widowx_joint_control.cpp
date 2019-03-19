#include "widowx_joint_control.h"

using namespace ros;
using namespace std;

extern WidowXJointControl *node;

WidowXJointControl::WidowXJointControl():
	rate(WIDOWX_INTERFACE_RATE)
{
	arrayControlSub = nh.subscribe (WIDOWX_JOINT_SUB, 1, arrayControlCallback);

	initJointsPublishers ();
	jointControlValues.resize (WIDOWX_JOINT_NO);
}

string WidowXJointControl::getTopicName (int i)
{
	stringstream topicName;
	topicName << WIDOWX_JOINT_PUB_PRE << i+1 << WIDOWX_JOINT_PUB_POST;

	return topicName.str();
}

void WidowXJointControl::initJointsPublishers ()
{
	jointsPub.resize (WIDOWX_JOINT_NO);
	for (int i= 0; i < WIDOWX_JOINT_NO; i++) {
		string currPubName = getTopicName (i);
		jointsPub[i] = nh.advertise<SingleControlMsg> (currPubName, 1);
	}
}

void WidowXJointControl::publishAll ()
{
	for (int i= 0; i < WIDOWX_JOINT_NO; i++) {
		SingleControlMsg currMsg;

		currMsg.data = jointControlValues[i];
		jointsPub[i].publish (currMsg);
	}
}

int WidowXJointControl::spin()
{
	while (ros::ok ()) {

		publishAll ();
		spinOnce ();
		rate.sleep ();
	}
}

void WidowXJointControl::setValues (std::vector<float> values) {
	jointControlValues = values;
}

void WidowXJointControl::arrayControlCallback(const ArrayControlMsg &newControlMsg)
{
	vector<float> newControl;

	newControl = newControlMsg.data;

	node->setValues (newControl);
}














