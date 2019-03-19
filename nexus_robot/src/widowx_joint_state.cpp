#include <ros/ros.h>
#include <XmlRpc.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/JointState.h>

using namespace ros;
using namespace std;
using namespace XmlRpc;

#define JOINT_N 4
#define JOINT_MSG_N 6

class WidowxJointState
{
	NodeHandle nh;
	Rate *rate;
	Subscriber jointsSub;
	Publisher jointsPub;
	XmlRpcValue params;
	vector<float> currState;

	void initROS ();
	void initParams ();
	void sendMsg ();
	string getJointName (int i);

public:
	WidowxJointState();

	int spin ();
	void jointMsgCallback (const sensor_msgs::JointState &jointMsg);
};

WidowxJointState::WidowxJointState ():
	currState(JOINT_MSG_N, 0)
{
	initParams ();
	initROS ();
}

int WidowxJointState::spin()
{
	while (ok ()) {
		sendMsg ();
		spinOnce ();
		rate->sleep ();
	}

	return 0;
}

void WidowxJointState::initParams ()
{
	try {
		nh.getParam ("widowx_joint_state", params);
	} catch (const XmlRpcException &e) {
		ROS_ERROR ("Error loading params: %s", e.getMessage ().c_str ());
	}
}

void WidowxJointState::sendMsg()
{
	std_msgs::Float32MultiArray jointMsg;
	jointMsg.layout.dim.resize (1);
	jointMsg.layout.dim[0].size = JOINT_MSG_N;
	jointMsg.layout.dim[0].stride = JOINT_MSG_N;
	jointMsg.data.resize(JOINT_MSG_N);
	jointMsg.data = currState;

	jointsPub.publish(jointMsg);
}

string WidowxJointState::getJointName(int i) {
	return (string) params["joints_names"][i];
}

void WidowxJointState::initROS ()
{
	rate = new Rate (double (params["rate"]));

	jointsSub = nh.subscribe ((string) params["widowx_joint_topic_sub"], 1, &WidowxJointState::jointMsgCallback, this);
	jointsPub = nh.advertise<std_msgs::Float32MultiArray> ((string) params["widowx_joint_topic_pub"], 1);
}

void WidowxJointState::jointMsgCallback (const sensor_msgs::JointState &jointState)
{
	for (int i = 0; i < JOINT_N; i++) {
		const int curr = distance (jointState.name.begin (),
								   find (jointState.name.begin (),
										 jointState.name.end (),
										 string (getJointName (i))));
		if (curr <= jointState.position.size())
			currState[i] = jointState.position[curr];
	}
}

int main (int argc, char *argv[])
{
	init (argc, argv, "widowx_joint_state");
	WidowxJointState widowxJointState;

	return widowxJointState.spin ();
}
