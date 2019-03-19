#include "aerial_widowx_joint_state.h"

using namespace ros;
using namespace std;
using namespace XmlRpc;

WidowxJointState::WidowxJointState ():
    currState(JOINT_MSG_N, 0),
    pub(false)
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
    if (!pub)
        return;

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
    pub = false;
    for (int i = 0; i < JOINT_N; i++) {
        const int curr = distance (jointState.name.begin (),
                                   find (jointState.name.begin (),
                                         jointState.name.end (),
                                         string (getJointName (i))));
        if (curr <= JOINT_N) {
            currState[i] = jointState.position[curr];
            pub = true;
        }
    }
}

int main (int argc, char *argv[])
{
    init (argc, argv, "widowx_joint_state");
    WidowxJointState widowxJointState;

    return widowxJointState.spin ();
}
