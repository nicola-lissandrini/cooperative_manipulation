#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace ros;

int main (int argc, char *argv[])
{
	init (argc, argv, "simple_multi_pub");
	NodeHandle nh;
	Publisher pub = nh.advertise<std_msgs::Float32MultiArray> ("/ground_widowx_control/joints_control", 1);
	Rate r(2);

	while (ros::ok ())
	{
		std_msgs::Float32MultiArray msg;

		msg.data.clear ();
		msg.data.push_back (1);
		msg.data.push_back (1);
		msg.data.push_back (1);
		msg.data.push_back (1);
		msg.data.push_back (1);

		pub.publish (msg);
		spinOnce ();
		r.sleep ();
	}

}
