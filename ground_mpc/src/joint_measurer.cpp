#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

using namespace ros;
using namespace std;
bool published;

geometry_msgs::Point getDispl (const gazebo_msgs::LinkStates &linkStates, int i, int j)
{
	geometry_msgs::Point first = linkStates.pose[i].position;
	geometry_msgs::Point second = linkStates.pose[j].position;
	geometry_msgs::Point ret;

	ret.x = second.x - first.x;
	ret.y = second.y - first.y;
	ret.z = second.z - first.z;

	return ret;
}

double getDist (const gazebo_msgs::LinkStates &linkStates, int i, int j)
{
	geometry_msgs::Point first = linkStates.pose[i].position;
	geometry_msgs::Point second = linkStates.pose[j].position;

	return sqrt (pow (first.x - second.x, 2) +
				 pow (first.y - second.y, 2) +
				 pow (first.z - second.z, 2));
}

void linkStatesCallback (const gazebo_msgs::LinkStates &linkStates)
{
	if (!published)
		published = true;
	else
		shutdown ();

	for (int i = 0; i < linkStates.name.size () - 1; i++) {
		ROS_INFO("%d: %s", i, linkStates.name[i].c_str());
	}

	ROS_INFO(" ");
	ROS_INFO("From %s to %s", linkStates.name[1].c_str(), linkStates.name[10].c_str());
	geometry_msgs::Point baseDisplacement = getDispl (linkStates, 1, 10);
	ROS_INFO("Displacement x: %lg, y: %lg, z: %lg\n", baseDisplacement.x, baseDisplacement.y, baseDisplacement.z);

	for (int i = 10; i < 15; i++) {
		ROS_INFO("From %s to %s", linkStates.name[i].c_str(), linkStates.name[i+1].c_str());
		ROS_INFO("Distance: %lg\n", getDist (linkStates, i, i+1));
	}

	ROS_INFO ("Wheels:");
	geometry_msgs::Point curr = getDispl (linkStates, 1, 9);
	ROS_INFO ("FR %lg %lg", curr.x, curr.y);
	curr = getDispl (linkStates, 1, 7);
	ROS_INFO ("FL %lg %lg", curr.x, curr.y);
	curr = getDispl (linkStates, 1, 5);
	ROS_INFO ("RR %lg %lg", curr.x, curr.y);
	curr = getDispl (linkStates, 1, 3);
	ROS_INFO ("FR %lg %lg", curr.x, curr.y);

	geometry_msgs::Point costum = getDispl (linkStates, 0, 25);

	ROS_INFO ("Wrist: %lg %lg %lg", costum.x, costum.y, costum.z);
}



int main (int argc, char *argv[])
{
	published = false;
	init (argc, argv, "joint_measurer");
	NodeHandle nh;
	Subscriber jointSub = nh.subscribe ("/gazebo/link_states", 1, linkStatesCallback);

	spin ();

	return 0;
}
