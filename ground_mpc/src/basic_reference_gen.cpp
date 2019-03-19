#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

using namespace ros;
using namespace std;

#define TS 0.1

#define SAMPLE_TO_SEC(k) (k/TS)

vector<geometry_msgs::Pose> initTraj ()
{
	vector<geometry_msgs::Pose> traj;

	traj.resize (SAMPLE_TO_SEC(30));
	for (int i= 0; i < traj.size (); i++) {
		geometry_msgs::Pose curr;

		curr.position.x = (i < SAMPLE_TO_SEC(2)? 0.3: 1);
		curr.position.y = (i < SAMPLE_TO_SEC(5)? 0: 0.2 * sin (1/(20 * M_PI) * (double)SAMPLE_TO_SEC(i)));
		curr.position.z = (i < SAMPLE_TO_SEC(5)? 0.4: 0.4 + 0.15 * cos (1/(20 * M_PI) * (double)SAMPLE_TO_SEC(i)));

		traj[i] = curr;
	}

	return traj;
}

int main (int argc, char *argv[])
{
	init (argc, argv,"basic_reference_gen");
	NodeHandle nh;
	Publisher pub = nh.advertise<geometry_msgs::Pose> ("/ground/cmd_pose/object_pose", 1);
	Rate rt(10);
	vector<geometry_msgs::Pose> trajectory = initTraj ();
	int i(0);

	for (i= 0; i < SAMPLE_TO_SEC(15); i++)
	{
		pub.publish (trajectory[i]);
		spinOnce ();
		rt.sleep ();
	}

}
