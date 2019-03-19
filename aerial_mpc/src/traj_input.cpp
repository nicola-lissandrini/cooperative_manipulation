#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <boost/date_time.hpp>

using namespace ros;
using namespace std;
#define TRAJ
#ifdef TRAJ
#define TS 0.01

#define SAMPLE_TO_SEC(k) (double(k)*TS)
#define SEC_TO_SAMPLE(k) (double(k)/TS)

vector<geometry_msgs::Pose> initTraj ()
{
	vector<geometry_msgs::Pose> traj;

	traj.resize (SEC_TO_SAMPLE(30));
	for (int i= 0; i < traj.size (); i++) {
		geometry_msgs::Pose curr;

		curr.orientation.x = -0.707;
		curr.orientation.y = 0;
		curr.orientation.z = 0;
		curr.orientation.w = 0.707;

		if (SAMPLE_TO_SEC(i) < 5) {
			curr.position.x = 1;
		}
		else {
			if (SAMPLE_TO_SEC(i) < 10)
				curr.position.x = -1;
			else {
					curr.position.x = 0.5 * cos (2 * M_PI / 6 * (double)SAMPLE_TO_SEC(i)); // +
						//	0.5 * cos (2 * M_PI / 3 * (double)SAMPLE_TO_SEC(i)) +
						//	0.5 * cos (2 * M_PI / 9 * (double)SAMPLE_TO_SEC(i));
					curr.position.y = 0.5 * sin (2 * M_PI / 6 * (double)SAMPLE_TO_SEC(i));
			}
		}
		curr.position.z = 1;

		traj[i] = curr;
	}

	return traj;
}


geometry_msgs::Pose lastPose;
void writeOdom (const nav_msgs::Odometry &odom)
{
	FILE *fp = fopen ("/home/nicola/odomOut","a");
	FILE *fp2 = fopen ("/home/nicola/expInput","a");
	fprintf (fp,"%lg, %lg, %lg\n", odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
	fprintf (fp2,"%lg, %lg, %lg\n", lastPose.position.x, lastPose.position.y, lastPose.position.z);
	fclose (fp);
	fclose (fp2);
}

int main (int argc, char *argv[])
{
	init (argc, argv, "traj_input");

	NodeHandle nh;
	Publisher trajPub = nh.advertise<geometry_msgs::Pose> ("/neo11/command/object_pose", 1);
	Subscriber odomSub = nh.subscribe ("/neo11/odometry_sensor1/odometry", 1, writeOdom);
	Rate rt(100);
	int i(0);
	vector<geometry_msgs::Pose> trajectory = initTraj ();

	FILE *fp = fopen ("/home/nicola/odomOut","w");
	FILE *fp2 = fopen ("/home/nicola/expInput","w");
	fclose (fp);
	fclose (fp2);

	for (i= 0; i < SEC_TO_SAMPLE(30); i++)
	{
		geometry_msgs::Pose msg;
		msg = trajectory[i];
		ROS_INFO ("x %lg", msg.position.x);
		trajPub.publish (msg);
		spinOnce ();
		rt.sleep ();
	}
}

#elif defined SETPTS
Time startTime;
int i;
vector<geometry_msgs::Twist> vels;
geometry_msgs::Twist curr;
Publisher velPub;

bool started(false);
geometry_msgs::Pose lastPose;
void writeOdom (const nav_msgs::Odometry &odom)
{
	if (!started)
		return;
	FILE *fp = fopen ("/home/nicola/velOut","a");
	fprintf (fp,"%lg, %lg, %lg, %lg, %lg, %lg, %lg\n", odom.twist.twist.linear.x,
			 odom.twist.twist.linear.y, odom.twist.twist.linear.z,
			 (Time::now () - startTime).toSec (),
			 curr.linear.x, curr.linear.y, curr.linear.z);
	fclose (fp);
}


void timerCallback (const TimerEvent &e)
{
	switch (i) {
	case 2:
		ROS_INFO ("Take off");
		started = true;
		startTime = Time::now();
		curr = vels[0];
		velPub.publish (curr);
		break;
	case 4:
	case 9:
	case 13:
		ROS_INFO ("Stop");
		curr = vels[1];
		velPub.publish (curr);
		break;
	case 6:
	case 14:
		ROS_INFO ("Move on x");
		curr = vels[2];
		velPub.publish (curr);
		break;
	case 10:
		ROS_INFO ("Move back on x");
		curr = vels[3];
		velPub.publish (curr);
		break;
	default:
		break;
	}
	i++;
}

int main (int argc, char *argv[])
{
	init (argc, argv, "traj_input");

	NodeHandle nh;
	velPub = nh.advertise<geometry_msgs::Twist> ("/neo11/command/object_pose", 1);
	Subscriber odomSub = nh.subscribe ("/neo11/odometry_sensor1/odometry", 1, writeOdom);


	while (ok) {
		spinOnce ();
		r.sleep ();
	}

	return 0;
}
#endif




























