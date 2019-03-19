#include "ros/ros.h"
#include "std_msgs/String.h"

#include "mpc_wrapper/InputDataMsg.h"

using namespace ros;
using namespace std;

#include "acado_common.h"

int main(int argc, char **argv)
{
	init(argc, argv, "simple_publisher");
	NodeHandle nh;

	Publisher chatter_pub = nh.advertise<mpc_wrapper::InputDataMsg>("/mpc/input", 1000);

	Rate loop_rate(1);

	while (ros::ok())
	{
		mpc_wrapper::InputDataMsg msg;

		vector<double> stateData = {-0.0002,   -0.0000,    0.0003,    0.0450,   -0.6049,   -0.7950,    0.0589,   -0.7928,    0.6066,   -0.9972, -0.0741, 0.0, -0.8765, 0.7933, -1.0707, -0.6416, -0.3887, 5.2268, -0.6201};
		vector<double> refData = {2,   -1.0000,    0.0003,
								  0, 0, 0,
								  0, 0, 0,
								  0, 0, 0,
								 0, 0, 0 ,0, 0 ,0 ,0};
		vector<double> termData = {2,   -1.0000,    0.0003,
								   0, 0, 0,
								   0, 0, 0,
								   0, 0, 0};

		vector<double> onData = {0.0450,   -0.6049,   -0.7950,
								 0.0589,   -0.7928,    0.6066,
								 -0.9972,   -0.0741,         0};
		vector<double> onDataRep, refDataRep;

		refDataRep.resize ((ACADO_N) * ACADO_NY);
		onDataRep.resize ((ACADO_N + 1) * ACADO_NOD);

		for (int i= 0; i < ACADO_N; i++) {
			for (int j = 0; j < ACADO_NY; j++)
				refDataRep[i*ACADO_NY + j] = refData[j];
		}

		for (int i = 0; i < ACADO_N + 1; i++) {
			for (int j = 0; j < ACADO_NOD; j++)
				onDataRep[i*ACADO_NOD + j] = onData[j];
		}


		msg.state.layout.dim.resize (1);
		msg.state.layout.dim[0].size = 19;
		msg.state.data = stateData;

		msg.refWindow.layout.dim.resize (2);
		msg.refWindow.layout.dim[0].size = ACADO_NY;
		msg.refWindow.layout.dim[1].size = ACADO_N;
		msg.refWindow.data = refDataRep;

		msg.refTerminal.layout.dim.resize (1);
		msg.refTerminal.layout.dim[0].size = ACADO_NYN;
		msg.refTerminal.data = termData;

		msg.onlineData.layout.dim.resize (2);
		msg.onlineData.layout.dim[0].size = ACADO_NOD;
		msg.onlineData.layout.dim[1].size = ACADO_N + 1;
		msg.onlineData.data = onDataRep;
		chatter_pub.publish(msg);


		spinOnce();

		loop_rate.sleep();
	}

	return 0;
}
