#include "../include/prediction_printer.h"

using namespace gazebo;
using namespace gazebo::rendering;
using namespace physics;
using namespace ignition::math;
using namespace ros;
using namespace std;

#define DEFAULT_HISTORY_COUNT 10

void PredictionPrinter::Load (VisualPtr _visual, sdf::ElementPtr sdf)
{
	visual = _visual;

	if (!processSdf (sdf))
		return;

	// Start ros node
	if (!isInitialized ())
	{
		int argc = 0;
		char **argv = NULL;
		ros::init (argc, argv, "prediction_printer");

	}

	rosNode = new NodeHandle ("gazebo_client");
	rosNode->setCallbackQueue (&rosQueue);
	SubscribeOptions so =
			SubscribeOptions::create<mpc_wrapper::OutputDataMsg> (topicName, 1,
																  boost::bind(&PredictionPrinter::pathCallback, this, _1),
																  ros::VoidPtr(), &rosQueue);

	rosSub = rosNode->subscribe (so);
	rosQueueThread = thread (bind (&PredictionPrinter::queueThread, this));
	updateConnection = event::Events::ConnectRender (
				boost::bind(&PredictionPrinter::UpdateChild, this));
}


bool PredictionPrinter::processSdf (sdf::ElementPtr sdf)
{
	if (!sdf->HasElement ("topic")) {
		gzerr << "Missing sdf element <topic>. Plugin will not work." << endl;
		return false;
	}
	topicName = sdf->GetElement ("topic")->GetValue ()->GetAsString ();
	sdf->Get<int> ("history_count", linesCount, DEFAULT_HISTORY_COUNT);

	if (!sdf->HasElement ("config_data")) {
		gzerr << "Missing sdf element <config_data>. Plugin will not work." << endl;
	}
	sdf::ElementPtr config = sdf->GetElement ("config_data");
	if (!config->HasElement ("path")) {
		gzerr << "Missing sdf element <path>. Plugin will not work." << endl;
		return false;
	}
	sdf::ElementPtr currPath = config->GetElement ("path");

	do {
		if (!currPath->HasAttribute ("name")) {
			gzerr << "Missing attribute \"name\" to element <path>. Plugin will not work." << endl;
			return false;
		}
		string pathName = currPath->GetAttribute ("name")->GetAsString ();
		PathData pathData;
		const char *pathValuesStr = currPath->GetValue ()->GetAsString ().c_str ();
		sscanf (pathValuesStr,"%d %d %d", &pathData.dataIndexes[0], &pathData.dataIndexes[1], &pathData.dataIndexes[2]);

		if (currPath->HasAttribute ("color"))
			currPath->GetAttribute ("color")->Get (pathData.color);
		// Check defaults
		if (currPath->HasAttribute ("x"))
			currPath->GetAttribute ("x")->Get (pathData.defaultValues[0]);
		if (currPath->HasAttribute ("y"))
			currPath->GetAttribute ("y")->Get (pathData.defaultValues[1]);
		if (currPath->HasAttribute ("z"))
			currPath->GetAttribute ("z")->Get (pathData.defaultValues[2]);

		linesMap[pathName] = pathData;

		currPath = currPath->GetNextElement ("path");
	} while (currPath != sdf::ElementPtr(nullptr));

	return true;
}

void PredictionPrinter::pathCallback (const mpc_wrapper::OutputDataMsgConstPtr &newTrajectory) {
	mpcData = *newTrajectory;
}

void PredictionPrinter::queueThread () {
	static const double timeout = 0.01;
	while (rosNode->ok ()) {
		rosQueue.callAvailable (WallDuration(timeout));
	}
}

void PredictionPrinter::setMpcParams ()
{
	samples = mpcData.stateTrajectory.layout.dim[1].size;
	states = mpcData.stateTrajectory.layout.dim[0].size;
}

void PredictionPrinter::UpdateChild () {
	if (mpcData.stateTrajectory.layout.dim.size () < 2)
		return;
	setMpcParams ();
	drawLine ();
}

double PredictionPrinter::getValue (const PathData &data, int i, int j)
{
	if (data.dataIndexes[j] >= 0) {
		return mpcData.stateTrajectory.data[i * states + data.dataIndexes[j]];
	}
	else {
		return data.defaultValues[j];
	}
}

Vector3d PredictionPrinter::getTrajectoryPoint (const PathData &data, int i)
{
	return Vector3d (getValue (data, i, 0),
					 getValue (data, i, 1),
					 getValue (data, i, 2));
}

void PredictionPrinter::drawLine ()
{
	for (LinesMap::iterator it = linesMap.begin (); it != linesMap.end (); it++) {
		string name = it->first;
		PathData &curr = it->second;

		if (curr.lines.size () >= linesCount) {
			visual->DeleteDynamicLine (curr.lines.front ());
			curr.lines.pop ();
		}

		DynamicLines *newLine = visual->CreateDynamicLine ();

		for (int i = 0; i < samples; i++) {
			newLine->AddPoint (getTrajectoryPoint (curr, i));
		}

		newLine->setMaterial (string("Gazebo/") + curr.color);
		newLine->setVisibilityFlags (GZ_VISIBILITY_ALL);
		curr.lines.push (newLine);
	}

	visual->SetVisible (true);
}

GZ_REGISTER_VISUAL_PLUGIN(PredictionPrinter)






























