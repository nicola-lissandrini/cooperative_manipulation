#ifndef PREDICTION_PRINTER_H
#define PREDICTION_PRINTER_H

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/JointVisual.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <thread>
#include <map>

#include "aerial_mpc/Trajectory.h"
#include "mpc_wrapper/OutputDataMsg.h"

namespace gazebo {

namespace rendering {
struct PathData {
	int count;
	std::queue<DynamicLines *> lines;
	ignition::math::Vector3i dataIndexes;
	ignition::math::Vector3d defaultValues;
	std::string color;

	PathData():
		count(0),
		defaultValues(NAN, NAN, NAN),
		color("Black")
	{}

	PathData(const PathData &b):
		lines(b.lines),
		count(b.count),
		color(b.color),
		dataIndexes(b.dataIndexes),
		defaultValues(b.defaultValues)
	{}
};

typedef std::map <std::string, PathData> LinesMap;

class PredictionPrinter : public VisualPlugin
{
	ros::NodeHandle *rosNode;
	ros::CallbackQueue rosQueue;
	ros::Subscriber rosSub;
	std::thread rosQueueThread;
	rendering::VisualPtr visual;
	ScenePtr scene;
	LinesMap linesMap;
	mpc_wrapper::OutputDataMsg mpcData;


	event::ConnectionPtr updateConnection;

	void drawLine ();
	void queueThread ();

	// Params
	int linesCount;
	std::string topicName;

	// MPC Output params
	int samples, states;

	void setMpcParams ();
	bool processSdf (sdf::ElementPtr sdf);
	ignition::math::Vector3d getTrajectoryPoint(const PathData &data, int i);
	double getValue(const PathData &data, int i, int j);

protected:
	virtual void UpdateChild ();

public:
	PredictionPrinter ():
		VisualPlugin ()
	{}

	void Load (VisualPtr _visual, sdf::ElementPtr _sdf);
	void pathCallback (const mpc_wrapper::OutputDataMsgConstPtr &newTrajectory);
};

}
}

#endif // PREDICTION_PRINTER_H
