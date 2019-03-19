#include <functional>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

namespace gazebo
{

class SystemGUI : public SystemPlugin
{
	std::vector<event::ConnectionPtr> connections;

public:
	virtual ~SystemGUI () {
		connections.clear();
	}

	void Load (int argc, char **argv) {

	}
}

}
