// rosmon GUI
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>

#include <pluginlib/class_list_macros.hpp>

namespace imgui_ros
{

class GUI : public imgui_ros::Window
{
public:
    void paint() override
    {

    }
};

}

PLUGINLIB_EXPORT_CLASS(imgui_ros::GUI, imgui_ros::Window)
