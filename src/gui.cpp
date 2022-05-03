// rosmon GUI
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <imgui_ros/window.h>
#include <imgui_ros/topic_selector.h>
#include <imgui_ros/box.h>
#include <imgui_ros/imgui/imgui.h>

#include <rosfmt/full.h>

#include <pluginlib/class_list_macros.hpp>

#include <rosmon_msgs/State.h>
#include <rosmon_msgs/StartStop.h>

#include <ros/service.h>

namespace imgui_rosmon
{

namespace
{
    static inline uint countLeadingZeroBits(std::uint64_t v)
    {
        return __builtin_clzll(v);
    }

    std::string formattedDataSize(std::uint64_t bytes, int precision)
    {
        int power, base = 1000;
        if (!bytes) {
            power = 0;
        } else { // Compute log2(bytes) / 10:
            power = int((63 - countLeadingZeroBits(bytes)) / 10);
            base = 1024;
        }

        const char* unit = "?";
        if (power > 0)
        {
            switch(power)
            {
                case 1: unit = "KiB"; break;
                case 2: unit = "MiB"; break;
                case 3: unit = "GiB"; break;
                case 4: unit = "TiB"; break;
                case 5: unit = "PiB"; break;
                case 6: unit = "EiB"; break;
            }
        }
        else
            unit = "B";

        double val = power ? (bytes / std::pow(double(base), power)) : bytes;
        return fmt::format("{:.{}f} {}", val, precision, unit);
    }
}

class GUI : public imgui_ros::Window
{
public:
    void paint() override
    {
        ImGui::SetNextItemWidth(-FLT_MIN);
        if(m_topicSelector.draw("##Topic", &m_topic))
            subscribe();

        if(m_stamp == ros::Time(0))
        {
            ImGui::TextUnformatted("No message");
            return;
        }

        ImGui::BeginDisabled(ros::Time::now() - m_stamp > ros::Duration(5.0));

        auto flags = ImGuiTableFlags_BordersV | ImGuiTableFlags_BordersOuterH | ImGuiTableFlags_Resizable | ImGuiTableFlags_RowBg | ImGuiTableFlags_NoBordersInBody | ImGuiTableFlags_Hideable;
        if(ImGui::BeginTable("table", 5, flags, {-1.0f, -1.0f}))
        {
            ImGui::TableSetupColumn("Node");
            ImGui::TableSetupColumn("State");
            ImGui::TableSetupColumn("#Restarts");
            ImGui::TableSetupColumn("CPU Load");
            ImGui::TableSetupColumn("Memory");
            ImGui::TableHeadersRow();

            for(auto& node : m_nodes)
            {
                ImGui::PushID(node.fullName.c_str());

                ImGui::TableNextRow();

                std::uint32_t color = [&]() -> std::uint32_t {
                    switch(node.nodeState.state)
                    {
                        case rosmon_msgs::NodeState::RUNNING:   return 0;
                        case rosmon_msgs::NodeState::IDLE:      return IM_COL32(128,128,128, 128);
                        case rosmon_msgs::NodeState::CRASHED:   return IM_COL32(255,0,0,255);
                        case rosmon_msgs::NodeState::WAITING:   return IM_COL32(128,255,255,255);
                    }
                    return 0;
                }();
                if(color != 0)
                    ImGui::TableSetBgColor(ImGuiTableBgTarget_RowBg0, color);


                ImGui::TableNextColumn();
                ImGui::Selectable(node.fullName.c_str(), false, ImGuiSelectableFlags_SpanAllColumns);
                if(ImGui::BeginPopupContextItem("context"))
                {
                    if(ImGui::Selectable("Start"))
                    {
                        sendCommand(node, rosmon_msgs::StartStop::Request::START);
                        ImGui::CloseCurrentPopup();
                    }
                    if(ImGui::Selectable("Stop"))
                    {
                        sendCommand(node, rosmon_msgs::StartStop::Request::STOP);
                        ImGui::CloseCurrentPopup();
                    }
                    if(ImGui::Selectable("Restart"))
                    {
                        sendCommand(node, rosmon_msgs::StartStop::Request::RESTART);
                        ImGui::CloseCurrentPopup();
                    }

                    ImGui::EndPopup();
                }

                if(ImGui::BeginPopupModal("Service Failure", nullptr, ImGuiWindowFlags_NoResize))
                {
                    ImGui::TextUnformatted("Could not call start_stop service");
                    if(ImGui::Button("OK", {120.0f, 0.0f}))
                        ImGui::CloseCurrentPopup();
                    ImGui::EndPopup();
                }

                ImGui::TableNextColumn();
                switch(node.nodeState.state)
                {
                    case rosmon_msgs::NodeState::RUNNING: ImGui::TextUnformatted("RUNNING");  break;
                    case rosmon_msgs::NodeState::IDLE:    ImGui::TextUnformatted("IDLE");     break;
                    case rosmon_msgs::NodeState::CRASHED: ImGui::TextUnformatted("CRASHED");  break;
                    case rosmon_msgs::NodeState::WAITING: ImGui::TextUnformatted("WAITING");  break;
                }

                ImGui::TableNextColumn();
                ImGui::Text("%d", node.nodeState.restart_count);

                ImGui::TableNextColumn();
                ImGui::Text("%.2f", node.nodeState.user_load + node.nodeState.system_load);

                ImGui::TableNextColumn();
                ImGui::TextUnformatted(node.memory.c_str());

                ImGui::PopID();
            }

            ImGui::EndTable();
        }

        ImGui::EndDisabled();
    }

    imgui_ros::Settings getState() const override
    {
        return {
            {"topic", m_topic}
        };
    }

    void setState(const imgui_ros::Settings & settings) override
    {
        if(auto var = settings.get("topic"))
        {
            m_topic = *var;
            subscribe();
        }
    }

private:
    void subscribe()
    {
        m_sub = {};
        m_nodes.clear();
        m_stamp = {};

        if(m_topic.empty())
            return;

        constexpr const char* suffix = "/state";
        if(m_topic.size() < strlen(suffix) || m_topic.substr(m_topic.size() - strlen(suffix)) != suffix)
        {
            ROS_ERROR("Invalid topic '%s'", m_topic.c_str());
            return;
        }

        m_base = m_topic.substr(0, m_topic.size() - strlen(suffix));
        m_sub = context()->nodeHandle().subscribe(m_topic, 1, &GUI::handleState, this);
    }

    void handleState(const rosmon_msgs::State& state)
    {
        m_nodes.clear();
        m_nodes.reserve(state.nodes.size());

        for(auto& n : state.nodes)
            m_nodes.emplace_back(n);

        std::sort(m_nodes.begin(), m_nodes.end());

        m_stamp = state.header.stamp;
    }

    struct Node
    {
        explicit Node(const rosmon_msgs::NodeState& st)
         : nodeState{st}, fullName{st.ns + "/" + st.name}
         , memory{formattedDataSize(st.memory, 2)}
        {}

        bool operator<(const Node& other) const
        { return fullName < other.fullName; }

        rosmon_msgs::NodeState nodeState;
        std::string fullName;
        std::string memory;
    };

    void sendCommand(const Node& n, int action)
    {
        rosmon_msgs::StartStop srv;
        srv.request.action = action;
        srv.request.node = n.nodeState.name;
        srv.request.ns = n.nodeState.ns;
        if(!ros::service::call(m_base + "/start_stop", srv))
        {
            ImGui::OpenPopup("Service Failure");
        }
    }

    imgui_ros::TopicSelector m_topicSelector{{"rosmon_msgs/State"}};
    std::string m_topic;
    std::string m_base;

    std::vector<Node> m_nodes;
    ros::Time m_stamp;

    ros::Subscriber m_sub;
};

}

PLUGINLIB_EXPORT_CLASS(imgui_rosmon::GUI, imgui_ros::Window)
