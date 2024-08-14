#include <QWidget>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.h>
#include <std_srvs/srv/trigger.h>

#include "waypoint_operator_panel.hpp"

namespace waypoint_visualization {
    WaypointOperatorPanel::WaypointOperatorPanel(QWidget *parent)
        : rviz_common::Panel(parent),
         node_(rclcpp::Node::make_shared("waypoint_operator_panel"))
    {
        hbox_layout = new QHBoxLayout();

        switch_cancel_button = new QPushButton(this);
        switch_cancel_button->setText("Switch Cancel");
        connect(switch_cancel_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callSwitchCancel);
        switch_cancel_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        next_waypoint_button = new QPushButton(this);
        next_waypoint_button->setText("Next Waypoint");
        connect(next_waypoint_button, &QPushButton::clicked, this, &WaypointOperatorPanel::callNextWaypoint);
        next_waypoint_button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        hbox_layout->addWidget(switch_cancel_button);
        hbox_layout->addWidget(next_waypoint_button);

        setLayout(hbox_layout);

        switch_cancel_client = node_->create_client<std_srvs::srv::Trigger>("waypoint_manager/waypoint_server/switch_cancel");
        next_waypoint_client = node_->create_client<std_srvs::srv::Trigger>("waypoint_manager/waypoint_server/next_waypoint");
}

    WaypointOperatorPanel::~WaypointOperatorPanel() {
    }

    // switch cancelを伝えるサービス
    void WaypointOperatorPanel::callSwitchCancel() {
        RCLCPP_INFO(node_->get_logger(), "Pushed callSwitchCancel()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        switch_cancel_client->async_send_request(request);
    }

    // next waypointを伝えるサービス
    void WaypointOperatorPanel::callNextWaypoint() {
        RCLCPP_INFO(node_->get_logger(), "Pushed callNextWaypoint()");
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        next_waypoint_client->async_send_request(request);        
    }

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    waypoint_visualization::WaypointOperatorPanel,
    rviz_common::Panel
)