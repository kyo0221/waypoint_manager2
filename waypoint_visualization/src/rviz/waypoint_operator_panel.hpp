#pragma once

#include <QWidget>
#include <QHBoxLayout>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

namespace waypoint_visualization {
    class WaypointOperatorPanel : public rviz_common::Panel {
        Q_OBJECT

        public:
            WaypointOperatorPanel(QWidget *parent = nullptr);
            virtual ~WaypointOperatorPanel();

        protected Q_SLOTS:
            void callSwitchCancel();
            void callNextWaypoint();

        protected:
            QHBoxLayout *hbox_layout;

            QPushButton *switch_cancel_button;
            QPushButton *next_waypoint_button;

            rclcpp::Node::SharedPtr node_;

            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr switch_cancel_client;
            rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr next_waypoint_client;
    };
}