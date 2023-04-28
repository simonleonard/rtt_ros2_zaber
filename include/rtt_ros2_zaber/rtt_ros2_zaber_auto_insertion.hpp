#pragma once

#include "needle_steering_control_demo_msgs/msg/control_demo_point.hpp"
#include "rtt_ros2_zaber/auto_insertion_command.hpp"
#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"

class RttRos2ZaberAutoInsertion : public RttRos2ZaberBase {
   public:
    RttRos2ZaberAutoInsertion(const std::string& name);

    bool configureHook() override;
    bool startHook() override;
    void updateHook() override;
    void stopHook() override;
    void cleanupHook() override;

    void autoInsertion(const std::string& file);

   private:
    RTT::OutputPort<needle_steering_control_demo_msgs::msg::ControlDemoPoint>
        port_demo_point_;

    long insertion_start_time_;
    std::queue<Command> insert_cmds_;
};