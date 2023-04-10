#pragma once

#include "needle_steering_control_demo_msgs/msg/control_demo_point.hpp"
#include "rtt_ros2_zaber/auto_insertion_command.hpp"
#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
        portDemoPoint;

    long insertion_start_time;
    std::queue<Command> insert_cmds;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};