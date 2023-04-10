#pragma once
#include <iostream>
#include <queue>
#include <vector>

#include "Eigen/Dense"
#include "rtt_ros2_zaber/auto_insertion_command.hpp"
#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

struct WayPoint {
    Eigen::Vector3d input;
    Eigen::Vector3d output;
};

class RttRos2ZaberControlReproduce : public RttRos2ZaberBase {
   public:
    RttRos2ZaberControlReproduce(const std::string& name);

    bool configureHook() override;
    bool startHook() override;
    void updateHook() override;
    void stopHook() override;
    void cleanupHook() override;

    void autoInsertion(const std::string& file);
    void reproduce(const std::string& experiment);

    enum class State { IDLE, DEMO, CONTROL };

   private:
    void collect_demo_points();
    void control_loop();

    State state;

    long insertion_start_time;
    std::queue<Command> insert_cmds;

    double linear_stage_step;
    double next_linear_stage_plane;

    bool ready_to_reproduce;

    std::queue<Eigen::Vector3d> target_trajectory;
    std::vector<WayPoint> demo_trajectory;
    std::vector<WayPoint> mimic_trajectory;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped RxBaseTip;
};

std::ostream& operator<<(std::ostream& os,
                         RttRos2ZaberControlReproduce::State s);