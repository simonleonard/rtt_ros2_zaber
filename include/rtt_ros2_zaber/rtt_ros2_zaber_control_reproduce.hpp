#pragma once
#include <iostream>
#include <queue>
#include <vector>

#include "Eigen/Dense"
#include "rtt_ros2_zaber/auto_insertion_command.hpp"
#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"
#include "rtt_ros2_zaber/sg_filter.hpp"
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

    void printTipPosition() const;
    void printJacobian() const;

    void autoInsertion(const std::string& file);
    void reproduce(const std::string& experiment);

    enum class State { IDLE, DEMO, CONTROL };

   private:
    void collect_demo_points();
    void control_loop();

    State state;

    // Auto insertion.
    long insertion_start_time;
    std::queue<Command> insert_cmds;

    // Record target way points.
    double linear_stage_step;
    double prev_linear_stage_plane;

    double max_control_vel;

    // Reproduce control loop.
    bool ready_to_reproduce;
    WayPoint prev_wpt;
    Eigen::Matrix3d jacobian;

    std::vector<Eigen::Vector3d> target_trajectory;
    size_t target_index;

    std::vector<WayPoint> demo_trajectory;
    std::vector<WayPoint> mimic_trajectory;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    Eigen::Vector3d tip_position;
};

class DemoTrajCollector {
   public:
    DemoTrajCollector(int filter_window_size, int filter_order);
    std::vector<Eigen::Vector3d> get_target_traj() const;

   private:
    std::vector<Eigen::Vector3d> inputs;
    std::vector<Eigen::Vector3d> outputs;
    SavitzkyGolayFilter ndi_filter;
};

class ControlTrajTracker {
    public:
    private:
    std::vector<Eigen::Vector3d> inputs;
}

std::ostream& operator<<(std::ostream& os,
                         RttRos2ZaberControlReproduce::State s);