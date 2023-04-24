#pragma once
#include <iostream>
#include <queue>
#include <vector>

#include "rtt_ros2_zaber/auto_insertion_command.hpp"
#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"
#include "rtt_ros2_zaber/sg_filter.hpp"
#include "rtt_ros2_zaber/traj_collector.hpp"

class RttRos2ZaberControlReproduce : public RttRos2ZaberBase {
   public:
    RttRos2ZaberControlReproduce(const std::string& name);

    bool configureHook() override;
    bool startHook() override;
    void updateHook() override;
    void stopHook() override;
    void cleanupHook() override;

    void printJacobian() const;

    void autoInsertion(const std::string& file);
    void reproduce(const std::string& experiment);

    enum class State { IDLE, DEMO, CONTROL };

   private:
    void collect_demo_points();

    bool safety_check();
    void update_jacobian();
    void control_loop();

    State state_;

    // Auto insertion.
    long insertion_start_time_;
    std::queue<Command> insert_cmds_;

    // Record target way points.
    TrajCollector demo_traj_;
    std::unique_ptr<TrajCollectorIterator> demo_traj_itr_;

    // Reproduce control loop.
    bool ready_to_reproduce_;
    TrajCollector reproduce_traj_;

    bool sg_filtering_;
    std::unique_ptr<SavitzkyGolayFilter> filter;

    Eigen::Vector3d current_target_;

    Eigen::Vector3d prev_joint_states_;
    Eigen::Vector3d prev_tip_position_;
    Eigen::Matrix3d jacobian_;
    double jacobian_update_step_;  // mm

    double target_ahead_dis_;  // mm
    double max_control_vel_;
    double error_tolerance_;  // mm
};

std::ostream& operator<<(std::ostream& os,
                         RttRos2ZaberControlReproduce::State s);