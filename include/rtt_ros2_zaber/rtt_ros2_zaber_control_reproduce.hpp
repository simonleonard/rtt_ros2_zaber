#pragma once

#include <iostream>
#include <queue>
#include <vector>

#include <std_srvs/srv/empty.hpp>

#include "control_reproduce_interfaces/msg/measurement.hpp"
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
    void setJacobian(const std::vector<double>& j);

    void autoInsertion(const std::string& file);
    void reproduce();

    void save_reproduce_results(const std::string& experiment);

    void clear_demo_traj_plot() const;
    enum class State { IDLE, DEMO, CONTROL };

   private:
    void collect_demo_points();

    bool safety_check();

    bool update_target(Eigen::Vector3d curr_tip_position);
    Eigen::Vector3d update_jacobian();
    void send_control_vels(Eigen::Vector3d curr_tip_position);

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

    bool demo_filtering_;
    bool demo_points_filtered_;
    bool reproduce_filtering_;
    std::unique_ptr<SavitzkyGolayFilter> filter_;

    Eigen::Vector3d current_target_;

    Eigen::Vector3d prev_joint_states_;
    Eigen::Vector3d prev_tip_position_;
    Eigen::Matrix3d jacobian_;
    Eigen::Matrix3d jacobian_inv_;
    double jacobian_update_step_;   // s
    long jacobian_update_step_ns_;  // s
    bool use_estimate_tip_position_;

    double target_ahead_dis_;  // mm
    double max_control_vel_;
    double error_tolerance_;  // mm

    long prev_cmd_time_;       // ns
    long prev_jacobian_time_;  // ns

    std::string reproduce_result_folder_;

    RTT::OutputPort<control_reproduce_interfaces::msg::Measurement>
        port_demo_wpt_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr
        clear_demo_wpts_client_;
};

std::ostream& operator<<(std::ostream& os,
                         RttRos2ZaberControlReproduce::State s);