#pragma once

#include <iostream>
#include <queue>
#include <std_srvs/srv/empty.hpp>
#include <vector>

#include "control_reproduce_interfaces/msg/measurement.hpp"
#include "control_reproduce_interfaces/msg/tip_position.hpp"
#include "control_reproduce_interfaces/srv/add_filtered_demo_wpts.hpp"
#include "control_reproduce_interfaces/srv/toggle_plot.hpp"
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
    void setRevInitJacobian();

    void autoInsertion(const std::string& file);
    void reproduce();

    void save_reproduce_results(const std::string& experiment);

    void clearDemoTrajPlot() const;
    void clearReproduceTrajPlot() const;

    void togglePlot(const std::string& name);

    void setMinLsVel(double vel);

    enum class State { IDLE, DEMO, CONTROL };

   private:
    void collectDemoPoints();

    bool safetyCheck();

    bool updateTarget(Eigen::Vector3d curr_tip_position);
    Eigen::Vector3d updateJacobian();
    void sendControlVels(Eigen::Vector3d curr_tip_position);

    void controlLoop();

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
    int demo_filter_window_;
    int demo_filter_order_;

    bool reproduce_filtering_;
    int reproduce_filter_window_;
    int reproduce_filter_order_;

    std::unique_ptr<SavitzkyGolayFilter> filter_;

    Eigen::Vector3d current_target_;

    Eigen::Vector3d prev_joint_states_;
    Eigen::Vector3d prev_tip_position_;
    Eigen::Matrix3d jacobian_;
    Eigen::Matrix3d jacobian_inv_;
    double jacobian_update_step_;  // s
    bool use_estimate_tip_position_;

    double target_ahead_dis_;  // mm
    double max_control_vel_;
    double y_error_tolerance_;   // mm
    double xz_error_tolerance_;  // mm
    double min_ls_vel_;          // mm/s

    long prev_cmd_time_;       // ns
    long prev_jacobian_time_;  // ns

    std::string reproduce_result_folder_;

    RTT::OutputPort<control_reproduce_interfaces::msg::Measurement>
        port_demo_wpt_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_demo_wpts_client_;
    rclcpp::Client<control_reproduce_interfaces::srv::AddFilteredDemoWpts>::
        SharedPtr add_filtered_demo_wpts_client_;

    RTT::OutputPort<control_reproduce_interfaces::msg::Measurement>
        port_reproduce_wpt_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr
        clear_reproduce_wpts_client_;

    RTT::OutputPort<control_reproduce_interfaces::msg::TipPosition>
        port_reproduce_tp_filtered_;
    control_reproduce_interfaces::msg::TipPosition curr_repr_tip_filtered_msg_;

    RTT::OutputPort<control_reproduce_interfaces::msg::TipPosition>
        port_jacobian_update_tp_;
    control_reproduce_interfaces::msg::TipPosition jacobian_update_tp_msg_;

    rclcpp::Client<control_reproduce_interfaces::srv::TogglePlot>::SharedPtr
        toggle_plot_client_;
};

std::ostream& operator<<(std::ostream& os,
                         RttRos2ZaberControlReproduce::State s);