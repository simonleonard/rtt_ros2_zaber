#include "rtt_ros2_zaber/rtt_ros2_zaber_control_reproduce.hpp"

#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>

#include "rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp"
using namespace std::chrono_literals;
RttRos2ZaberControlReproduce::RttRos2ZaberControlReproduce(
    const std::string& name)
    : RttRos2ZaberBase(name),
      state_(State::IDLE),
      ready_to_reproduce_(false),
      demo_filtering_(false),
      reproduce_filtering_(false),
      use_estimate_tip_position_(false) {
    addOperation("Jacobian", &RttRos2ZaberControlReproduce::printJacobian, this,
                 RTT::OwnThread);
    addOperation("SetJacobian", &RttRos2ZaberControlReproduce::setJacobian,
                 this, RTT::OwnThread);
    addOperation("SetRevInitJacobian",
                 &RttRos2ZaberControlReproduce::setRevInitJacobian, this,
                 RTT::OwnThread);

    addOperation("AutoInsertion", &RttRos2ZaberControlReproduce::autoInsertion,
                 this, RTT::OwnThread);
    addOperation("Reproduce", &RttRos2ZaberControlReproduce::reproduce, this,
                 RTT::OwnThread);

    addOperation("SaveReproduceResults",
                 &RttRos2ZaberControlReproduce::save_reproduce_results, this,
                 RTT::OwnThread);
    addOperation("ClearDemoTrajPlot",
                 &RttRos2ZaberControlReproduce::clearDemoTrajPlot, this,
                 RTT::OwnThread);
    addOperation("ClearReproduceTrajPlot",
                 &RttRos2ZaberControlReproduce::clearReproduceTrajPlot, this,
                 RTT::OwnThread);

    addOperation("TogglePlot", &RttRos2ZaberControlReproduce::togglePlot, this,
                 RTT::OwnThread);

    addProperty("max_control_vel", max_control_vel_);
    addProperty("jacobian_update_step", jacobian_update_step_);
    addProperty("target_ahead_dis", target_ahead_dis_);
    addProperty("y_error_tolerance", y_error_tolerance_);
    addProperty("xz_error_tolerance", xz_error_tolerance_);

    addProperty("demo_filtering", demo_filtering_);
    addProperty("demo_filter_window", demo_filter_window_);
    addProperty("demo_filter_order", demo_filter_order_);

    addProperty("reproduce_filtering", reproduce_filtering_);
    addProperty("reproduce_filter_window", reproduce_filter_window_);
    addProperty("reproduce_filter_order", reproduce_filter_order_);

    addProperty("use_estimate_tip_position", use_estimate_tip_position_);

    addProperty("reproduce_result_folder", reproduce_result_folder_);

    addPort("demo_wpt", port_demo_wpt_);
    addPort("reproduce_wpt", port_reproduce_wpt_);
    addPort("reproduce_tp_filtered", port_reproduce_tp_filtered_);
    addPort("jacobian_update_tp", port_jacobian_update_tp_);

    clear_demo_wpts_client_ =
        rtt_ros2_node::getNode(this)->create_client<std_srvs::srv::Empty>(
            "clear_demo_way_points");
    add_filtered_demo_wpts_client_ =
        rtt_ros2_node::getNode(this)
            ->create_client<
                control_reproduce_interfaces::srv::AddFilteredDemoWpts>(
                "add_filtered_demo_way_points");

    clear_reproduce_wpts_client_ =
        rtt_ros2_node::getNode(this)->create_client<std_srvs::srv::Empty>(
            "clear_reproduce_way_points");

    toggle_plot_client_ =
        rtt_ros2_node::getNode(this)
            ->create_client<control_reproduce_interfaces::srv::TogglePlot>(
                "toggle_plot");
}

bool RttRos2ZaberControlReproduce::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberControlReproduce::startHook() {
    jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    jacobian_inv_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberControlReproduce::updateHook() {
    RttRos2ZaberBase::updateHook();

    if (state_ == State::DEMO) {
        collectDemoPoints();
    } else if (state_ == State::CONTROL) {
        controlLoop();
    }
}

void RttRos2ZaberControlReproduce::stopHook() { RttRos2ZaberBase::stopHook(); }

void RttRos2ZaberControlReproduce::cleanupHook() {
    RttRos2ZaberBase::cleanupHook();
}

void RttRos2ZaberControlReproduce::printJacobian() const {
    RTT::log(RTT::Info)
        << "Jacobian:\n########################################\n"
        << jacobian_ << "\n########################################\n"
        << RTT::endlog();
}

void RttRos2ZaberControlReproduce::setJacobian(const std::vector<double>& j) {
    jacobian_ << j[0], j[1], j[2], j[3], j[4], j[5], j[6], j[7], j[8];
    jacobian_inv_ = jacobian_.inverse();
}

void RttRos2ZaberControlReproduce::setRevInitJacobian() {
    jacobian_ << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
    jacobian_inv_ = jacobian_.inverse();
}

void RttRos2ZaberControlReproduce::autoInsertion(const std::string& file) {
    if (state_ != State::IDLE) {
        RTT::log(RTT::Error)
            << "Cannot perform auto insertion, current state is " << state_
            << RTT::endlog();
        return;
    }

    std::ifstream infile(file);
    std::string line;
    while (std::getline(infile, line)) {
        Command cmd(line);
        insert_cmds_.push(cmd);
    }

    insertion_start_time_ = curr_time_;
    RTT::log(RTT::Info) << "Insertion start time: " << insertion_start_time_
                        << RTT::endlog();

    demo_traj_.clear();
    clearDemoTrajPlot();
    clearReproduceTrajPlot();

    state_ = State::DEMO;
    RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();
}

void RttRos2ZaberControlReproduce::collectDemoPoints() {
    demo_traj_.addPoint(joint_states_, tip_position_, curr_time_);

    port_demo_wpt_.write(curr_meas_msg_);

    while (!insert_cmds_.empty() &&
           curr_time_ >=
               insert_cmds_.front().start_time + insertion_start_time_) {
        const auto cmd = insert_cmds_.front();
        insert_cmds_.pop();

        RTT::log(RTT::Info) << cmd.joint << RTT::endlog();
        if (cmd.joint == "LS" || cmd.joint == "TX" || cmd.joint == "TZ") {
            axes_.at(cmd.joint).moveAbs(cmd.target, cmd.velocity,
                                        kDefaultAccel);
        } else if (cmd.joint == "END") {
            if (demo_filtering_) {
                filter_.reset(new SavitzkyGolayFilter(
                    (demo_filter_window_ - 1) / 2,
                    (demo_filter_window_ - 1) / 2, demo_filter_order_));
                demo_traj_.filter_tip_position_all(*filter_);
                auto itr = demo_traj_.createrIterator(demo_filtering_);
                auto request =
                    std::make_shared<control_reproduce_interfaces::srv::
                                         AddFilteredDemoWpts::Request>();
                request->tps.reserve(demo_traj_.size());
                while (!itr->isDone()) {
                    Eigen::Vector3d tp = itr->current_tp();
                    request->tps.emplace_back();
                    request->tps.back().x = tp.x();
                    request->tps.back().y = tp.y();
                    request->tps.back().z = tp.z();
                    itr->next();
                }
                add_filtered_demo_wpts_client_->async_send_request(request);
            }

            state_ = State::IDLE;
            RTT::log(RTT::Info)
                << "Switch to state: " << state_
                << "\nNumber of demo way points: " << demo_traj_.size()
                << RTT::endlog();

            ready_to_reproduce_ = true;
            demo_traj_itr_ = demo_traj_.createrIterator(demo_filtering_);
        }
    }
}

void RttRos2ZaberControlReproduce::reproduce() {
    if (state_ != State::IDLE) {
        RTT::log(RTT::Error)
            << "Cannot reproduce, current state is " << state_ << RTT::endlog();
        return;
    }
    if (!ready_to_reproduce_) {
        RTT::log(RTT::Error) << "Not ready to reprpduce." << RTT::endlog();
        return;
    }

    reproduce_traj_.clear();
    demo_traj_itr_->first();

    clearReproduceTrajPlot();

    prev_tip_position_ = tip_position_;
    prev_joint_states_ = joint_states_;
    prev_cmd_time_ = curr_time_;
    prev_jacobian_time_ = curr_time_;

    jacobian_update_tp_msg_.x = tip_position_.x();
    jacobian_update_tp_msg_.y = tip_position_.y();
    jacobian_update_tp_msg_.z = tip_position_.z();
    port_jacobian_update_tp_.write(jacobian_update_tp_msg_);

    printJacobian();

    if (reproduce_filtering_) {
        filter_.reset(new SavitzkyGolayFilter(reproduce_filter_window_, 0,
                                              reproduce_filter_order_));
    }

    current_target_ = demo_traj_itr_->current_tp();

    state_ = State::CONTROL;
    RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();
}

bool RttRos2ZaberControlReproduce::safetyCheck() {
    if (joint_states_.x() + kTxHome < kTxLowerLimit ||
        joint_states_.x() + kTxHome > kTxUpperLimit ||
        joint_states_.y() + kLsHome < kLsLowerLimit ||
        joint_states_.y() + kLsHome > kLsUpperLimit ||
        joint_states_.z() + kTzHome < kTzLowerLimit ||
        joint_states_.z() + kTzHome > kTzUpperLimit) {
        stopAllAxes();

        state_ = State::IDLE;
        RTT::log(RTT::Error)
            << "Joint out of range! Stopped\n"
            << "LS = " << joint_states_.y() << " TX = " << joint_states_.x()
            << " TZ = " << joint_states_.z() << "\nSwitch to state: " << state_
            << RTT::endlog();
        jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
        jacobian_inv_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
        return false;
    }
    return true;
}

bool RttRos2ZaberControlReproduce::updateTarget(
    Eigen::Vector3d curr_tip_position) {
    Eigen::Vector3d e_abs = (current_target_ - curr_tip_position).cwiseAbs();
    // Update target.
    while (current_target_.y() < curr_tip_position.y() ||
           (e_abs.y() < y_error_tolerance_ && e_abs.x() < xz_error_tolerance_ &&
            e_abs.z() < xz_error_tolerance_)) {
        // Stop if no target left.
        if (demo_traj_itr_->isDone()) {
            stopAllAxes();

            state_ = State::IDLE;

            RTT::log(RTT::Info)
                << "Finished reproducing trajectory."
                << "\nLast target              : "
                << current_target_.transpose()
                << "\nEstimate Tip position    : "
                << curr_tip_position.transpose()
                << "\nTip position             : " << tip_position_.transpose()
                << "\nSwitch to state          : " << state_
                << "\nNumber of demo way points: " << reproduce_traj_.size()
                << RTT::endlog();
            jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
            jacobian_inv_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

            return false;
        }

        while (!demo_traj_itr_->isDone() &&
               (demo_traj_itr_->current_tp() - current_target_).norm() <
                   target_ahead_dis_) {
            demo_traj_itr_->next();
        }

        current_target_ = demo_traj_itr_->isDone()
                              ? demo_traj_itr_->last_tp()
                              : demo_traj_itr_->current_tp();

        e_abs = (current_target_ - curr_tip_position).cwiseAbs();
    }
    return true;
}

Eigen::Vector3d RttRos2ZaberControlReproduce::updateJacobian() {
    Eigen::Vector3d dx = joint_states_ - prev_joint_states_;
    Eigen::Vector3d dy = tip_position_ - prev_tip_position_;
    if ((curr_time_ - prev_jacobian_time_) * 1.0e-9 > jacobian_update_step_ &&
        dx.norm() > 0.1) {
        jacobian_ = jacobian_ + (dy - jacobian_ * dx) * dx.transpose() /
                                    (dx.transpose() * dx);

        jacobian_inv_ =
            jacobian_inv_ + (dx - jacobian_inv_ * dy) /
                                (dx.transpose() * jacobian_inv_ * dy) *
                                dx.transpose() * jacobian_inv_;
        printJacobian();

        jacobian_update_tp_msg_.x = tip_position_.x();
        jacobian_update_tp_msg_.y = tip_position_.y();
        jacobian_update_tp_msg_.z = tip_position_.z();
        port_jacobian_update_tp_.write(jacobian_update_tp_msg_);

        // Update previous way point.
        prev_joint_states_ = joint_states_;
        prev_tip_position_ = tip_position_;
        prev_jacobian_time_ = curr_time_;
        return tip_position_;
    }

    return use_estimate_tip_position_ ? prev_tip_position_ + jacobian_ * dx
                                      : tip_position_;
}

void RttRos2ZaberControlReproduce::sendControlVels(
    Eigen::Vector3d curr_tip_position) {
    // Calculate control input.
    Eigen::Vector3d error = current_target_ - curr_tip_position;
    error = (error.cwiseAbs().array() < Eigen::Array3d(xz_error_tolerance_,
                                                       y_error_tolerance_,
                                                       xz_error_tolerance_))
                .select(0.0, error);
    Eigen::Vector3d control_input = jacobian_inv_ * error;
    control_input *= max_control_vel_ / control_input.norm();

    RTT::log(RTT::Info)
        << "\nCurrent Target        : " << current_target_.transpose()
        << "\nEstimate Tip Position : " << curr_tip_position.transpose()
        << "\nCurrent Tip Position  : " << tip_position_.transpose()
        << "\nCurrent Joint States  : " << joint_states_.transpose()
        << "\nError vector          : " << error.transpose()
        << "\nControl inputs        : " << control_input.transpose()
        << "\nDelta t               : "
        << (curr_time_ - prev_cmd_time_) * 1.0e-9 << RTT::endlog();
    prev_cmd_time_ = curr_time_;

    // Send velocities.
    axes_.at("TX").sendVel(control_input.x());
    axes_.at("LS").sendVel(control_input.y());
    axes_.at("TZ").sendVel(control_input.z());
}

void RttRos2ZaberControlReproduce::controlLoop() {
    reproduce_traj_.addPoint(joint_states_, tip_position_, curr_time_);

    if (reproduce_filtering_) {
        // if (!reproduce_traj_.filter_tip_position_xz_last(*filter_)) {
        //     RTT::log(RTT::Info)
        //         << "Accumulating measurements..." << RTT::endlog();
        //     prev_joint_states_ = joint_states_;
        //     prev_tip_position_ = tip_position_;
        //     prev_jacobian_time_ = curr_time_;
        //     return;
        // }
        reproduce_traj_.filter_tip_position_xz_last(*filter_);

        curr_repr_tip_filtered_msg_.x = reproduce_traj_.tip_x_filtered().back();
        curr_repr_tip_filtered_msg_.y = reproduce_traj_.tip_y_filtered().back();
        curr_repr_tip_filtered_msg_.z = reproduce_traj_.tip_z_filtered().back();
        port_reproduce_tp_filtered_.write(curr_repr_tip_filtered_msg_);

        tip_position_.x() = reproduce_traj_.tip_x_filtered().back();
        tip_position_.z() = reproduce_traj_.tip_z_filtered().back();
    }

    port_reproduce_wpt_.write(curr_meas_msg_);

    if (!safetyCheck()) return;

    Eigen::Vector3d curr_tip_position = updateJacobian();

    if (!updateTarget(curr_tip_position)) return;

    sendControlVels(curr_tip_position);
}

void RttRos2ZaberControlReproduce::save_reproduce_results(
    const std::string& experiment) {
    boost::filesystem::path exp_path(reproduce_result_folder_);
    exp_path.append(experiment);

    if (!boost::filesystem::exists(exp_path)) {
        boost::filesystem::create_directories(exp_path);
    }

    demo_traj_.write_to_file((exp_path / "demo_traj.txt").string(),
                             demo_filtering_);
    reproduce_traj_.write_to_file((exp_path / "reproduce_traj.txt").string(),
                                  reproduce_filtering_);
}

void RttRos2ZaberControlReproduce::clearDemoTrajPlot() const {
    if (!clear_demo_wpts_client_->service_is_ready()) {
        RTT::log(RTT::Info) << "Service not ready" << RTT::endlog();
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    clear_demo_wpts_client_->async_send_request(request);
}

void RttRos2ZaberControlReproduce::clearReproduceTrajPlot() const {
    if (!clear_reproduce_wpts_client_->service_is_ready()) {
        RTT::log(RTT::Info) << "Service not ready" << RTT::endlog();
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
    clear_reproduce_wpts_client_->async_send_request(request);
}

void RttRos2ZaberControlReproduce::togglePlot(const std::string& name) {
    if (!toggle_plot_client_->service_is_ready()) {
        RTT::log(RTT::Info) << "Service not ready" << RTT::endlog();
        return;
    }
    auto request = std::make_shared<
        control_reproduce_interfaces::srv::TogglePlot::Request>();
    request->name = name;
    auto result = toggle_plot_client_->async_send_request(request);
    if (result.wait_for(0.1s) == std::future_status::ready) {
        const auto status = result.get()->status;
        if (status ==
            control_reproduce_interfaces::srv::TogglePlot::Request::FAILED) {
            RTT::log(RTT::Info)
                << "Plot " << name << " does not exist" << RTT::endlog();
        } else {
            RTT::log(RTT::Info)
                << "Plot " << name << " is "
                << (status == control_reproduce_interfaces::srv::TogglePlot::
                                  Request::SHOWN
                        ? "shown"
                        : "hidden")
                << RTT::endlog();
        }

    } else {
        RTT::log(RTT::Info) << "Failed to call service" << RTT::endlog();
    }
}

std::ostream& operator<<(std::ostream& os,
                         RttRos2ZaberControlReproduce::State s) {
    switch (s) {
        case RttRos2ZaberControlReproduce::State::IDLE:
            os << "Idle";
            break;
        case RttRos2ZaberControlReproduce::State::DEMO:
            os << "Demo";
            break;
        case RttRos2ZaberControlReproduce::State::CONTROL:
            os << "control";
            break;
    }
    return os;
}

ORO_CREATE_COMPONENT(RttRos2ZaberControlReproduce)