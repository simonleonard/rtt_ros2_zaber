#include "rtt_ros2_zaber/rtt_ros2_zaber_control_reproduce.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>

#include "rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp"

RttRos2ZaberControlReproduce::RttRos2ZaberControlReproduce(
    const std::string& name)
    : RttRos2ZaberBase(name),
      state_(State::IDLE),
      ready_to_reproduce_(false),
      demo_filtering_(false),
      demo_points_filtered_(false),
      reproduce_filtering_(false),
      use_estimate_tip_position_(false) {
    addOperation("Jacobian", &RttRos2ZaberControlReproduce::printJacobian, this,
                 RTT::OwnThread);
    addOperation("SetJacobian", &RttRos2ZaberControlReproduce::setJacobian,
                 this, RTT::OwnThread);

    addOperation("AutoInsertion", &RttRos2ZaberControlReproduce::autoInsertion,
                 this, RTT::OwnThread);
    addOperation("Reproduce", &RttRos2ZaberControlReproduce::reproduce, this,
                 RTT::OwnThread);

    addOperation("SaveReproduceResults",
                 &RttRos2ZaberControlReproduce::save_reproduce_results, this,
                 RTT::OwnThread);

    addProperty("max_control_vel", max_control_vel_);
    addProperty("jacobian_update_step", jacobian_update_step_);
    addProperty("target_ahead_dis", target_ahead_dis_);
    addProperty("error_tolerance", error_tolerance_);
    addProperty("demo_filtering", demo_filtering_);
    addProperty("reproduce_filtering", reproduce_filtering_);
    addProperty("use_estimate_tip_position", use_estimate_tip_position_);

    addProperty("reproduce_result_folder", reproduce_result_folder_);
}

bool RttRos2ZaberControlReproduce::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberControlReproduce::startHook() {
    jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    jacobian_inv_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    jacobian_update_step_ns_ = jacobian_update_step_ * 1e9;
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberControlReproduce::updateHook() {
    RttRos2ZaberBase::updateHook();

    if (state_ == State::DEMO) {
        collect_demo_points();
    } else if (state_ == State::CONTROL) {
        control_loop();
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

    insertion_start_time_ = rtt_ros2_node::getNode(this)->now().nanoseconds();
    RTT::log(RTT::Info) << "Insertion start time: " << insertion_start_time_
                        << RTT::endlog();

    demo_traj_.clear();

    state_ = State::DEMO;
    RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();

    demo_points_filtered_ = false;
}

void RttRos2ZaberControlReproduce::collect_demo_points() {
    demo_traj_.addPoint(joint_states_, tip_position_, curr_time_);

    const long curr_time = rtt_ros2_node::getNode(this)->now().nanoseconds();
    while (!insert_cmds_.empty() &&
           curr_time >=
               insert_cmds_.front().start_time + insertion_start_time_) {
        const auto cmd = insert_cmds_.front();
        insert_cmds_.pop();

        RTT::log(RTT::Info) << cmd.joint << RTT::endlog();
        if (cmd.joint == "LS") {
            linearStage.moveAbsolute((cmd.target + kLsHome), kLenUnitMM, false,
                                     cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                     kAccelUnitMMPS2);
        } else if (cmd.joint == "TX") {
            templateX.moveAbsolute((cmd.target + kTxHome), kLenUnitMM, false,
                                   cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                   kAccelUnitMMPS2);
        } else if (cmd.joint == "TZ") {
            templateZ.moveAbsolute((cmd.target + kTzHome), kLenUnitMM, false,
                                   cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                   kAccelUnitMMPS2);
        } else if (cmd.joint == "END") {
            state_ = State::IDLE;
            RTT::log(RTT::Info)
                << "Switch to state: " << state_ << RTT::endlog();

            RTT::log(RTT::Info)
                << "Number of demo way points: " << demo_traj_.size()
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

    prev_tip_position_ = tip_position_;
    prev_joint_states_ = joint_states_;
    prev_cmd_time_ = curr_time_;
    prev_jacobian_time_ = curr_time_;

    printJacobian();

    if (demo_filtering_ && !demo_points_filtered_) {
        demo_points_filtered_ = true;
        filter_.reset(new SavitzkyGolayFilter((kSGFilterWindow - 1) / 2,
                                              (kSGFilterWindow - 1) / 2,
                                              kSGFilterOrder));
        demo_traj_.filter_tip_position_all(*filter_);
        filter_.reset(new SavitzkyGolayFilter((kSGFilterWindow - 1) / 2, 0,
                                              kSGFilterOrder));
    }

    current_target_ = demo_traj_itr_->current_outputs();

    state_ = State::CONTROL;
    RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();
}

bool RttRos2ZaberControlReproduce::safety_check() {
    if (joint_states_.x() + kTxHome < kTxLowerLimit ||
        joint_states_.x() + kTxHome > kTxUpperLimit ||
        joint_states_.y() + kLsHome < kLsLowerLimit ||
        joint_states_.y() + kLsHome > kLsUpperLimit ||
        joint_states_.z() + kTzHome < kTzLowerLimit ||
        joint_states_.z() + kTzHome > kTzUpperLimit) {
        linearStage.stop();
        templateX.stop();
        templateZ.stop();
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

bool RttRos2ZaberControlReproduce::update_target(
    Eigen::Vector3d curr_tip_position) {
    // Update target.
    while ((current_target_.y() < curr_tip_position.y() ||
            (current_target_ - curr_tip_position).cwiseAbs().maxCoeff() <
                error_tolerance_)) {
        // Stop if no target left.
        if (demo_traj_itr_->isDone()) {
            linearStage.stop();
            templateX.stop();
            templateZ.stop();

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
               (demo_traj_itr_->current_outputs() - current_target_).norm() <
                   target_ahead_dis_) {
            demo_traj_itr_->next();
        }

        current_target_ = demo_traj_itr_->isDone()
                              ? demo_traj_itr_->last_outputs()
                              : demo_traj_itr_->current_outputs();
    }
    return true;
}

Eigen::Vector3d RttRos2ZaberControlReproduce::update_jacobian() {
    Eigen::Vector3d dx = joint_states_ - prev_joint_states_;
    Eigen::Vector3d dy = tip_position_ - prev_tip_position_;
    if (curr_time_ - prev_jacobian_time_ > jacobian_update_step_ns_ &&
        dx.norm() > 0.1) {
        jacobian_ = jacobian_ + (dy - jacobian_ * dx) * dx.transpose() /
                                    (dx.transpose() * dx);

        jacobian_inv_ =
            jacobian_inv_ + (dx - jacobian_inv_ * dy) /
                                (dx.transpose() * jacobian_inv_ * dy) *
                                dx.transpose() * jacobian_inv_;
        printJacobian();

        // Update previous way point.
        prev_joint_states_ = joint_states_;
        prev_tip_position_ = tip_position_;
        prev_jacobian_time_ = curr_time_;
        return tip_position_;
    }

    return use_estimate_tip_position_ ? prev_tip_position_ + jacobian_ * dx
                                      : tip_position_;
}

void RttRos2ZaberControlReproduce::send_control_vels(
    Eigen::Vector3d curr_tip_position) {
    // Calculate control input.
    Eigen::Vector3d error = current_target_ - curr_tip_position;
    error = (error.cwiseAbs().array() < error_tolerance_).select(0.0, error);
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
    templateX.moveVelocity(control_input.x(), kVelUnitMMPS);
    linearStage.moveVelocity(control_input.y(), kVelUnitMMPS);
    templateZ.moveVelocity(control_input.z(), kVelUnitMMPS);
}

void RttRos2ZaberControlReproduce::control_loop() {
    reproduce_traj_.addPoint(joint_states_, tip_position_, curr_time_);

    if (reproduce_filtering_) {
        if (!reproduce_traj_.filter_tip_position_xz_last(*filter_)) {
            RTT::log(RTT::Info)
                << "Accumulating measurements..." << RTT::endlog();
            return;
        }
        tip_position_.x() = reproduce_traj_.tip_x_filtered().back();
        tip_position_.z() = reproduce_traj_.tip_z_filtered().back();
    }

    if (!safety_check()) return;

    Eigen::Vector3d curr_tip_position = update_jacobian();

    if (!update_target(curr_tip_position)) return;

    send_control_vels(curr_tip_position);
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