#include "rtt_ros2_zaber/rtt_ros2_zaber_control_reproduce.hpp"

#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp>

RttRos2ZaberControlReproduce::RttRos2ZaberControlReproduce(
    const std::string& name)
    : RttRos2ZaberBase(name), state_(State::IDLE), ready_to_reproduce_(false) {
    addOperation("Jacobian", &RttRos2ZaberControlReproduce::printJacobian, this,
                 RTT::OwnThread);
    addOperation("SetJacobian", &RttRos2ZaberControlReproduce::setJacobian,
                 this, RTT::OwnThread);

    addOperation("AutoInsertion", &RttRos2ZaberControlReproduce::autoInsertion,
                 this, RTT::OwnThread);
    addOperation("Reproduce", &RttRos2ZaberControlReproduce::reproduce, this,
                 RTT::OwnThread);

    addProperty("max_control_vel", max_control_vel_);
    addProperty("jacobian_update_step", jacobian_update_step_);
    addProperty("target_ahead_dis", target_ahead_dis_);
    addProperty("error_tolerance", error_tolerance_);
    addProperty("sg_filtering", sg_filtering_);
}

bool RttRos2ZaberControlReproduce::configureHook() {
    jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberControlReproduce::startHook() {
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

void RttRos2ZaberControlReproduce::stopHook() {
    RTT::log(RTT::Info) << "stopHook" << RTT::endlog();
}

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
    demo_traj_.addPoint(joint_states_, tip_position_);

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
            demo_traj_itr_ = demo_traj_.createrIterator();
        }
    }
}

void RttRos2ZaberControlReproduce::reproduce(const std::string& experiment) {
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

    printJacobian();

    if (sg_filtering_ && !demo_points_filtered_) {
        demo_points_filtered_ = true;
        filter.reset(new SavitzkyGolayFilter((kSGFilterWindow - 1) / 2,
                                             (kSGFilterWindow - 1) / 2,
                                             kSGFilterOrder));
        demo_traj_.tip_x() = filter->filter(demo_traj_.tip_x());
        demo_traj_.tip_y() = filter->filter(demo_traj_.tip_y());
        demo_traj_.tip_z() = filter->filter(demo_traj_.tip_z());
        filter.reset(
            new SavitzkyGolayFilter(kSGFilterWindow - 1, 0, kSGFilterOrder));
    }

    current_target_ = demo_traj_itr_->current_outputs();

    state_ = State::CONTROL;
    RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();
}

bool RttRos2ZaberControlReproduce::safety_check() {
    const double ls = linearStage.getPosition(kLenUnitMM),
                 tx = templateX.getPosition(kLenUnitMM),
                 tz = templateZ.getPosition(kLenUnitMM);
    if (ls < kLsLowerLimit || ls > kLsUpperLimit || tx < kTxLowerLimit ||
        tx > kTxUpperLimit || tz < kTzLowerLimit || tz > kTzUpperLimit) {
        linearStage.stop();
        templateX.stop();
        templateZ.stop();
        state_ = State::IDLE;
        RTT::log(RTT::Error)
            << "Joint out of range! Stopped\n"
            << "LS = " << ls - kLsHome << " TX = " << tx - kTxHome
            << " TZ = " << tz - kTzHome << "\nSwitch to state: " << state_
            << RTT::endlog();
        return false;
    }
    return true;
}

void RttRos2ZaberControlReproduce::update_jacobian() {
    if ((tip_position_ - prev_tip_position_).norm() > jacobian_update_step_) {
        Eigen::Vector3d dx = joint_states_ - prev_joint_states_;
        Eigen::Vector3d dy = tip_position_ - prev_tip_position_;

        const double dx_norm_2 = dx.transpose() * dx;
        jacobian_ =
            jacobian_ + (dy - jacobian_ * dx) * dx.transpose() / dx_norm_2;

        printJacobian();

        // Update previous way point.
        prev_joint_states_ = joint_states_;
        prev_tip_position_ = tip_position_;
    }
}

void RttRos2ZaberControlReproduce::control_loop() {
    reproduce_traj_.addPoint(joint_states_, tip_position_);

    /*
        if (sg_filtering_) {
            if (reproduce_traj_.size() < kSGFilterWindow) {
                RTT::log(RTT::Info)
                    << "Accumulating measurements..." << RTT::endlog();
                return;
            }
            tip_position_.x() =
       filter->filter_last_one(reproduce_traj_.tip_x()); tip_position_.y() =
       filter->filter_last_one(reproduce_traj_.tip_y()); tip_position_.z() =
       filter->filter_last_one(reproduce_traj_.tip_z());
        }
    */

    // Stop of joint limits out of ranges.
    if (!safety_check()) return;

    // Update target.
    while ((current_target_.y() < tip_position_.y() ||
            (current_target_ - tip_position_).cwiseAbs().maxCoeff() <
                error_tolerance_)) {
        // Stop if no target left.
        if (demo_traj_itr_->isDone()) {
            linearStage.stop();
            templateX.stop();
            templateZ.stop();

            state_ = State::IDLE;

            RTT::log(RTT::Info)
                << "Finished reproducing trajectory."
                << "\nLast target: " << current_target_.transpose()
                << "\nTip position: " << tip_position_.transpose()
                << "\nSwitch to state: " << state_ << RTT::endlog();

            jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

            return;
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

    // Update Jacobian.
    update_jacobian();

    // Calculate control input.
    Eigen::Vector3d error = current_target_ - tip_position_;
    error = (error.cwiseAbs().array() < error_tolerance_).select(0.0, error);
    Eigen::Vector3d control_input = jacobian_.inverse() * error;
    control_input *= max_control_vel_ / control_input.norm();

    control_input.x() = std::max(std::min(control_input.x(), 0.3), -0.3);
    control_input.z() = std::max(std::min(control_input.z(), 0.3), -0.3);

    // Send velocities.
    templateX.moveVelocity(control_input.x(), kVelUnitMMPS);
    linearStage.moveVelocity(control_input.y(), kVelUnitMMPS);
    templateZ.moveVelocity(control_input.z(), kVelUnitMMPS);

    RTT::log(RTT::Info)
        << "\nCurrent Target:       " << current_target_.transpose()
        << "\nCurrent Tip Position: " << tip_position_.transpose()
        << "\nError vector        : " << error.transpose()
        << "\nCurrent Joint States: " << joint_states_.transpose()
        << "\nControl inputs:       " << control_input.transpose()
        << RTT::endlog();
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