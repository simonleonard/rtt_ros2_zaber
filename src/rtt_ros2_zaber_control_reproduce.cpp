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
    addPort("demo_point", portDemoPoint);

    addOperation("TipPosition", &RttRos2ZaberControlReproduce::printTipPosition,
                 this, RTT::OwnThread);
    addOperation("Jacobian", &RttRos2ZaberControlReproduce::printJacobian, this,
                 RTT::OwnThread);

    addOperation("AutoInsertion", &RttRos2ZaberControlReproduce::autoInsertion,
                 this, RTT::OwnThread);
    addOperation("Reproduce", &RttRos2ZaberControlReproduce::reproduce, this,
                 RTT::OwnThread);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    addProperty("max_control_vel", max_control_vel_);
    addProperty("jacobian_update_step", jacobian_update_step_);
    addProperty("target_ahead_dis", target_ahead_dis_);
    addProperty("error_tolerance", error_tolerance_);
    addProperty("sg_filtering", sg_filtering_);
}

bool RttRos2ZaberControlReproduce::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberControlReproduce::startHook() {
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberControlReproduce::updateHook() {
    geometry_msgs::msg::TransformStamped RxBaseTip;
    try {
        RxBaseTip =
            tf_buffer_->lookupTransform("base", "tip", tf2::TimePointZero);
        tip_position_ << RxBaseTip.transform.translation.x * 1000.0,
            RxBaseTip.transform.translation.y * 1000.0,
            RxBaseTip.transform.translation.z * 1000.0;

    } catch (const tf2::TransformException& ex) {
        RTT::log(RTT::Error) << "Could not transform base to tip: " << ex.what()
                             << RTT::endlog();
    }

    joint_states_ << getPositionTX(), getPositionLS(), getPositionTZ();

    needle_steering_control_demo_msgs::msg::ControlDemoPoint demo_pt;
    demo_pt.header.frame_id = "Control";
    demo_pt.header.stamp = RxBaseTip.header.stamp;

    demo_pt.inputs.tx = joint_states_.x();
    demo_pt.inputs.ls = joint_states_.y();
    demo_pt.inputs.tz = joint_states_.z();
    demo_pt.outputs.x = tip_position_.x();
    demo_pt.outputs.y = tip_position_.y();
    demo_pt.outputs.z = tip_position_.z();
    portDemoPoint.write(demo_pt);

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

void RttRos2ZaberControlReproduce::printTipPosition() const {
    RTT::log(RTT::Info) << tip_position_.x() << " " << tip_position_.y() << " "
                        << tip_position_.z() << RTT::endlog();
}

void RttRos2ZaberControlReproduce::printJacobian() const {
    RTT::log(RTT::Info) << jacobian_ << RTT::endlog();
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
        insert_cmds.push(cmd);
    }

    insertion_start_time = rtt_ros2_node::getNode(this)->now().nanoseconds();
    RTT::log(RTT::Info) << "Insertion start time: " << insertion_start_time
                        << RTT::endlog();

    demo_traj_.clear();

    state_ = State::DEMO;
    RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();
}

void RttRos2ZaberControlReproduce::collect_demo_points() {
    demo_traj_.addPoint(joint_states_, tip_position_);

    const long curr_time = rtt_ros2_node::getNode(this)->now().nanoseconds();
    while (!insert_cmds.empty() &&
           curr_time >= insert_cmds.front().start_time + insertion_start_time) {
        const auto cmd = insert_cmds.front();
        insert_cmds.pop();

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
    jacobian_ << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    if (sg_filtering_) {
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
        RTT::log(RTT::Error)
            << "Joint out of range! Stopped\n"
            << "LS = " << ls - kLsHome << " TX = " << tx - kTxHome
            << " TZ = " << tz - kTzHome << RTT::endlog();
        linearStage.stop();
        templateX.stop();
        templateZ.stop();
        state_ = State::IDLE;
        RTT::log(RTT::Info) << "Switch to state: " << state_ << RTT::endlog();
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

        RTT::log(RTT::Info)
            << "Jacobian:\n########################################\n"
            << jacobian_ << "\n########################################\n"
            << RTT::endlog();

        // Update previous way point.
        prev_joint_states_ = prev_joint_states_;
        prev_tip_position_ = tip_position_;
    }
}

void RttRos2ZaberControlReproduce::control_loop() {
    reproduce_traj_.addPoint(joint_states_, tip_position_);

    if (sg_filtering_) {
        if (reproduce_traj_.size() < kSGFilterWindow) {
            RTT::log(RTT::Info)
                << "Accumulating measurements..." << RTT::endlog();
            return;
        }
        tip_position_.x() = filter->filter_last_one(reproduce_traj_.tip_x());
        tip_position_.y() = filter->filter_last_one(reproduce_traj_.tip_y());
        tip_position_.z() = filter->filter_last_one(reproduce_traj_.tip_z());
    }

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

            RTT::log(RTT::Info)
                << "Finished reproducing trajectory." << RTT::endlog();
            RTT::log(RTT::Info)
                << "Last target: " << current_target_.transpose()
                << RTT::endlog();
            state_ = State::IDLE;
            RTT::log(RTT::Info)
                << "Switch to state: " << state_ << RTT::endlog();

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

    RTT::log(RTT::Info) << "Current Target:       "
                        << current_target_.transpose() << RTT::endlog();
    RTT::log(RTT::Info) << "Current Tip Position: " << tip_position_.transpose()
                        << RTT::endlog();
    RTT::log(RTT::Info) << "Current Joint States: " << joint_states_.transpose()
                        << RTT::endlog();

    // Calculate control input.
    Eigen::Vector3d control_input =
        jacobian_.inverse() * (current_target_ - tip_position_);
    control_input *= max_control_vel_ / control_input.norm();

    // Send velocities.
    templateX.moveVelocity(control_input.x(), kVelUnitMMPS);
    linearStage.moveVelocity(control_input.y(), kVelUnitMMPS);
    templateZ.moveVelocity(control_input.z(), kVelUnitMMPS);
    RTT::log(RTT::Info) << "Control inputs:       " << control_input.transpose()
                        << RTT::endlog();

    std::cout << std::endl;
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