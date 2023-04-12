#include "rtt_ros2_zaber/rtt_ros2_zaber_control_reproduce.hpp"

#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp>

RttRos2ZaberControlReproduce::RttRos2ZaberControlReproduce(
    const std::string& name)
    : RttRos2ZaberBase(name), state(State::IDLE), ready_to_reproduce(false) {
    addOperation("PrintTipPosition",
                 &RttRos2ZaberControlReproduce::printTipPosition, this,
                 RTT::OwnThread);
    addOperation("PrintJacobian", &RttRos2ZaberControlReproduce::printJacobian,
                 this, RTT::OwnThread);

    addOperation("AutoInsertion", &RttRos2ZaberControlReproduce::autoInsertion,
                 this, RTT::OwnThread);
    addOperation("Reproduce", &RttRos2ZaberControlReproduce::reproduce, this,
                 RTT::OwnThread);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    addProperty("linear_stage_step", linear_stage_step);
    addProperty("max_control_vel", max_control_vel);
}

bool RttRos2ZaberControlReproduce::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberControlReproduce::startHook() {
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberControlReproduce::updateHook() {
    // RttRos2ZaberBase::updateHook();

    try {
        geometry_msgs::msg::TransformStamped RxBaseTip =
            tf_buffer_->lookupTransform("base", "tip", tf2::TimePointZero);
        tip_position << RxBaseTip.transform.translation.x * 1000.0,
            RxBaseTip.transform.translation.y * 1000.0,
            RxBaseTip.transform.translation.z * 1000.0;

    } catch (const tf2::TransformException& ex) {
        RTT::log(RTT::Error) << "Could not transform base to tip: " << ex.what()
                             << RTT::endlog();
    }

    if (state == State::DEMO) {
        collect_demo_points();
    } else if (state == State::CONTROL) {
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
    RTT::log(RTT::Info) << tip_position.x() << " " << tip_position.y() << " "
                        << tip_position.z() << RTT::endlog();
}

void RttRos2ZaberControlReproduce::printJacobian() const {
    RTT::log(RTT::Info) << jacobian << RTT::endlog();
}

void RttRos2ZaberControlReproduce::autoInsertion(const std::string& file) {
    if (state != State::IDLE) {
        RTT::log(RTT::Error)
            << "Cannot perform auto insertion, current state is " << state
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
    prev_linear_stage_plane = 0.0;

    demo_trajectory.clear();

    target_trajectory.clear();

    state = State::DEMO;
    RTT::log(RTT::Info) << "Switch to state: " << state << RTT::endlog();
}

void RttRos2ZaberControlReproduce::reproduce(const std::string& experiment) {
    if (state != State::IDLE) {
        RTT::log(RTT::Error)
            << "Cannot reproduce, current state is " << state << RTT::endlog();
        return;
    }
    if (!ready_to_reproduce) {
        RTT::log(RTT::Error) << "Not ready to reprpduce." << RTT::endlog();
        return;
    }
    mimic_trajectory.clear();

    state = State::CONTROL;
    RTT::log(RTT::Info) << "Switch to state: " << state << RTT::endlog();

    // Tx, LS, TZ
    prev_wpt.input << getPositionTX(), getPositionLS(), getPositionTZ();

    // X, Y, Z
    prev_wpt.output = tip_position;

    jacobian << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;

    target_index = 0;
}

void RttRos2ZaberControlReproduce::collect_demo_points() {
    WayPoint wpt;
    // Tx, LS, TZ
    wpt.input << getPositionTX(), getPositionLS(), getPositionTZ();

    // X, Y, Z
    wpt.output = tip_position;
    demo_trajectory.push_back(wpt);

    if (wpt.output.y() - prev_linear_stage_plane > linear_stage_step) {
        RTT::log(RTT::Info) << wpt.output.transpose() << RTT::endlog();
        target_trajectory.push_back(wpt.output);
        prev_linear_stage_plane = wpt.output.y();
    }

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
            if (wpt.output.y() > prev_linear_stage_plane)
                target_trajectory.push_back(wpt.output);

            state = State::IDLE;
            RTT::log(RTT::Info)
                << "Switch to state: " << state << RTT::endlog();

            ready_to_reproduce = true;
            RTT::log(RTT::Info)
                << "Number of target way points: " << target_trajectory.size()
                << RTT::endlog();
            RTT::log(RTT::Info)
                << "Number of demo way points: " << demo_trajectory.size()
                << RTT::endlog();
        }
    }
}

void RttRos2ZaberControlReproduce::control_loop() {
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
        state = State::IDLE;
        RTT::log(RTT::Info) << "Switch to state: " << state << RTT::endlog();
        return;
    }
    WayPoint wpt;
    // Tx, LS, TZ
    wpt.input << getPositionTX(), getPositionLS(), getPositionTZ();

    // X, Y, Z
    wpt.output = tip_position;

    mimic_trajectory.push_back(wpt);

    // Skip reached targets.
    while (target_index < target_trajectory.size() &&
           (wpt.output.y() > target_trajectory[target_index].y() ||
            (wpt.output - target_trajectory[target_index]).norm() <
                kControlTargetThreshold)) {
        ++target_index;
    }

    // Stop if no target left.
    if (target_index == target_trajectory.size() ) {
        linearStage.stop();
        templateX.stop();
        templateZ.stop();

        RTT::log(RTT::Info)
            << "Finished reproducing trajectory." << RTT::endlog();
        RTT::log(RTT::Info) << "Last target:\n"
                            << target_trajectory.back().transpose() << RTT::endlog();
        state = State::IDLE;
        RTT::log(RTT::Info) << "Switch to state: " << state << RTT::endlog();

        return;
    }

    // Update Jacobian.
    if (wpt.output.y() - prev_wpt.output.y() > linear_stage_step) {
        Eigen::Vector3d dx = wpt.input - prev_wpt.input;
        Eigen::Vector3d dy = wpt.output - prev_wpt.output;

        const double dx_norm_2 = dx.transpose() * dx;
        jacobian = jacobian + (dy - jacobian * dx) * dx.transpose() / dx_norm_2;

        RTT::log(RTT::Info)
            << "Jacobian:\n########################################\n"
            << jacobian << "\n########################################"
            << RTT::endlog();

        // Update previous way point.
        prev_wpt = wpt;
    }

    RTT::log(RTT::Info) << "Current Target:\n "
                        << target_trajectory[target_index].transpose()
                        << RTT::endlog();
    RTT::log(RTT::Info) << "Current Tip Position:\n " << wpt.output.transpose()
                        << RTT::endlog();

    // Calculate control input.
    Eigen::Vector3d control_input =
        jacobian.inverse() * (target_trajectory[target_index] - wpt.output);
    control_input *= max_control_vel / control_input.norm();

    // Send velocities.
    templateX.moveVelocity(control_input.x(), kVelUnitMMPS);
    linearStage.moveVelocity(control_input.y(), kVelUnitMMPS);
    templateZ.moveVelocity(control_input.z(), kVelUnitMMPS);
    RTT::log(RTT::Info) << "Control inputs:\n " << control_input.transpose()
                        << " " << control_input.norm() << RTT::endlog();

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