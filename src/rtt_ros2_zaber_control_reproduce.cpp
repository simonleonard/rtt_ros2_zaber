#include "rtt_ros2_zaber/rtt_ros2_zaber_control_reproduce.hpp"

#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp>

RttRos2ZaberControlReproduce::RttRos2ZaberControlReproduce(
    const std::string& name)
    : RttRos2ZaberBase(name), ready_to_reproduce(false) {
    addOperation("AutoInsertion", &RttRos2ZaberControlReproduce::autoInsertion,
                 this, RTT::OwnThread);
    addOperation("Reproduce", &RttRos2ZaberControlReproduce::reproduce, this,
                 RTT::OwnThread);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    addProperty("linear_stage_step", linear_stage_step);
}

bool RttRos2ZaberControlReproduce::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberControlReproduce::startHook() {
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberControlReproduce::updateHook() {
    const auto timestamp = rtt_ros2_node::getNode(this)->now();

    RttRos2ZaberBase::updateHook();

    try {
        RxBaseTip = tf_buffer_->lookupTransform("base", "tip",
                                                tf2_ros::fromRclcpp(timestamp));
    } catch (const tf2::TransformException& ex) {
        std::cout << "Could not transform base to tip: " << ex.what()
                  << std::endl;
    }

    if (state == State::DEMO) {
        collect_demo_points();
    } else if (state == State::CONTROL) {
        control_loop();
    }
}

void RttRos2ZaberControlReproduce::stopHook() {
    std::cout << "stopHook" << std::endl;
}

void RttRos2ZaberControlReproduce::cleanupHook() {
    RttRos2ZaberBase::cleanupHook();
}

void RttRos2ZaberControlReproduce::autoInsertion(const std::string& file) {
    if (state != State::IDLE) {
        std::cout << "Cannot perform auto insertion, current state is " << state
                  << std::endl;
        return;
    }

    std::ifstream infile(file);
    std::string line;
    while (std::getline(infile, line)) {
        Command cmd(line);
        insert_cmds.push(cmd);
        std::cout << cmd.joint << " " << cmd.start_time << " " << cmd.target
                  << " " << cmd.velocity << std::endl;
    }

    insertion_start_time = rtt_ros2_node::getNode(this)->now().nanoseconds();
    next_linear_stage_plane = 0.0;

    demo_trajectory.clear();

    std::queue<Eigen::Vector3d> empty_queue;
    target_trajectory.swap(empty_queue);

    ready_to_reproduce = false;
    state = State::DEMO;
}

void RttRos2ZaberControlReproduce::reproduce(const std::string& experiment) {
    if (state != State::IDLE) {
        std::cout << "Cannot reproduce, current state is " << state
                  << std::endl;
        return;
    }
    if (!ready_to_reproduce) {
        std::cout << "Not ready to reprpduce." << std::endl;
    }

    state = State::CONTROL;
    ready_to_reproduce = false;
    // Tx, LS, TZ
    prev_wpt.input << js.position[1], js.position[0], js.position[2];
    // X, Y, Z
    prev_wpt.output << RxBaseTip.transform.translation.x,
        RxBaseTip.transform.translation.y, RxBaseTip.transform.translation.z;

    jacobian << -1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0;
}

void RttRos2ZaberControlReproduce::collect_demo_points() {
    WayPoint wpt;
    // Tx, LS, TZ
    wpt.input << js.position[1], js.position[0], js.position[2];
    // X, Y, Z
    wpt.output << RxBaseTip.transform.translation.x,
        RxBaseTip.transform.translation.y, RxBaseTip.transform.translation.z;

    demo_trajectory.push_back(wpt);

    if (wpt.output.y() > next_linear_stage_plane) {
        target_trajectory.push(wpt.output);
        next_linear_stage_plane += linear_stage_step;
    }

    const long curr_time = rtt_ros2_node::getNode(this)->now().nanoseconds();
    while (!insert_cmds.empty() &&
           curr_time >= insert_cmds.front().start_time + insertion_start_time) {
        const auto cmd = insert_cmds.front();
        insert_cmds.pop();
        std::cout << cmd.joint << std::endl;
        if (cmd.joint == "LS") {
            linearStage.moveAbsolute((cmd.target + kLsHome), kLenUnitMM, false,
                                     cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                     kAccelUnitMMPS2);
        } else if (cmd.joint == "TX") {
            templateX.moveAbsolute((cmd.target + kTXHome), kLenUnitMM, false,
                                   cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                   kAccelUnitMMPS2);
        } else if (cmd.joint == "TZ") {
            templateZ.moveAbsolute((cmd.target + kTzHome), kLenUnitMM, false,
                                   cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                   kAccelUnitMMPS2);
        } else if (cmd.joint == "END") {
            state = State::IDLE;
            ready_to_reproduce = true;
        }
    }
}

void RttRos2ZaberControlReproduce::control_loop() {
    WayPoint wpt;
    // Tx, LS, TZ
    wpt.input << js.position[1], js.position[0], js.position[2];
    // X, Y, Z
    wpt.output << RxBaseTip.transform.translation.x,
        RxBaseTip.transform.translation.y, RxBaseTip.transform.translation.z;

    mimic_trajectory.push_back(wpt);

    // Skip reached targets.
    while (!target_trajectory.empty() &&
           wpt.output.y() > target_trajectory.front().y() &&
           (wpt.output - target_trajectory.front()).norm() <
               kControlTargetThreshold) {
        target_trajectory.pop();
    }

    // Stop if no target left.
    if (target_trajectory.empty()) {
        linearStage.stop();
        templateX.stop();
        templateZ.stop();
    }

    // Update Jacobian.
    Eigen::Vector3d dx = wpt.input - prev_wpt.input;
    Eigen::Vector3d dy = wpt.output - prev_wpt.output;
    jacobian = jacobian +
               (dy - jacobian * dx) * dx.transpose() / (dx.transpose() * dx);

    // Calculate control input.
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