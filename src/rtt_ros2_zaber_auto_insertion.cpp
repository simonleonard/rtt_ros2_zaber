#include "rtt_ros2_zaber/rtt_ros2_zaber_auto_insertion.hpp"

#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp>

RttRos2ZaberAutoInsertion::RttRos2ZaberAutoInsertion(const std::string& name)
    : RttRos2ZaberBase(name),
      insertion_start_time_(std::numeric_limits<long>::max() / 2) {
    addOperation("AutoInsertion", &RttRos2ZaberAutoInsertion::autoInsertion,
                 this, RTT::OwnThread);
}

bool RttRos2ZaberAutoInsertion::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberAutoInsertion::startHook() {
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberAutoInsertion::updateHook() {
    RttRos2ZaberBase::updateHook();

    while (!insert_cmds_.empty() &&
           rtt_ros2_node::getNode(this)->now().nanoseconds() >=
               insert_cmds_.front().start_time + insertion_start_time_) {
        const auto cmd = insert_cmds_.front();
        insert_cmds_.pop();
        std::cout << cmd.joint << std::endl;
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
            insertion_start_time_ = std::numeric_limits<long>::max() / 2;
        }
    }
}

void RttRos2ZaberAutoInsertion::stopHook() {
    std::cout << "stopHook" << std::endl;
}

void RttRos2ZaberAutoInsertion::cleanupHook() {
    RttRos2ZaberBase::cleanupHook();
}

void RttRos2ZaberAutoInsertion::autoInsertion(const std::string& file) {
    std::ifstream infile(file);

    std::string line;
    while (std::getline(infile, line)) {
        Command cmd(line);
        insert_cmds_.push(cmd);
        std::cout << cmd.joint << " " << cmd.start_time << " " << cmd.target
                  << " " << cmd.velocity << std::endl;
    }

    insertion_start_time_ = rtt_ros2_node::getNode(this)->now().nanoseconds();
}

ORO_CREATE_COMPONENT(RttRos2ZaberAutoInsertion)
