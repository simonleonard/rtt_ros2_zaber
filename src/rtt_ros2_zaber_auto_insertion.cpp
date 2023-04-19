#include "rtt_ros2_zaber/rtt_ros2_zaber_auto_insertion.hpp"

#include <fstream>
#include <limits>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp>

RttRos2ZaberAutoInsertion::RttRos2ZaberAutoInsertion(const std::string& name)
    : RttRos2ZaberBase(name),
      insertion_start_time(std::numeric_limits<long>::max() / 2) {
    addPort("demo_point", portDemoPoint);
    addPort("control_input", portControlInput);
    addPort("control_output", portControlOutput);

    addOperation("AutoInsertion", &RttRos2ZaberAutoInsertion::autoInsertion,
                 this, RTT::OwnThread);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

bool RttRos2ZaberAutoInsertion::configureHook() {
    return RttRos2ZaberBase::configureHook();
}

bool RttRos2ZaberAutoInsertion::startHook() {
    return RttRos2ZaberBase::startHook();
}

void RttRos2ZaberAutoInsertion::updateHook() {
    const auto timestamp = rtt_ros2_node::getNode(this)->now();
    const long curr_time = timestamp.nanoseconds();

    RttRos2ZaberBase::updateHook();

    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer_->lookupTransform("base", "tip",
                                        tf2_ros::fromRclcpp(timestamp));
    } catch (const tf2::TransformException& ex) {
        std::cout << "Could not transform base to tip: " << ex.what()
                  << std::endl;
    }

    needle_steering_control_demo_msgs::msg::ControlDemoPoint demo_pt;
    demo_pt.js = js;
    demo_pt.transform = t;

    demo_pt.inputs[0] = getPositionTX();
    demo_pt.inputs[1] = getPositionLS();
    demo_pt.inputs[2] = getPositionTZ();
    demo_pt.outputs[0] = t.transform.translation.x * 1000.0;
    demo_pt.outputs[1] = t.transform.translation.y * 1000.0;
    demo_pt.outputs[2] = t.transform.translation.z * 1000.0;
    portDemoPoint.write(demo_pt);

    needle_steering_control_demo_msgs::msg::Inputs control_inputs;
    control_inputs.tx = demo_pt.inputs[0];
    control_inputs.ls = demo_pt.inputs[1];
    control_inputs.tz = demo_pt.inputs[2];
    portControlInput.write(control_inputs);

    needle_steering_control_demo_msgs::msg::Outputs control_outputs;
    control_outputs.x = demo_pt.outputs[0];
    control_outputs.y = demo_pt.outputs[1];
    control_outputs.z = demo_pt.outputs[2];
    portControlOutput.write(control_outputs);

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
            templateX.moveAbsolute((cmd.target + kTxHome), kLenUnitMM, false,
                                   cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                   kAccelUnitMMPS2);
        } else if (cmd.joint == "TZ") {
            templateZ.moveAbsolute((cmd.target + kTzHome), kLenUnitMM, false,
                                   cmd.velocity, kVelUnitMMPS, kDefaultAccel,
                                   kAccelUnitMMPS2);
        } else if (cmd.joint == "END") {
            insertion_start_time = std::numeric_limits<long>::max() / 2;
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
        insert_cmds.push(cmd);
        std::cout << cmd.joint << " " << cmd.start_time << " " << cmd.target
                  << " " << cmd.velocity << std::endl;
    }

    insertion_start_time = rtt_ros2_node::getNode(this)->now().nanoseconds();
}

ORO_CREATE_COMPONENT(RttRos2ZaberAutoInsertion)
