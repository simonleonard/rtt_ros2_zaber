#pragma once
#include <zaber/motion/ascii.h>

#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

class RttRos2ZaberBase : public RTT::TaskContext {
   public:
    RttRos2ZaberBase(const std::string& name);

    bool configureHook() override;
    bool startHook() override;
    void updateHook() override;
    void stopHook() override;
    void cleanupHook() override;

    double getPositionLS();
    double getPositionTX();
    double getPositionTZ();

    void printPosition();

    void MoveRelativeLS(double distance, double velocity);
    void MoveRelativeTX(double distance, double velocity);
    void MoveRelativeTZ(double distance, double velocity);
    void MoveAbsoluteLS(double pose, double velocity, double accel);
    void MoveAbsoluteTX(double pose, double velocity, double accel);
    void MoveAbsoluteTZ(double pose, double velocity, double accel);

    void home();

   protected:
    void setHome();

    // serial port device file
    std::string device_file;

    RTT::Service::shared_ptr global_ros;

    zaber::motion::ascii::Axis linearStage;
    zaber::motion::ascii::Axis templateX;
    zaber::motion::ascii::Axis templateZ;

    zaber::motion::ascii::Device deviceLS;
    zaber::motion::ascii::Device deviceTX;
    zaber::motion::ascii::Device deviceTZ;

    zaber::motion::ascii::Connection connection;

    long oldTime;

    double oldPoseLS;
    double oldPoseTX;
    double oldPoseTZ;

    RTT::OutputPort<sensor_msgs::msg::JointState> portGetJointState;
    sensor_msgs::msg::JointState js;
};