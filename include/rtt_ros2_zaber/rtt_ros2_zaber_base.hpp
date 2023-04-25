#pragma once
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <zaber/motion/ascii.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/TaskContext.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "Eigen/Dense"
#include "needle_steering_control_demo_msgs/msg/control_demo_point.hpp"

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

    void printJointPositions();
    void printTipPosition() const;

    void MoveRelativeLS(double distance, double velocity);
    void MoveRelativeTX(double distance, double velocity);
    void MoveRelativeTZ(double distance, double velocity);
    void MoveAbsoluteLS(double pose, double velocity, double accel);
    void MoveAbsoluteTX(double pose, double velocity, double accel);
    void MoveAbsoluteTZ(double pose, double velocity, double accel);

    void home();

   protected:
    void setHome(bool wait_until_idle = false);
    bool lookUpTransform(const std::string& target, const std::string& source,
                         tf2::Transform& output, double time_out = 0.0);

    void start_calibrate(double duration /* s */);
    void calibration();
    bool clear_calibration();

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

    RTT::OutputPort<needle_steering_control_demo_msgs::msg::ControlDemoPoint>
        port_demo_point_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    Eigen::Vector3d joint_states_;
    Eigen::Vector3d tip_position_;

    Eigen::Transform<double, 3, Eigen::Isometry> transform_offset_;

    long calibration_end_time_;
    bool calibrating_;
    std::vector<Eigen::Vector3d> calibration_points_;
};