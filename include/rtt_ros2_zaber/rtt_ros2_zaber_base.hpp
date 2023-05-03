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
#include <unordered_map>

#include "Eigen/Dense"
#include "control_reproduce_interfaces/msg/measurement.hpp"

class NeedleSteeringZaberAxis {
   public:
    NeedleSteeringZaberAxis(const std::string& name, double home,
                            double upper_limit, double lower_limit,
                            const zaber::motion::ascii::Axis& axis);
    NeedleSteeringZaberAxis(const NeedleSteeringZaberAxis&) = delete;
    NeedleSteeringZaberAxis& operator=(const NeedleSteeringZaberAxis&) = delete;

    const std::string& name() const { return name_; }
    double getPosition();
    void moveAbs(double position, double velocity, double accel);
    void moveRel(double position, double velocity, double accel);
    void sendVel(double vel);
    void home(bool wait_until_idle = false);
    void stop();
    std::pair<double, double> getRange() const {
        return std::make_pair(lower_limit_ - home_, upper_limit_ - home_);
    }

   private:
    bool withinRange(double position) const;
    bool busy();
    std::string name_;
    double home_;
    double lower_limit_;
    double upper_limit_;

    zaber::motion::ascii::Axis axis_;
};

class RttRos2ZaberBase : public RTT::TaskContext {
   public:
    RttRos2ZaberBase(const std::string& name);

    bool configureHook() override;
    bool startHook() override;
    void updateHook() override;
    void stopHook() override;
    void cleanupHook() override;

    double getPosition(const std::string& name);
    void moveAbs(const std::string& name, double position, double velocity,
                 double accel);
    void moveRel(const std::string& name, double distance, double velocity,
                 double accel);
    std::pair<double, double> getRange(const std::string& name) const;

    void printJointPositions();
    void printTipPosition() const;

    void home();
    void stopAllAxes();

   protected:
    void setHome(bool wait_until_idle = false);
    bool lookUpTransform(const std::string& target, const std::string& source,
                         tf2::Transform& output,
                         const rclcpp::Time time = rclcpp::Time(0),
                         double time_out = 0.0);

    void startCalibrate(double duration /* s */);
    void calibration();
    bool clearCalibration();

    bool deviceNameExists(const std::string& name) const;

    // serial port device file
    std::string device_file_;

    RTT::Service::shared_ptr global_ros_;

    // Zaber devices
    zaber::motion::ascii::Connection connection_;
    std::vector<zaber::motion::ascii::Device> devices_;
    std::unordered_map<std::string, NeedleSteeringZaberAxis> axes_;

    RTT::OutputPort<control_reproduce_interfaces::msg::Measurement> port_meas_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    Eigen::Vector3d joint_states_;
    Eigen::Vector3d tip_position_;

    Eigen::Transform<double, 3, Eigen::Isometry> transform_offset_;

    long curr_time_;             // ns
    long calibration_end_time_;  // ns
    bool calibrating_;
    std::vector<Eigen::Vector3d> calibration_points_;

    control_reproduce_interfaces::msg::Measurement curr_meas_msg_;
};