#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <chrono>
#include <cmath>
#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>

#include "Eigen/Core"
#include "rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp"

NeedleSteeringZaberAxis::NeedleSteeringZaberAxis(
    const std::string& name, double home, double lower_limit,
    double upper_limit, const zaber::motion::ascii::Axis& axis)
    : name_(name),
      home_(home),
      lower_limit_(lower_limit),
      upper_limit_(upper_limit),
      axis_(axis) {
    RTT::log(RTT::Info) << "Axis " << name << ": [" << lower_limit_ - home_
                        << ", " << upper_limit_ - home_
                        << "]\n Info: " << axis_.toString() << RTT::endlog();
}

double NeedleSteeringZaberAxis::getPosition() {
    return axis_.getPosition(kLenUnitMM) - home_;
}

void NeedleSteeringZaberAxis::moveAbs(double position, double velocity,
                                      double accel) {
    if (!busy() && withinRange(position))
        axis_.moveAbsolute((position + home_), kLenUnitMM, false, velocity,
                           kVelUnitMMPS, accel, kAccelUnitMMPS2);
}
void NeedleSteeringZaberAxis::moveRel(double distance, double velocity,
                                      double accel) {
    if (!busy() && withinRange(distance + getPosition()))
        axis_.moveRelative(distance, kLenUnitMM, false, velocity, kVelUnitMMPS);
}

void NeedleSteeringZaberAxis::sendVel(double vel) {
    axis_.moveVelocity(vel, kVelUnitMMPS);
}

void NeedleSteeringZaberAxis::home(bool wait_until_idle) {
    axis_.moveAbsolute(home_, kLenUnitMM, wait_until_idle, kDefaultVel,
                       kVelUnitMMPS, kDefaultAccel, kAccelUnitMMPS2);
}

void NeedleSteeringZaberAxis::stop() { axis_.stop(); }

bool NeedleSteeringZaberAxis::withinRange(double position) const {
    if (lower_limit_ <= position + home_ && position + home_ <= upper_limit_)
        return true;
    RTT::log(RTT::Info) << "Position" << position << " is out of range: ["
                        << lower_limit_ - home_ << ", " << upper_limit_ - home_
                        << "]" << RTT::endlog();
    return false;
}

bool NeedleSteeringZaberAxis::busy() {
    if (axis_.isBusy()) {
        RTT::log(RTT::Info)
            << "Device " << name_ << " is busy" << RTT::endlog();
        return true;
    }
    return false;
}

RttRos2ZaberBase::RttRos2ZaberBase(const std::string& name)
    : RTT::TaskContext(name), calibrating_(false) {
    global_ros_ = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<bool(const std::string&)> create_node =
        global_ros_->getOperation("create_named_node");
    create_node.ready();
    create_node(name);

    addPort("control_measurement", port_meas_);

    addOperation("GetPosition", &RttRos2ZaberBase::getPosition, this,
                 RTT::OwnThread);
    addOperation("MoveAbs", &RttRos2ZaberBase::moveAbs, this, RTT::OwnThread);
    addOperation("MoveRel", &RttRos2ZaberBase::moveRel, this, RTT::OwnThread);
    addOperation("getRange", &RttRos2ZaberBase::getRange, this, RTT::OwnThread);

    addOperation("JointPositions", &RttRos2ZaberBase::printJointPositions, this,
                 RTT::OwnThread);
    addOperation("TipPosition", &RttRos2ZaberBase::printTipPosition, this,
                 RTT::OwnThread);

    addOperation("Home", &RttRos2ZaberBase::home, this, RTT::OwnThread);
    addOperation("StopAllAxes", &RttRos2ZaberBase::stopAllAxes, this,
                 RTT::OwnThread);
    addOperation("Calibrate", &RttRos2ZaberBase::startCalibrate, this,
                 RTT::OwnThread);
    addOperation("ClearCalibration", &RttRos2ZaberBase::clearCalibration, this,
                 RTT::OwnThread);
    addProperty("device_file", device_file_);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    zaber::motion::Library::enableDeviceDbStore();
}

bool RttRos2ZaberBase::RttRos2ZaberBase::configureHook() {
    RTT::log().setLogLevel(RTT::Logger::Info);

    try {
        connection_ =
            zaber::motion::ascii::Connection::openSerialPort(device_file_);
    } catch (const std::exception& exc) {
        RTT::log(RTT::Info)
            << "Failed to open serial port: " << exc.what() << RTT::endlog();
        return false;
    }

    devices_ = connection_.detectDevices();
    RTT::log(RTT::Info) << "Found " << devices_.size() << " device."
                        << RTT::endlog();
    if (devices_.size() != kNumDevices) return false;

    axes_.emplace(std::piecewise_construct, std::forward_as_tuple("LS"),
                  std::forward_as_tuple("LS", kLsHome, kLsLowerLimit,
                                        kLsUpperLimit, devices_[0].getAxis(1)));
    axes_.emplace(std::piecewise_construct, std::forward_as_tuple("TX"),
                  std::forward_as_tuple("TX", kTxHome, kTxLowerLimit,
                                        kTxUpperLimit, devices_[1].getAxis(1)));
    axes_.emplace(std::piecewise_construct, std::forward_as_tuple("TZ"),
                  std::forward_as_tuple("TZ", kTzHome, kTzLowerLimit,
                                        kTzUpperLimit, devices_[2].getAxis(1)));

    setHome(true /* waitUntilIdle */);
    return true;
}

bool RttRos2ZaberBase::startHook() {
    if (!clearCalibration()) return false;
    return true;
}

void RttRos2ZaberBase::updateHook() {
    tf2::Transform rx_base_tip;
    if (!lookUpTransform("base", "tip", rx_base_tip)) return;

    tf2::Vector3 curr_tip_position = 1000.0 * rx_base_tip.getOrigin();
    tip_position_ << curr_tip_position[0], curr_tip_position[1],
        curr_tip_position[2];
    tip_position_ = transform_offset_ * tip_position_;

    joint_states_ << getPosition("TX"), getPosition("LS"), getPosition("TZ");

    auto now = rtt_ros2_node::getNode(this)->now();

    curr_meas_msg_.header.frame_id = "Control";
    curr_meas_msg_.header.stamp = now;
    curr_meas_msg_.js.tx = joint_states_.x();
    curr_meas_msg_.js.ls = joint_states_.y();
    curr_meas_msg_.js.tz = joint_states_.z();
    curr_meas_msg_.tp.x = tip_position_.x();
    curr_meas_msg_.tp.y = tip_position_.y();
    curr_meas_msg_.tp.z = tip_position_.z();
    port_meas_.write(curr_meas_msg_);

    curr_time_ = now.nanoseconds();

    if (calibrating_) {
        if (now.nanoseconds() >= calibration_end_time_) {
            calibrating_ = false;
            axes_.at("LS").stop();
            calibration();
        } else {
            tf2::Transform rx_base_handle;
            if (!lookUpTransform("base", "handle", rx_base_handle)) return;
            tf2::Vector3 curr_handle_position =
                1000.0 * rx_base_handle.getOrigin();
            Eigen::Vector3d handle_pos;
            handle_pos << curr_handle_position[0], curr_handle_position[1],
                curr_handle_position[2];
            calibration_points_.push_back(handle_pos);
        }
    }
}

void RttRos2ZaberBase::stopHook() {
    RTT::log(RTT::Info) << "stopHook" << RTT::endlog();
    setHome(true /* waitUntilIdle */);
}

void RttRos2ZaberBase::cleanupHook() {
    RTT::log(RTT::Info) << "cleanupHook" << RTT::endlog();
    setHome(true /* waitUntilIdle */);
}

double RttRos2ZaberBase::getPosition(const std::string& name) {
    return deviceNameExists(name) ? axes_.at(name).getPosition() : 0.0;
}

void RttRos2ZaberBase::moveAbs(const std::string& name, double position,
                               double velocity, double accel) {
    if (deviceNameExists(name))
        axes_.at(name).moveAbs(position, velocity, accel);
}
void RttRos2ZaberBase::moveRel(const std::string& name, double distance,
                               double velocity, double accel) {
    if (deviceNameExists(name))
        axes_.at(name).moveRel(distance, velocity, accel);
}

std::pair<double, double> RttRos2ZaberBase::getRange(
    const std::string& name) const {
    if (deviceNameExists(name)) axes_.at(name).getRange();
}

void RttRos2ZaberBase::printJointPositions() {
    RTT::log(RTT::Info) << getPosition("TX") << " " << getPosition("LS") << " "
                        << getPosition("TZ") << RTT::endlog();
}

void RttRos2ZaberBase::printTipPosition() const {
    RTT::log(RTT::Info) << tip_position_.x() << " " << tip_position_.y() << " "
                        << tip_position_.z() << RTT::endlog();
}

void RttRos2ZaberBase::home() { setHome(); }

bool RttRos2ZaberBase::lookUpTransform(const std::string& target,
                                       const std::string& source,
                                       tf2::Transform& output,
                                       const rclcpp::Time time,
                                       double time_out) {
    geometry_msgs::msg::TransformStamped rx_stamped;
    try {
        rx_stamped = tf_buffer_->lookupTransform(
            target, source, time, tf2::durationFromSec(time_out));
        tf2::fromMsg(rx_stamped.transform, output);
    } catch (const tf2::TransformException& ex) {
        RTT::log(RTT::Error) << "Could not transform" << source << " to "
                             << target << ex.what() << RTT::endlog();
        return false;
    }
    return true;
}

void RttRos2ZaberBase::setHome(bool wait_until_idle) {
    for (auto& pair : axes_) pair.second.home(wait_until_idle);
}

void RttRos2ZaberBase::stopAllAxes() {
    for (auto& pair : axes_) pair.second.stop();
}

void RttRos2ZaberBase::startCalibrate(double duration) {
    setHome(true /* waitUntilIdle */);

    RTT::log(RTT::Info) << "Calibrating..." << RTT::endlog();
    calibration_end_time_ = rtt_ros2_node::getNode(this)->now().nanoseconds() +
                            duration * 1000000000;
    calibration_points_.clear();
    axes_.at("LS").sendVel(kDefaultVel);
    calibrating_ = true;
}

void RttRos2ZaberBase::calibration() {
    setHome(true /* waitUntilIdle */);
    size_t n = calibration_points_.size();
    Eigen::Matrix<Eigen::Vector3d::Scalar, Eigen::Dynamic, Eigen::Dynamic>
        points(n, 3);
    for (size_t i = 0; i < n; ++i) points.row(i) = calibration_points_[i];

    Eigen::Vector3d origin = points.colwise().mean();
    Eigen::MatrixXd centered = points.rowwise() - origin.transpose();
    Eigen::MatrixXd cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig(cov);
    Eigen::Vector3d new_y = eig.eigenvectors().col(2).normalized();

    RTT::log(RTT ::Info) << "Insertion direction: \n"
                         << new_y.transpose() << RTT::endlog();

    Eigen::Vector3d rot_axis =
        Eigen::Vector3d::UnitY().cross(new_y).normalized();
    const double angle = std::acos(Eigen::Vector3d::UnitY().dot(new_y));

    rclcpp::sleep_for(std::chrono::seconds(1));
    tf2::Transform rx_base_tip;
    if (!lookUpTransform("base", "tip", rx_base_tip,
                         rtt_ros2_node::getNode(this)->now(), 5.0))
        return;

    tf2::Vector3 curr_tip_position = 1000.0 * rx_base_tip.getOrigin();
    Eigen::Vector3d tip_position;
    tip_position << curr_tip_position[0], curr_tip_position[1],
        curr_tip_position[2];

    transform_offset_ = Eigen::Translation3d(tip_position) *
                        Eigen::AngleAxis<double>(angle, rot_axis);
    transform_offset_ = transform_offset_.inverse();

    RTT::log(RTT ::Info) << "\nTransform offset:\n"
                         << transform_offset_.matrix()
                         << "\nFinished calibration" << RTT::endlog();
}

bool RttRos2ZaberBase::clearCalibration() {
    setHome(true /* waitUntilIdle */);

    tf2::Transform rx_base_tip;
    if (!lookUpTransform("base", "tip", rx_base_tip,
                         rtt_ros2_node::getNode(this)->now(), 5.0))
        return false;

    tf2::Vector3 curr_tip_position = 1000.0 * rx_base_tip.getOrigin();
    tip_position_ << curr_tip_position[0], curr_tip_position[1],
        curr_tip_position[2];

    transform_offset_ = Eigen::Translation3d(tip_position_);
    transform_offset_ = transform_offset_.inverse();

    RTT::log(RTT ::Info) << "Transform offset:\n"
                         << transform_offset_.matrix() << RTT::endlog();
    return true;
}

bool RttRos2ZaberBase::deviceNameExists(const std::string& name) const{
    if (axes_.count(name) == 0) {
        RTT::log(RTT::Info) << "Name " << name << " not found" << RTT::endlog();
        return false;
    }
    return true;
}

ORO_CREATE_COMPONENT(RttRos2ZaberBase)