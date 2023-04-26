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

RttRos2ZaberBase::RttRos2ZaberBase(const std::string& name)
    : RTT::TaskContext(name) {
    global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<bool(const std::string&)> create_node =
        global_ros->getOperation("create_named_node");
    create_node.ready();
    create_node(name);

    addPort("demo_point", port_demo_point_);

    addOperation("GetPositionLS", &RttRos2ZaberBase::getPositionLS, this,
                 RTT::OwnThread);
    addOperation("GetPositionTX", &RttRos2ZaberBase::getPositionTX, this,
                 RTT::OwnThread);
    addOperation("GetPositionTZ", &RttRos2ZaberBase::getPositionTZ, this,
                 RTT::OwnThread);

    addOperation("JointPositions", &RttRos2ZaberBase::printJointPositions, this,
                 RTT::OwnThread);
    addOperation("TipPosition", &RttRos2ZaberBase::printTipPosition, this,
                 RTT::OwnThread);

    addOperation("MoveRelativeLS", &RttRos2ZaberBase::MoveRelativeLS, this,
                 RTT::OwnThread);
    addOperation("MoveRelativeTX", &RttRos2ZaberBase::MoveRelativeTX, this,
                 RTT::OwnThread);
    addOperation("MoveRelativeTZ", &RttRos2ZaberBase::MoveRelativeTZ, this,
                 RTT::OwnThread);
    addOperation("MoveAbsoluteLS", &RttRos2ZaberBase::MoveAbsoluteLS, this,
                 RTT::OwnThread);
    addOperation("MoveAbsoluteTX", &RttRos2ZaberBase::MoveAbsoluteTX, this,
                 RTT::OwnThread);
    addOperation("MoveAbsoluteTZ", &RttRos2ZaberBase::MoveAbsoluteTZ, this,
                 RTT::OwnThread);

    addOperation("Home", &RttRos2ZaberBase::home, this, RTT::OwnThread);
    addOperation("Calibrate", &RttRos2ZaberBase::start_calibrate, this,
                 RTT::OwnThread);
    addOperation("ClearCalibration", &RttRos2ZaberBase::clear_calibration, this,
                 RTT::OwnThread);
    addProperty("device_file", device_file);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    zaber::motion::Library::enableDeviceDbStore();
}

bool RttRos2ZaberBase::RttRos2ZaberBase::configureHook() {
    RTT::log().setLogLevel(RTT::Logger::Info);

    try {
        connection =
            zaber::motion::ascii::Connection::openSerialPort(device_file);
    } catch (const std::exception& exc) {
        RTT::log(RTT::Info)
            << "Failed to open serial port: " << exc.what() << RTT::endlog();
        return false;
    }

    std::vector<zaber::motion::ascii::Device> deviceList =
        connection.detectDevices();
    RTT::log(RTT::Info) << "Found " << deviceList.size() << " device."
                        << RTT::endlog();

    deviceLS = deviceList[0];
    deviceTX = deviceList[1];
    deviceTZ = deviceList[2];

    linearStage = deviceLS.getAxis(1);
    templateX = deviceTX.getAxis(1);
    templateZ = deviceTZ.getAxis(1);

    setHome(true /* waitUntilIdle */);

    return true;
}

bool RttRos2ZaberBase::startHook() {
    if (!clear_calibration()) return false;
    return true;
}

void RttRos2ZaberBase::updateHook() {
    tf2::Transform rx_base_tip;
    if (!lookUpTransform("base", "tip", rx_base_tip)) return;

    tf2::Vector3 curr_tip_position = 1000.0 * rx_base_tip.getOrigin();
    tip_position_ << curr_tip_position[0], curr_tip_position[1],
        curr_tip_position[2];
    tip_position_ = transform_offset_ * tip_position_;

    joint_states_ << getPositionTX(), getPositionLS(), getPositionTZ();

    needle_steering_control_demo_msgs::msg::ControlDemoPoint demo_pt;
    demo_pt.header.frame_id = "Control";
    demo_pt.header.stamp = rtt_ros2_node::getNode(this)->now();

    demo_pt.inputs.tx = joint_states_.x();
    demo_pt.inputs.ls = joint_states_.y();
    demo_pt.inputs.tz = joint_states_.z();
    demo_pt.outputs.x = tip_position_.x();
    demo_pt.outputs.y = tip_position_.y();
    demo_pt.outputs.z = tip_position_.z();
    port_demo_point_.write(demo_pt);

    if (calibrating_) {
        if (rtt_ros2_node::getNode(this)->now().nanoseconds() >=
            calibration_end_time_) {
            calibrating_ = false;
            linearStage.stop();
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

double RttRos2ZaberBase::getPositionLS() {
    return linearStage.getPosition(kLenUnitMM) - kLsHome;
}

double RttRos2ZaberBase::getPositionTX() {
    return templateX.getPosition(kLenUnitMM) - kTxHome;
}

double RttRos2ZaberBase::getPositionTZ() {
    return templateZ.getPosition(kLenUnitMM) - kTzHome;
}

void RttRos2ZaberBase::printJointPositions() {
    RTT::log(RTT::Info) << getPositionTX() << " " << getPositionLS() << " "
                        << getPositionTZ() << RTT::endlog();
}

void RttRos2ZaberBase::printTipPosition() const {
    RTT::log(RTT::Info) << tip_position_.x() << " " << tip_position_.y() << " "
                        << tip_position_.z() << RTT::endlog();
}

void RttRos2ZaberBase::MoveRelativeLS(double distance, double velocity) {
    if (linearStage.isBusy()) {
        throw std::invalid_argument(
            "Device is busy, cannot recieve new command");
    } else if (((linearStage.getPosition(kLenUnitMM) + distance) <
                kLsLowerLimit) ||
               ((linearStage.getPosition(kLenUnitMM) + distance) >
                kLsUpperLimit)) {
        RTT::log(RTT::Info)
            << "LinearStage pose: " << getPositionLS() << RTT::endlog();
        throw std::invalid_argument(
            "Device cannot recede beyond the origin 0mm and cannot exceed "
            "above 100mm");
    } else {
        linearStage.moveRelative(distance, kLenUnitMM, false, velocity,
                                 kVelUnitMMPS);
    }
}

void RttRos2ZaberBase::MoveRelativeTX(double distance, double velocity) {
    if (templateX.isBusy()) {
        throw std::invalid_argument(
            "Template x-axis is busy, cannot recieve new command");
    } else if (((templateX.getPosition(kLenUnitMM) + distance) <
                kTxLowerLimit) ||
               ((templateX.getPosition(kLenUnitMM) + distance) >
                kTxUpperLimit)) {
        RTT::log(RTT::Info)
            << "Template x-axis: " << getPositionTX() << RTT::endlog();
        throw std::invalid_argument(
            "Relative move for template along x-axis is out of bound, {-5,5}");
    } else {
        templateX.moveRelative(distance, kLenUnitMM, false, velocity,
                               kVelUnitMMPS);
    }
}

void RttRos2ZaberBase::MoveRelativeTZ(double distance, double velocity) {
    if (templateZ.isBusy()) {
        throw std::invalid_argument(
            "Template z-axis is busy, cannot recieve new command");
    } else if (((templateZ.getPosition(kLenUnitMM) + distance) <
                kTzLowerLimit) ||
               ((templateZ.getPosition(kLenUnitMM) + distance) >
                kTzUpperLimit)) {
        RTT::log(RTT::Info)
            << "Template z-axis: " << getPositionTZ() << RTT::endlog();
        throw std::invalid_argument(
            "Relative move for template along z-axis is out of bound, {-5,5}");
    } else {
        templateZ.moveRelative(distance, kLenUnitMM, false, velocity,
                               kVelUnitMMPS);
    }
}

void RttRos2ZaberBase::MoveAbsoluteLS(double pose, double velocity,
                                      double accel) {
    if (linearStage.isBusy()) {
        throw std::invalid_argument(
            "LinearStage is busy, cannot recieve new command");
    } else if (pose < (kLsLowerLimit - kLsHome) ||
               pose > (kLsUpperLimit - kLsHome)) {
        throw std::invalid_argument(
            "Requested pose for linear stage is out of bound;  {0,100}");
    } else {
        linearStage.moveAbsolute((pose + kLsHome), kLenUnitMM, false, velocity,
                                 kVelUnitMMPS, accel, kAccelUnitMMPS2);
    }
}

void RttRos2ZaberBase::MoveAbsoluteTX(double pose, double velocity,
                                      double accel) {
    if (templateX.isBusy()) {
        throw std::invalid_argument("Template x-axis is busy!");
    } else if (pose < (kTxLowerLimit - kTxHome) ||
               pose > (kTxUpperLimit - kTxHome)) {
        throw std::invalid_argument(
            "Requested pose for template x-axis is out of bound;  {-5,5}");
    } else {
        templateX.moveAbsolute((pose + kTxHome), kLenUnitMM, false, velocity,
                               kVelUnitMMPS, accel, kAccelUnitMMPS2);
    }
}

void RttRos2ZaberBase::MoveAbsoluteTZ(double pose, double velocity,
                                      double accel) {
    if (templateZ.isBusy()) {
        throw std::invalid_argument("Template z-axis is busy!");
    } else if (pose < (kTzLowerLimit - kTzHome) ||
               pose > (kTzUpperLimit - kTzHome)) {
        throw std::invalid_argument(
            "Requested pose for template z-axis is out of bound;  {-5,5}");
    } else {
        templateZ.moveAbsolute((pose + kTzHome), kLenUnitMM, false, velocity,
                               kVelUnitMMPS, accel, kAccelUnitMMPS2);
    }
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
    templateX.moveAbsolute(kTxHome, kLenUnitMM, wait_until_idle, kDefaultVel,
                           kVelUnitMMPS);
    templateZ.moveAbsolute(kTzHome, kLenUnitMM, wait_until_idle, kDefaultVel,
                           kVelUnitMMPS);
    linearStage.moveAbsolute(kLsHome, kLenUnitMM, wait_until_idle, kDefaultVel,
                             kVelUnitMMPS);
}

void RttRos2ZaberBase::start_calibrate(double duration) {
    setHome(true /* waitUntilIdle */);

    RTT::log(RTT::Info) << "Calibrating..." << RTT::endlog();
    calibration_end_time_ = rtt_ros2_node::getNode(this)->now().nanoseconds() +
                            duration * 1000000000;
    calibration_points_.clear();
    linearStage.moveVelocity(kDefaultVel, kVelUnitMMPS);
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

bool RttRos2ZaberBase::clear_calibration() {
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

ORO_CREATE_COMPONENT(RttRos2ZaberBase)