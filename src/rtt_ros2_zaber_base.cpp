#include "rtt_ros2_zaber/rtt_ros2_zaber_base.hpp"

#include <rtt/Component.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>

#include "rtt_ros2_zaber/rtt_ros2_zaber_constants.hpp"

RttRos2ZaberBase::RttRos2ZaberBase(const std::string& name)
    : RTT::TaskContext(name) {
    global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<bool(const std::string&)> create_node =
        global_ros->getOperation("create_named_node");
    create_node.ready();
    create_node(name);

    addPort("joint_state", portGetJointState);

    addOperation("GetPositionLS", &RttRos2ZaberBase::getPositionLS, this,
                 RTT::OwnThread);
    addOperation("GetPositionTX", &RttRos2ZaberBase::getPositionTX, this,
                 RTT::OwnThread);
    addOperation("GetPositionTZ", &RttRos2ZaberBase::getPositionTZ, this,
                 RTT::OwnThread);

    addOperation("JointPositions", &RttRos2ZaberBase::printJointPositions, this,
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

    addProperty("device_file", device_file);

    zaber::motion::Library::enableDeviceDbStore();
}

bool RttRos2ZaberBase::RttRos2ZaberBase::configureHook() {
    RTT::log().setLogLevel(RTT::Logger::Info);

    connection = zaber::motion::ascii::Connection::openSerialPort(device_file);

    std::vector<zaber::motion::ascii::Device> deviceList =
        connection.detectDevices();
    std::cout << "Found " << deviceList.size() << " device." << std::endl;

    deviceLS = deviceList[0];
    deviceTX = deviceList[1];
    deviceTZ = deviceList[2];

    linearStage = deviceLS.getAxis(1);
    templateX = deviceTX.getAxis(1);
    templateZ = deviceTZ.getAxis(1);

    setHome();

    return true;
}

bool RttRos2ZaberBase::startHook() {
    std::cout << "Started startHook" << std::endl;

    oldPoseLS = getPositionLS();
    oldPoseTX = getPositionTX();
    oldPoseTZ = getPositionTZ();

    oldTime = rtt_ros2_node::getNode(this)->now().nanoseconds();

    return true;
}

void RttRos2ZaberBase::updateHook() {
    const auto timestamp = rtt_ros2_node::getNode(this)->now();
    const long curr_time = timestamp.nanoseconds();

    const double qLS = getPositionLS();
    const double qTX = getPositionTX();
    const double qTZ = getPositionTZ();

    const double qdLS =
        (qLS - oldPoseLS) * (pow(10, 9)) / (curr_time - oldTime);
    const double qdTX =
        (qTX - oldPoseTX) * (pow(10, 9)) / (curr_time - oldTime);
    const double qdTZ =
        (qTZ - oldPoseTZ) * (pow(10, 9)) / (curr_time - oldTime);

    oldPoseLS = qLS;
    oldPoseTX = qTX;
    oldPoseTZ = qTZ;

    oldTime = curr_time;

    js.name.push_back("linearStage");
    js.position.push_back(qLS);
    js.velocity.push_back(qdLS);

    js.name.push_back("templateX");
    js.position.push_back(qTX);
    js.velocity.push_back(qdTX);

    js.name.push_back("templateZ");
    js.position.push_back(qTZ);
    js.velocity.push_back(qdTZ);

    js.header.stamp = timestamp;
    portGetJointState.write(js);
}

void RttRos2ZaberBase::stopHook() { std::cout << "stopHook" << std::endl; }

void RttRos2ZaberBase::cleanupHook() { setHome(); }

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

void RttRos2ZaberBase::MoveRelativeLS(double distance, double velocity) {
    if (linearStage.isBusy()) {
        throw std::invalid_argument(
            "Device is busy, cannot recieve new command");
    } else if (((linearStage.getPosition(kLenUnitMM) + distance) <
                kLsLowerLimit) ||
               ((linearStage.getPosition(kLenUnitMM) + distance) >
                kLsUpperLimit)) {
        std::cout << "LinearStage pose: " << getPositionLS() << std::endl;
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
        std::cout << "Template x-axis: " << getPositionTX() << std::endl;
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
        std::cout << "Template z-axis: " << getPositionTZ() << std::endl;
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

void RttRos2ZaberBase::setHome() {
    templateX.moveAbsolute(kTxHome, kLenUnitMM, true /* waitUntilIdle */,
                           kDefaultVel, kVelUnitMMPS);
    templateZ.moveAbsolute(kTzHome, kLenUnitMM, true /* waitUntilIdle */,
                           kDefaultVel, kVelUnitMMPS);
    linearStage.moveAbsolute(kLsHome, kLenUnitMM, true /* waitUntilIdle */,
                             kDefaultVel, kVelUnitMMPS);
}

ORO_CREATE_COMPONENT(RttRos2ZaberBase)
