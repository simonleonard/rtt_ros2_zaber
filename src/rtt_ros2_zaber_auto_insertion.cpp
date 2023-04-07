#include <fstream>

#include <rtt_ros2_zaber/rtt_ros2_zaber_auto_insertion.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/Component.hpp>
#include <limits>


constexpr double TX_HOME = 12.5;
constexpr double TX_LOWER_LIMIT = 7.5;
constexpr double TX_UPPER_LIMIT = 17.5;

constexpr double TZ_HOME = 10.0; 
constexpr double TZ_LOWER_LIMIT = 5.0;
constexpr double TZ_UPPER_LIMIT = 15.0;

constexpr double LS_HOME = 20.0;
constexpr double LS_LOWER_LIMIT = 20.0;
constexpr double LS_UPPER_LIMIT = 120.0;

constexpr double DEFAULT_SPEED = 5.0; /* mm /s */

rtt_ros2_zaber_auto_insertion::rtt_ros2_zaber_auto_insertion(const std::string& name):
    RTT::TaskContext(name),
    portSetJointState("set_joint_state"),
    insertion_start_time(std::numeric_limits<long>::max() / 2)
{
    global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<bool(const std::string&)> create_node = global_ros->getOperation("create_named_node");
    create_node.ready();
    create_node(name);

    addPort("set_joint_state", portSetJointState);
    addPort("joint_state", portGetJointState);
    addPort("demo_point", portDemoPoint);

    addOperation("AutoInsertion", &rtt_ros2_zaber_auto_insertion::autoInsertion, this, RTT::OwnThread);
    addOperation("Home", &rtt_ros2_zaber_auto_insertion::home, this, RTT::OwnThread);

    addOperation("GetPositionLS", &rtt_ros2_zaber_auto_insertion::getPositionLS, this, RTT::OwnThread);
    addOperation("GetPositionTX", &rtt_ros2_zaber_auto_insertion::getPositionTX, this, RTT::OwnThread);
    addOperation("GetPositionTZ", &rtt_ros2_zaber_auto_insertion::getPositionTZ, this, RTT::OwnThread);

    addOperation("MoveRelativeLS", &rtt_ros2_zaber_auto_insertion::MoveRelativeLS, this, RTT::OwnThread);
    addOperation("MoveRelativeTX", &rtt_ros2_zaber_auto_insertion::MoveRelativeTX, this, RTT::OwnThread);
    addOperation("MoveRelativeTZ", &rtt_ros2_zaber_auto_insertion::MoveRelativeTZ, this, RTT::OwnThread);
    addOperation("MoveAbsoluteLS", &rtt_ros2_zaber_auto_insertion::MoveAbsoluteLS, this, RTT::OwnThread);
    addOperation("MoveAbsoluteTX", &rtt_ros2_zaber_auto_insertion::MoveAbsoluteTX, this, RTT::OwnThread);
    addOperation("MoveAbsoluteTZ", &rtt_ros2_zaber_auto_insertion::MoveAbsoluteTZ, this, RTT::OwnThread);

    const auto node = rtt_ros2_node::getNode(this);
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    Library::enableDeviceDbStore(); 
}

bool rtt_ros2_zaber_auto_insertion::configureHook(){

    connection = Connection::openSerialPort("/dev/ttyUSB1");

    std::vector<Device> deviceList = connection.detectDevices(); 
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

bool rtt_ros2_zaber_auto_insertion::startHook() {

    std::cout << "Started startHook" << std::endl;

    oldPoseLS = getPositionLS(); 
    oldPoseTX = getPositionTX(); 
    oldPoseTZ = getPositionTZ();

    oldTime = rtt_ros2_node::getNode(this)->now().nanoseconds(); 

    return true; 

}

void rtt_ros2_zaber_auto_insertion::updateHook(){
    const auto timestamp = rtt_ros2_node::getNode(this)->now();
    const long curr_time = timestamp.nanoseconds();

    const double qLS = getPositionLS();
    const double qTX = getPositionTX();
    const double qTZ = getPositionTZ();

    const double qdLS = (qLS - oldPoseLS) * (pow(10,9)) / (curr_time - oldTime);
    const double qdTX = (qTX - oldPoseTX) * (pow(10,9)) / (curr_time - oldTime);
    const double qdTZ = (qTZ - oldPoseTZ) * (pow(10,9)) / (curr_time - oldTime);

    oldPoseLS = qLS;
    oldPoseTX = qTX;
    oldPoseTZ = qTZ;

    oldTime = curr_time;

    sensor_msgs::msg::JointState js;

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

    geometry_msgs::msg::TransformStamped t;
    try {
        t = tf_buffer_->lookupTransform("base", "tip", tf2_ros::fromRclcpp(timestamp));
    } catch (const tf2::TransformException & ex) {
        std::cout << "Could not transform base to tip: " << ex.what() << std::endl;
    }

    needle_steering_control_demo_msgs::msg::ControlDemoPoint demo_pt;
    demo_pt.js = js;
    demo_pt.transform = t;
    demo_pt.inputs[0] = qLS;
    demo_pt.inputs[1] = qTX;
    demo_pt.inputs[2] = qTZ;
    demo_pt.outputs[0] = t.transform.translation.x;
    demo_pt.outputs[1] = t.transform.translation.y;
    demo_pt.outputs[2] = t.transform.translation.z;

    portDemoPoint.write(demo_pt);

    while(!insert_cmds.empty() && curr_time >= insert_cmds.front().start_time + insertion_start_time){
        const auto cmd = insert_cmds.front();
        insert_cmds.pop();
        std::cout << cmd.joint << std::endl;
        if(cmd.joint == "LS"){
            linearStage.moveAbsolute((cmd.target + LS_HOME), Units::LENGTH_MILLIMETRES, false, cmd.velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
        }else if(cmd.joint == "TX"){
            templateX.moveAbsolute((cmd.target + TX_HOME), Units::LENGTH_MILLIMETRES, false, cmd.velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
        }else if (cmd.joint == "TZ"){
            templateZ.moveAbsolute((cmd.target + TZ_HOME), Units::LENGTH_MILLIMETRES, false, cmd.velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
        }else if(cmd.joint == "END"){
            insertion_start_time = std::numeric_limits<long>::max() / 2;
        }
    }
}

void rtt_ros2_zaber_auto_insertion::stopHook() {
    std::cout << "stopHook" << std::endl;
}

void rtt_ros2_zaber_auto_insertion::cleanupHook() {
    setHome();
}

double rtt_ros2_zaber_auto_insertion::getPositionLS() {
    return linearStage.getPosition(Units::LENGTH_MILLIMETRES) - LS_HOME;
}

double rtt_ros2_zaber_auto_insertion::getPositionTX() {
    return templateX.getPosition(Units::LENGTH_MILLIMETRES) - TX_HOME; 
}

double rtt_ros2_zaber_auto_insertion::getPositionTZ() {
    return templateZ.getPosition(Units::LENGTH_MILLIMETRES) - TZ_HOME; 
}

void rtt_ros2_zaber_auto_insertion::MoveRelativeLS(const double& distance, const double& velocity){
    if (linearStage.isBusy()){
        throw std::invalid_argument("Device is busy, cannot recieve new command");
    }
    else if (((linearStage.getPosition(Units::LENGTH_MILLIMETRES) + distance) < LS_LOWER_LIMIT) ||
            ((linearStage.getPosition(Units::LENGTH_MILLIMETRES) + distance) > LS_UPPER_LIMIT)){
        std::cout << "LinearStage pose: " << getPositionLS() << std::endl;
        throw std::invalid_argument("Device cannot recede beyond the origin 0mm and cannot exceed above 100mm");
    }
    else{
        linearStage.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber_auto_insertion::MoveRelativeTX(const double& distance, const double& velocity) {
    if(templateX.isBusy()){
        throw std::invalid_argument("Template x-axis is busy, cannot recieve new command");
    }
    else if (((templateX.getPosition(Units::LENGTH_MILLIMETRES) + distance) < TX_LOWER_LIMIT) ||
            ((templateX.getPosition(Units::LENGTH_MILLIMETRES) + distance) > TX_UPPER_LIMIT)){

                std::cout << "Template x-axis: " << getPositionTX() << std::endl;
                throw std::invalid_argument("Relative move for template along x-axis is out of bound, {-5,5}");
    }
    else {
        templateX.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }


}

void rtt_ros2_zaber_auto_insertion::MoveRelativeTZ(const double& distance, const double& velocity) {
    if(templateZ.isBusy()){
        throw std::invalid_argument("Template z-axis is busy, cannot recieve new command");
    }
    else if (((templateZ.getPosition(Units::LENGTH_MILLIMETRES) + distance) < TZ_LOWER_LIMIT) ||
            ((templateZ.getPosition(Units::LENGTH_MILLIMETRES) + distance) > TZ_UPPER_LIMIT)){

                std::cout << "Template z-axis: " << getPositionTZ() << std::endl;
                throw std::invalid_argument("Relative move for template along z-axis is out of bound, {-5,5}");
    }
    else {
        templateZ.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber_auto_insertion::MoveAbsoluteLS(const double& pose, const double& velocity){
    if (linearStage.isBusy()){
        throw std::invalid_argument("LinearStage is busy, cannot recieve new command");
    }
    else if (pose < (LS_LOWER_LIMIT - LS_HOME) || pose > (LS_UPPER_LIMIT - LS_HOME)){
        throw std::invalid_argument("Requested pose for linear stage is out of bound;  {0,100}");
    }
    else {
        linearStage.moveAbsolute((pose+LS_HOME), Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber_auto_insertion::MoveAbsoluteTX(const double& pose, const double& velocity){
    if (templateX.isBusy()){
        throw std::invalid_argument("Template x-axis is busy!");
    }
    else if (pose < (TX_LOWER_LIMIT - TX_HOME) || pose > (TX_UPPER_LIMIT - TX_HOME)){
        throw std::invalid_argument("Requested pose for template x-axis is out of bound;  {-5,5}");
    }
    else {
        templateX.moveAbsolute((pose+TX_HOME), Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber_auto_insertion::MoveAbsoluteTZ(const double& pose, const double& velocity){
    if (templateZ.isBusy()){
        throw std::invalid_argument("Template z-axis is busy!");
    }
    else if (pose < (TZ_LOWER_LIMIT - TZ_HOME) || pose > (TZ_UPPER_LIMIT - TZ_HOME)){
        throw std::invalid_argument("Requested pose for template z-axis is out of bound;  {-5,5}");
    }
    else {
        templateZ.moveAbsolute((pose+TZ_HOME), Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber_auto_insertion::autoInsertion(const std::string& file){
    std::ifstream infile(file);

    std::string line;
    while(std::getline(infile, line)){
        Command cmd(line);
        insert_cmds.push(cmd); 
        std::cout << cmd.joint << " " << cmd.start_time << " " << cmd.target << " " << cmd.velocity << std::endl;
    }

    insertion_start_time = getCurrentTime(); 
}

void rtt_ros2_zaber_auto_insertion::home(){
    setHome();
}

void rtt_ros2_zaber_auto_insertion::setHome(){
    templateX.moveAbsolute(TX_HOME, Units::LENGTH_MILLIMETRES, true /* waitUntilIdle */, DEFAULT_SPEED, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    templateZ.moveAbsolute(TZ_HOME, Units::LENGTH_MILLIMETRES, true /* waitUntilIdle */, DEFAULT_SPEED, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    linearStage.moveAbsolute(LS_HOME, Units::LENGTH_MILLIMETRES, true /* waitUntilIdle */, DEFAULT_SPEED, Units::VELOCITY_MILLIMETRES_PER_SECOND);
}

long rtt_ros2_zaber_auto_insertion::getCurrentTime(){
    return rtt_ros2_node::getNode(this)->now().nanoseconds(); 
}

ORO_CREATE_COMPONENT(rtt_ros2_zaber_auto_insertion)
