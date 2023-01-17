#include <rtt_ros2_zaber/rtt_ros2_zaber2.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/Component.hpp>

rtt_ros2_zaber2::rtt_ros2_zaber2(const std::string& name):
    RTT::TaskContext(name),
    portSetJointState("set_joint_state")
{
    global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<bool(const std::string&)> create_node = global_ros->getOperation("create_named_node");
    create_node.ready();
    create_node(name);

    addPort("set_joint_state", portSetJointState);
    addPort("joint_state", portGetJointState);

    addOperation("GetPositionLS", &rtt_ros2_zaber2::getPositionLS, this, RTT::OwnThread);
    addOperation("GetPositionTX", &rtt_ros2_zaber2::getPositionTX, this, RTT::OwnThread);
    addOperation("GetPositionTZ", &rtt_ros2_zaber2::getPositionTZ, this, RTT::OwnThread);
    addOperation("HomeLS", &rtt_ros2_zaber2::HomeLS, this, RTT::OwnThread);
    addOperation("HomeTX", &rtt_ros2_zaber2::HomeTX, this, RTT::OwnThread);
    addOperation("HomeTZ", &rtt_ros2_zaber2::HomeTZ, this, RTT::OwnThread);
    addOperation("MoveRelativeLS", &rtt_ros2_zaber2::MoveRelativeLS, this, RTT::OwnThread);
    addOperation("MoveRelativeTX", &rtt_ros2_zaber2::MoveRelativeTX, this, RTT::OwnThread);
    addOperation("MoveRelativeTZ", &rtt_ros2_zaber2::MoveRelativeTZ, this, RTT::OwnThread);
    addOperation("MoveAbsoluteLS", &rtt_ros2_zaber2::MoveAbsoluteLS, this, RTT::OwnThread);
    addOperation("MoveAbsoluteTX", &rtt_ros2_zaber2::MoveAbsoluteTX, this, RTT::OwnThread);
    addOperation("MoveAbsoluteTZ", &rtt_ros2_zaber2::MoveAbsoluteTZ, this, RTT::OwnThread);
    addOperation("StopLS", &rtt_ros2_zaber2::StopLS, this, RTT::OwnThread);
    addOperation("StopTX", &rtt_ros2_zaber2::StopTX, this, RTT::OwnThread);
    addOperation("StopTZ", &rtt_ros2_zaber2::StopTZ, this, RTT::OwnThread);
    addOperation("Stop", &rtt_ros2_zaber2::Stop, this, RTT::OwnThread);

    Library::enableDeviceDbStore(); 
}

bool rtt_ros2_zaber2::configureHook(){

    connection = Connection::openSerialPort("/dev/ttyUSB1");

    std::vector<Device> deviceList = connection.detectDevices(); 
    std::cout << "Found " << deviceList.size() << " device." << std::endl;

    deviceLS = deviceList[0]; 
    deviceTX = deviceList[1];
    deviceTZ = deviceList[2];

    this->linearStage = deviceLS.getAxis(1);
    this->templateX = deviceTX.getAxis(1);
    this->templateZ = deviceTZ.getAxis(1);

    this->templateZ.moveAbsolute(10, Units::LENGTH_MILLIMETRES, true, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    this->templateX.moveAbsolute(12.5, Units::LENGTH_MILLIMETRES, true, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    this->linearStage.moveAbsolute(20, Units::LENGTH_MILLIMETRES, true, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND);


    return true; 

}

bool rtt_ros2_zaber2::startHook() {

    std::cout << "Started startHook" << std::endl;

    rclcpp::Node::SharedPtr node = rtt_ros2_node::getNode(this);
    this->oldPoseLS = this->getPositionLS(); 
    this->oldPoseTX = this->getPositionTX(); 
    this->oldPoseTZ = this->getPositionTZ();

    this->oldTime = node->now().nanoseconds(); 

    return true; 

}

void rtt_ros2_zaber2::updateHook(){

    rclcpp::Node::SharedPtr node = rtt_ros2_node::getNode(this);
    this->updateTime = node->now().nanoseconds(); 

    double qLS = this->getPositionLS(); 
    double qTX = this->getPositionTX();
    double qTZ = this->getPositionTZ(); 

    double qdLS = (qLS - this->oldPoseLS) * (pow(10,9)) / (this->updateTime - this->oldTime);
    double qdTX = (qTX - this->oldPoseTX) * (pow(10,9)) / (this->updateTime - this->oldTime);
    double qdTZ = (qTZ - this->oldPoseTZ) * (pow(10,9)) / (this->updateTime - this->oldTime);

    this->oldPoseLS = qLS; 
    this->oldPoseTX = qTX; 
    this->oldPoseTZ = qTZ; 

    this->oldTime = this->updateTime;

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

    js.header.stamp = node->now(); 
    portGetJointState.write(js);

    if (topic_control){
        sensor_msgs::msg::JointState set_js; 

        if (portSetJointState.read(set_js) == RTT::NewData){

            for (int i = 0; i < set_js.name.size(); i++){
                if (set_js.name[i] == "linearStage"){
                    this->MoveAbsoluteLS(set_js.position[i], set_js.velocity[i]);
                }
                else if (set_js.name[i] == "templateX"){
                    this->MoveAbsoluteTX(set_js.position[i], set_js.velocity[i]);
                }
                else if (set_js.name[i] == "templateY") {
                    this->MoveAbsoluteTZ(set_js.position[i], set_js.velocity[i]);
                }
                else {
                    std::cout << "Invalid Axis name" << std::endl;
                }
            }

        }
    }
 
}

void rtt_ros2_zaber2::stopHook() {
    std::cout << "stopHook" << std::endl;
   
}

void rtt_ros2_zaber2::cleanupHook() {
    
    this->linearStage.moveAbsolute(20, Units::LENGTH_MILLIMETRES, true, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    this->templateZ.moveAbsolute(10, Units::LENGTH_MILLIMETRES, true, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    this->templateX.moveAbsolute(12.5, Units::LENGTH_MILLIMETRES, true, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    
}

double rtt_ros2_zaber2::getPositionLS() {
    return (this->linearStage.getPosition(Units::LENGTH_MILLIMETRES) - this->linearStage_home);
}

double rtt_ros2_zaber2::getPositionTX() {
    return (this->templateX.getPosition(Units::LENGTH_MILLIMETRES) - this->templateX_home); 
}

double rtt_ros2_zaber2::getPositionTZ() {
    return (this->templateZ.getPosition(Units::LENGTH_MILLIMETRES) - this->templateZ_home); 
}

void rtt_ros2_zaber2::HomeLS() {

    if (this->linearStage.isBusy()){
        throw std::invalid_argument("Linear Stage is busy, cannot recieve new command");
    }
    else{
        this->linearStage.moveAbsolute(this->linearStage_home, Units::LENGTH_MILLIMETRES, false, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND); 
    }
}

void rtt_ros2_zaber2::HomeTX() {

    if (this->templateX.isBusy()){
        throw std::invalid_argument("Template X-axis is busy, cannot recieve new command");
    }
    else{
        this->templateX.moveAbsolute(this->templateX_home, Units::LENGTH_MILLIMETRES, false, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND); 
    }

}

void rtt_ros2_zaber2::HomeTZ() {

    if (this->templateZ.isBusy()){
        throw std::invalid_argument("Template X-axis is busy, cannot recieve new command");
    }
    else{
        this->templateZ.moveAbsolute(this->templateZ_home, Units::LENGTH_MILLIMETRES, false, 5, Units::VELOCITY_MILLIMETRES_PER_SECOND); 
    }
    
}

void rtt_ros2_zaber2::MoveRelativeLS(const double& distance, const double& velocity){
    if (this->linearStage.isBusy()){
        throw std::invalid_argument("Device is busy, cannot recieve new command");
    }
    else if (((this->linearStage.getPosition(Units::LENGTH_MILLIMETRES) + distance) < this->linearStage_lower_limit) || 
            ((this->linearStage.getPosition(Units::LENGTH_MILLIMETRES) + distance) > this->linearStage_upper_limit)){
        std::cout << "LinearStage pose: " << this->getPositionLS() << std::endl;
        throw std::invalid_argument("Device cannot recede beyond the origin 0mm and cannot exceed above 100mm");
        
    }
    else{
        this->linearStage.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber2::MoveRelativeTX(const double& distance, const double& velocity) {
    
    if(this->templateX.isBusy()){
        throw std::invalid_argument("Template x-axis is busy, cannot recieve new command");
    }
    else if (((this->templateX.getPosition(Units::LENGTH_MILLIMETRES) + distance) < this->templateX_lower_limit) || 
            ((this->templateX.getPosition(Units::LENGTH_MILLIMETRES) + distance) > this->templateX_upper_limit)){

                std::cout << "Template x-axis: " << this->getPositionTX() << std::endl;
                throw std::invalid_argument("Relative move for template along x-axis is out of bound, {-5,5}");
    }
    else {
        this->templateX.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }


}

void rtt_ros2_zaber2::MoveRelativeTZ(const double& distance, const double& velocity) {
    if(this->templateZ.isBusy()){
        throw std::invalid_argument("Template z-axis is busy, cannot recieve new command");
    }
    else if (((this->templateZ.getPosition(Units::LENGTH_MILLIMETRES) + distance) < this->templateZ_lower_limit) || 
            ((this->templateZ.getPosition(Units::LENGTH_MILLIMETRES) + distance) > this->templateZ_upper_limit)){

                std::cout << "Template z-axis: " << this->getPositionTZ() << std::endl;
                throw std::invalid_argument("Relative move for template along z-axis is out of bound, {-5,5}");
    }
    else {
        this->templateZ.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber2::MoveAbsoluteLS(const double& pose, const double& velocity){
    if (this->linearStage.isBusy()){
        throw std::invalid_argument("LinearStage is busy, cannot recieve new command");
    }
    else if (pose < (this->linearStage_lower_limit - this->linearStage_home) || pose > (this->linearStage_upper_limit - this->linearStage_home)){
        throw std::invalid_argument("Requested pose for linear stage is out of bound;  {0,100}");
    }
    else {
        this->linearStage.moveAbsolute((pose+this->linearStage_home), Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber2::MoveAbsoluteTX(const double& pose, const double& velocity){
    if (this->templateX.isBusy()){
        throw std::invalid_argument("Template x-axis is busy!");
    }
    else if (pose < (this->templateX_lower_limit - this->templateX_home) || pose > (this->templateX_upper_limit - this->templateX_home)){
        throw std::invalid_argument("Requested pose for template x-axis is out of bound;  {-5,5}");
    }
    else {
        this->templateX.moveAbsolute((pose+this->templateX_home), Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber2::MoveAbsoluteTZ(const double& pose, const double& velocity){
    if (this->templateZ.isBusy()){
        throw std::invalid_argument("Template z-axis is busy!");
    }
    else if (pose < (this->templateZ_lower_limit - this->templateZ_home) || pose > (this->templateZ_upper_limit - this->templateZ_home)){
        throw std::invalid_argument("Requested pose for template z-axis is out of bound;  {-5,5}");
    }
    else {
        this->templateZ.moveAbsolute((pose+this->templateZ_home), Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    }
}

void rtt_ros2_zaber2::StopLS() {
    this->linearStage.stop(false);
}

void rtt_ros2_zaber2::StopTX() {
    this->templateX.stop(false);
}

void rtt_ros2_zaber2::StopTZ() {
    this->templateZ.stop(false);
}

void rtt_ros2_zaber2::Stop() {
    this->StopLS(); 
    this->StopTX(); 
    this->StopTZ(); 
}

void rtt_ros2_zaber2::StartTopicControl() {
    this->topic_control = true;
}

void rtt_ros2_zaber2::StopTopicControl() {
    this->topic_control = false;
}


ORO_CREATE_COMPONENT(rtt_ros2_zaber2)
