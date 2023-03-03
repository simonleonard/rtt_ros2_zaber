#include <fstream>

#include<rtt_ros2_zaber/rtt_ros2_zaber_auto_insertion.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/Component.hpp>
#include <limits>

constexpr double TX_HOME = 12.5;
constexpr double templateX_lower_limit = 7.5; 
constexpr double templateX_upper_limit = 17.5; 

constexpr double TZ_HOME = 10.0; 
constexpr double templateZ_lower_limit = 5.0; 
constexpr double templateZ_upper_limit = 15.0; 

constexpr double LS_HOME = 20.0; 
constexpr double linearStage_lower_limit = 20.0; 
constexpr double linearStage_upper_limit = 120.0; 

constexpr double default_speed = 5.0; /* mm /s */

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

    addOperation("AutoInsertion", &rtt_ros2_zaber_auto_insertion::autoInsertion, this, RTT::OwnThread);
    addOperation("Home", &rtt_ros2_zaber_auto_insertion::home, this, RTT::OwnThread);

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(rtt_ros2_node::getNode(this)->get_clock());
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
    portGetJointState.write(getCurrentJointState());

    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("reference", "tip", tf2::TimePointZero);
    //   std::cout << "x: " <<  t.transform.translation.x << " "
                // << "y: " <<  t.transform.translation.y << " "
                // << "z: " <<  t.transform.translation.z << std::endl;
    } catch (const tf2::TransformException & ex) {
        std::cout << "Could not transform reference to tip: " << ex.what() << std::endl;
    }

    const long curr_time = getCurrentTime();
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

sensor_msgs::msg::JointState rtt_ros2_zaber_auto_insertion::getCurrentJointState() {
    
    const long updateTime = getCurrentTime(); 

    const double qLS = getPositionLS(); 
    const double qTX = getPositionTX();
    const double qTZ = getPositionTZ(); 

    const double qdLS = (qLS - oldPoseLS) * (pow(10,9)) / (updateTime - oldTime);
    const double qdTX = (qTX - oldPoseTX) * (pow(10,9)) / (updateTime - oldTime);
    const double qdTZ = (qTZ - oldPoseTZ) * (pow(10,9)) / (updateTime - oldTime);

    oldPoseLS = qLS; 
    oldPoseTX = qTX; 
    oldPoseTZ = qTZ; 

    oldTime = updateTime;

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

    js.header.stamp = rtt_ros2_node::getNode(this)->now(); 

    return js;
}

void rtt_ros2_zaber_auto_insertion::setHome(){
    templateX.moveAbsolute(TX_HOME, Units::LENGTH_MILLIMETRES, true /* waitUntilIdle */, default_speed, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    templateZ.moveAbsolute(TZ_HOME, Units::LENGTH_MILLIMETRES, true /* waitUntilIdle */, default_speed, Units::VELOCITY_MILLIMETRES_PER_SECOND);
    linearStage.moveAbsolute(LS_HOME, Units::LENGTH_MILLIMETRES, true /* waitUntilIdle */, default_speed, Units::VELOCITY_MILLIMETRES_PER_SECOND);
}

long rtt_ros2_zaber_auto_insertion::getCurrentTime(){
    return rtt_ros2_node::getNode(this)->now().nanoseconds(); 
}

ORO_CREATE_COMPONENT(rtt_ros2_zaber_auto_insertion)
