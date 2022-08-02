#include <rtt_ros2_zaber/rtt_ros2_zaber.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/Component.hpp>

rtt_ros2_zaber::rtt_ros2_zaber( const std::string& name ) :
  RTT::TaskContext( name ),
  port_input_wrench( "wrench" ),
  port_input_teleop("teleop_control")
  // port_emergency_stop( "emergency_stop" )
{

  RTT::Service::shared_ptr global_ros =
    RTT::internal::GlobalService::Instance()->getService("ros");
  RTT::OperationCaller<bool()> create_node =
    global_ros->getOperation("create_node");
  create_node.ready();
  create_node();

  // std::cout << "Time: " << global_ros::Clock::now(); std::endl;
  
  addPort("wrench", port_input_wrench );
  addPort("teleop_control",port_input_teleop);
  // addPort("emergency_stop", port_emergency_stop);

  addOperation("Home", &rtt_ros2_zaber::home, this, RTT::OwnThread);
  addOperation("MoveRelative", &rtt_ros2_zaber::move_relatvie, this, RTT::OwnThread);
  addOperation("MoveAbsolute", &rtt_ros2_zaber::move_absolute, this, RTT::OwnThread);
  addOperation("MoveVelocity", &rtt_ros2_zaber::move_velocity, this, RTT::OwnThread);
  addOperation("StopAxis", &rtt_ros2_zaber::stop_axis, this, RTT::OwnThread);
  addOperation("MoveMax", &rtt_ros2_zaber::move_max, this, RTT::OwnThread);
  addOperation("MoveMin", &rtt_ros2_zaber::move_min, this, RTT::OwnThread);
  addOperation("TeleopStart", &rtt_ros2_zaber::teleop_start, this, RTT::OwnThread);
  addOperation("TeleopStop", &rtt_ros2_zaber::teleop_stop, this, RTT::OwnThread);
  
  addProperty("zaber_axis", zaber_axis);

  Library::enableDeviceDbStore();
  
}

bool rtt_ros2_zaber::configureHook(){
  std::cout << "configureHook_1" << std::endl;

  connection = Connection::openSerialPort("/dev/ttyUSB1");

  std::vector<Device> deviceList = connection.detectDevices();
  std::cout << "Found " << deviceList.size() << " devices." << std::endl;

  device = deviceList[0];

  axis = device.getAxis(1);
  axis.home();

  return true;
}

bool rtt_ros2_zaber::startHook(){
  std::cout << "startupHook" << std::endl;
  std::cout << "Zaber Axis: " << zaber_axis << std::endl;
  // old_time = rclcpp::Clock::Clock::now();
  std::cout << "Time: " << old_time << std::endl;
  return true;
}

void rtt_ros2_zaber::updateHook(){
  geometry_msgs::msg::WrenchStamped wrench;

  if( port_input_wrench.read( wrench ) == RTT::NewData ){
    std::cout << wrench.header.stamp.nanosec << std::endl;

    if (wrench.wrench.force.z  > 5.0 ){
      axis.stop();
      throw std::invalid_argument("Force exceeded over the limit of 5N");
    }
        
  }
  
  // geometry_msgs::msg::Twist teleop_cmd;
    
  // if(port_input_teleop.read(teleop_cmd) == RTT::NewData){
  //   std::cout << "Teleop input: " << teleop_cmd.linear.x << std::endl;
  // }

  if (teleop_status)
  {
      geometry_msgs::msg::Twist teleop_cmd;
      // std::cout << rclcpp::Clock::  clock::now() << std::endl;
      if(port_input_teleop.read(teleop_cmd) == RTT::NewData){
        if (zaber_axis == 1){
          teleop_vel = teleop_cmd.linear.x;
        }
        else if (zaber_axis == 2){
          teleop_vel = teleop_cmd.linear.y;
        }
        else if (zaber_axis == 3){
          teleop_vel = teleop_cmd.linear.z;
        }
        else {
          throw std::invalid_argument("Zaber axis out of bound!");
        }

        axis.moveVelocity(((50*teleop_vel)), Units::VELOCITY_MILLIMETRES_PER_SECOND);
      }
      else {
        axis.moveVelocity(0, Units::VELOCITY_MILLIMETRES_PER_SECOND);
      }
  }
}

void rtt_ros2_zaber::stopHook(){
  std::cout << "stopHook" << std::endl;
  axis.stop(); 
}
void rtt_ros2_zaber::cleanupHook(){
  std::cout << "cleanupHook" << std::endl;
  axis.home();
}

void rtt_ros2_zaber::home()
{
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.home(false); 
  }
}

void rtt_ros2_zaber::move_relatvie(const double& distance, const double& velocity)
{
  // std::cout << distance << std::endl;
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.moveRelative(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
  }
}

void rtt_ros2_zaber::move_absolute(const double& distance, const double& velocity)
{
  // std::cout << distance << std::endl;
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.moveAbsolute(distance, Units::LENGTH_MILLIMETRES, false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
  }
}

void rtt_ros2_zaber::move_velocity(const double& velocity)
{
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.moveVelocity(velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
  }
}

void rtt_ros2_zaber::stop_axis(){
  axis.stop(false);
}

void rtt_ros2_zaber::move_max(const double& velocity)
{
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.moveMax(false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
  }
}

void rtt_ros2_zaber::move_min(const double& velocity)
{
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.moveMin(false, velocity, Units::VELOCITY_MILLIMETRES_PER_SECOND);
  }
}

void rtt_ros2_zaber::teleop_start()
{
  teleop_status = true ;
  std::cout << "Teleop control Activated!" << std::endl;
}

void rtt_ros2_zaber::teleop_stop()
{
  teleop_status = false; 
  std::cout << "Teleop control Deactivated!" << std::endl;

}


ORO_CREATE_COMPONENT(rtt_ros2_zaber)
