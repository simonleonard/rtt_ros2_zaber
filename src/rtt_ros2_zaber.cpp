#include <rtt_ros2_zaber/rtt_ros2_zaber.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/Component.hpp>

rtt_ros2_zaber::rtt_ros2_zaber( const std::string& name ) :
  RTT::TaskContext( name ),
  port_input_wrench( "wrench" ),
  port_input_teleop("teleop_control")
  // port_emergency_stop( "emergency_stop" )
{

  global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
  RTT::OperationCaller<bool(const std::string&)> create_node = global_ros->getOperation("create_named_node");
  create_node.ready();
  create_node(name);

  addPort("wrench", port_input_wrench );
  addPort("teleop_control",port_input_teleop);
  addPort("joint_state", port_output_jointstate);
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
  addOperation("GetPosition", &rtt_ros2_zaber::get_position, this, RTT::OwnThread);
  
  addProperty("zaber_axis", zaber_axis);
  addProperty("device_file", device_file);
  addProperty("home_position", home_position);

  Library::enableDeviceDbStore();
  
}

bool rtt_ros2_zaber::configureHook(){

  connection = Connection::openSerialPort(device_file);

  std::vector<Device> deviceList = connection.detectDevices();
  std::cout << "Found " << deviceList.size() << " devices." << std::endl;

  device = deviceList[0];

  axis = device.getAxis(1);
  axis.moveAbsolute(home_position, Units::LENGTH_MILLIMETRES, true, 10, Units::VELOCITY_MILLIMETRES_PER_SECOND);

  return true;
}

bool rtt_ros2_zaber::startHook(){
  rclcpp::Node::SharedPtr node = rtt_ros2_node::getNode(this);
  old_position = get_position();
  old_time = node->now().nanoseconds();
  sensor_time = node->now().nanoseconds();
  return true;
}

void rtt_ros2_zaber::updateHook(){

  rclcpp::Node::SharedPtr node = rtt_ros2_node::getNode(this);
  update_time = node->now().nanoseconds(); 
  double q = get_position();
  double qd = (q - old_position) * (pow(10,6)) / (update_time - old_time);

  old_position = q;
  old_time = update_time;

  geometry_msgs::msg::WrenchStamped wrench;
  if( port_input_wrench.read( wrench ) == RTT::NewData ){
    sensor_time = wrench.header.stamp.sec * pow(10,9) + wrench.header.stamp.nanosec;

    if (wrench.wrench.force.y  > 6.0 ){
      axis.stop();
      throw std::invalid_argument("Y-axis Force exceeded over the limit of 6N");
    }
    else if (wrench.wrench.force.x  > 6.0 ){
      axis.stop();
      throw std::invalid_argument("X-axis Force exceeded over the limit of 6N");
    }
    else if (wrench.wrench.force.z  > 8.0 ){
      axis.stop();
      throw std::invalid_argument("Z-axis Force exceeded over the limit of 8N");
    }
        
  }

  if ((update_time - sensor_time) > 0.05*pow(10,9)){
    axis.stop();
    throw std::invalid_argument("More than 0.05 sec delay between sensor data time stamp and update loop time stamp");
  }


  std::string name("Y");
  sensor_msgs::msg::JointState js;
  js.name.push_back(name);
  js.position.push_back(q);
  js.velocity.push_back(qd);

  
  js.header.stamp = node->now();
  port_output_jointstate.write(js);
 

  if (axis.getPosition(Units::LENGTH_MILLIMETRES) < home_position){
    axis.stop(false);
  }

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

        if (axis.getPosition(Units::LENGTH_MILLIMETRES) > home_position){
          axis.moveVelocity(((50*teleop_vel)), Units::VELOCITY_MILLIMETRES_PER_SECOND);
        }
        else {
          axis.moveVelocity(0, Units::VELOCITY_MILLIMETRES_PER_SECOND);
        }
      }
      else {
        axis.moveVelocity(0, Units::VELOCITY_MILLIMETRES_PER_SECOND);
      }
  }
}

void rtt_ros2_zaber::stopHook(){
  axis.stop(); 
}
void rtt_ros2_zaber::cleanupHook(){
  axis.moveAbsolute(home_position, Units::LENGTH_MILLIMETRES, true, 10, Units::VELOCITY_MILLIMETRES_PER_SECOND);
}

double rtt_ros2_zaber::get_position()
{ 
  return axis.getPosition(Units::LENGTH_MILLIMETRES); 
}

void rtt_ros2_zaber::home()
{
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else{
    axis.moveAbsolute(home_position, Units::LENGTH_MILLIMETRES, false, 10, Units::VELOCITY_MILLIMETRES_PER_SECOND); 
  }
}

void rtt_ros2_zaber::move_relatvie(const double& distance, const double& velocity)
{
  // std::cout << distance << std::endl;
  if (axis.isBusy()){
    throw std::invalid_argument("Device is busy, cannot recieve new command");
  }
  else if ((axis.getPosition(Units::LENGTH_MILLIMETRES) + distance) < home_position){
    throw std::invalid_argument("Device cannot recede beyond the 20mm initial position");
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
  else if (distance < home_position){
    throw std::invalid_argument("Device cannot recede beyond the 20mm initial position");
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
  else if (home_position != 0){
    throw std::invalid_argument("MoveMin can only be used when home position is zero");
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
