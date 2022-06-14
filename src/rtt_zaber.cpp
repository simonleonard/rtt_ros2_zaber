#include <rtt_zaber/rtt_zaber.hpp>
#include <rtt/internal/GlobalService.hpp>

#include <rtt/Component.hpp>

rtt_zaber::rtt_zaber( const std::string& name ) :
  RTT::TaskContext( name ),
  port_input_wrench( "wrench" ){

  RTT::Service::shared_ptr global_ros =
    RTT::internal::GlobalService::Instance()->getService("ros");
  RTT::OperationCaller<bool()> create_node =
    global_ros->getOperation("create_node");
  create_node.ready();
  create_node();
  
  addPort( "wrench", port_input_wrench );
  
}

bool rtt_zaber::configureHook(){
  std::cout << "configureHook" << std::endl;
  return true;
}
bool rtt_zaber::startHook(){
  std::cout << "startupHook" << std::endl;
  return true;
}

void rtt_zaber::updateHook(){
  geometry_msgs::msg::WrenchStamped wrench;

  if( port_input_wrench.read( wrench ) == RTT::NewData ){
	std::cout << wrench.wrench.force.x << std::endl;    
  }

}

void rtt_zaber::stopHook(){
  std::cout << "stopHook" << std::endl;
}
void rtt_zaber::cleanupHook(){
  std::cout << "cleanupHook" << std::endl;
}

ORO_CREATE_COMPONENT(rtt_zaber)
