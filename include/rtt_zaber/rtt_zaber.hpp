#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>

#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class rtt_zaber : public RTT::TaskContext{

private:

  RTT::InputPort<geometry_msgs::msg::WrenchStamped> port_input_wrench;
  RTT::OutputPort<sensor_msgs::msg::JointState> port_output_jointstate;

public:

  rtt_zaber( const std::string& name );

  virtual bool configureHook();
  virtual bool startHook();

  virtual void updateHook();

  virtual void stopHook();
  virtual void cleanupHook();
  
};
