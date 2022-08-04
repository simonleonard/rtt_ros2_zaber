#include <stdexcept>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

#include <zaber/motion/ascii.h>
using namespace zaber::motion;
using namespace zaber::motion::ascii;


class rtt_ros2_zaber : public RTT::TaskContext{

private:

  RTT::InputPort<geometry_msgs::msg::WrenchStamped> port_input_wrench;
  RTT::InputPort<geometry_msgs::msg::Twist> port_input_teleop;
  RTT::OutputPort<sensor_msgs::msg::JointState> port_output_jointstate;

  // RTT::InputPort<std_msgs::msg::Bool> port_emergency_stop; 
  double old_time = 0; 
  double new_time = 0; 
  double teleop_status = false; 
  double teleop_vel = 0.0;

  // auto nh = rclcpp::Node; 

  int zaber_axis = 0;

  Axis axis;
  Device device;
  Connection connection;

  RTT::Service::shared_ptr global_ros;
  
public:

  rtt_ros2_zaber( const std::string& name );

  virtual bool configureHook();
  virtual bool startHook();

  virtual void updateHook();

  virtual void stopHook();
  virtual void cleanupHook();

  double get_position();
  void move_relatvie(const double& distance, const double& velocity);
  void home(); 
  void move_absolute(const double& distance, const double& velocity);
  void move_velocity(const double& velocity);
  void stop_axis();
  void move_max(const double& velocity);
  void move_min(const double& velocity);
  void teleop_start();
  void teleop_stop(); 
};
