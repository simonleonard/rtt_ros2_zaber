#include <stdexcept>
#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <typeinfo>
#include <zaber/motion/ascii.h>
using namespace zaber::motion;
using namespace zaber::motion::ascii;

class rtt_ros2_zaber : public RTT::TaskContext {
public: 

    rtt_ros2_zaber(const std::string& name);

    bool configureHook() override; 
    bool startHook() override; 
    void updateHook() override; 
    void stopHook() override; 
    void cleanupHook() override; 

    double getPositionLS(); 
    double getPositionTX(); 
    double getPositionTZ(); 
    void HomeLS(); 
    void HomeTX(); 
    void HomeTZ();
    void MoveRelativeLS(const double& distance, const double& velocity);
    void MoveRelativeTX(const double& distance, const double& velocity);
    void MoveRelativeTZ(const double& distance, const double& velocity);
    void MoveAbsoluteLS(const double& pose, const double& velocity);
    void MoveAbsoluteTX(const double& pose, const double& velocity);
    void MoveAbsoluteTZ(const double& pose, const double& velocity);
    void MoveVelocityLS(const double& velocity);
    void MoveVelocityTX(const double& velocity);
    void MoveVelocityTZ(const double& velocity);
    void StopLS();
    void StopTX();
    void StopTZ();
    void Stop(); 
    void StartTopicControl(); 
    void StopTopicControl(); 
    void StartTeleopControl(); 
    void StopTeleopControl(); 
    void StartTeleopXZControl(const double& velocity); 
    void StopTeleopXZControl(); 


private: 

    RTT::Service::shared_ptr global_ros; 

    Axis linearStage; 
    Axis templateX; 
    Axis templateZ; 

    Device deviceLS;
    Device deviceTX; 
    Device deviceTZ;  
    Connection connection; 

    double templateX_home = 12.5;
    double templateX_lower_limit = 7.5; 
    double templateX_upper_limit = 17.5; 

    double templateZ_home = 10; 
    double templateZ_lower_limit = 5; 
    double templateZ_upper_limit = 15; 
    
    double linearStage_home = 20; 
    double linearStage_lower_limit = 20; 
    double linearStage_upper_limit = 120; 

    RTT::InputPort<sensor_msgs::msg::JointState> portSetJointState; 
    RTT::OutputPort<sensor_msgs::msg::JointState> portGetJointState;
    RTT::InputPort<geometry_msgs::msg::Twist> portSetTeleop; 

    bool topic_control = false;
    bool teleop_control = false;  
    bool teleop_controlXZ = false; 

    long int updateTime; 
    long int oldTime; 

    double oldPoseLS; 
    double oldPoseTX; 
    double oldPoseTZ; 

    double linearStageVel = 1.0; 

};