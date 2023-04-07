#include <stdexcept>
#include <string>
#include <sstream>
#include <queue>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <typeinfo>
#include <zaber/motion/ascii.h>

#include "needle_steering_control_demo_msgs/msg/control_demo_point.hpp"

using namespace zaber::motion;
using namespace zaber::motion::ascii;

struct Command{
    Command(const std::string& line){
        std::istringstream ss(line);
        double start_time_s;
        ss >> joint >> start_time_s >> target >> velocity;
        start_time = start_time_s * 1000000000;
    }

    std::string joint;
    long start_time;
    double target;
    double velocity;
};

class rtt_ros2_zaber_auto_insertion : public RTT::TaskContext {
public: 

    rtt_ros2_zaber_auto_insertion(const std::string& name);

    bool configureHook() override; 
    bool startHook() override; 
    void updateHook() override; 
    void stopHook() override; 
    void cleanupHook() override; 

    double getPositionLS(); 
    double getPositionTX(); 
    double getPositionTZ(); 

    void MoveRelativeLS(const double& distance, const double& velocity);
    void MoveRelativeTX(const double& distance, const double& velocity);
    void MoveRelativeTZ(const double& distance, const double& velocity);
    void MoveAbsoluteLS(const double& pose, const double& velocity);
    void MoveAbsoluteTX(const double& pose, const double& velocity);
    void MoveAbsoluteTZ(const double& pose, const double& velocity);

    void autoInsertion(const std::string& file);
    void home();

private: 
    void setHome();
    long getCurrentTime();

    RTT::Service::shared_ptr global_ros; 

    Axis linearStage; 
    Axis templateX; 
    Axis templateZ; 

    Device deviceLS;
    Device deviceTX; 
    Device deviceTZ;  
    Connection connection; 

    RTT::InputPort<sensor_msgs::msg::JointState> portSetJointState; 
    RTT::OutputPort<sensor_msgs::msg::JointState> portGetJointState;
    RTT::OutputPort<needle_steering_control_demo_msgs::msg::ControlDemoPoint> portDemoPoint;

    long oldTime;
    
    long insertion_start_time;
    std::queue<Command> insert_cmds;

    double oldPoseLS; 
    double oldPoseTX; 
    double oldPoseTZ; 

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

};