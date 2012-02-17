#ifndef JRL_CONTROLLER_MANAGER_H
#define JRL_CONTROLLER_MANAGER_H

#include "boost/thread/mutex.hpp"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <ros/node_handle.h>
#include <realtime_tools/realtime_publisher.h>

#include <abstract-robot-dynamics/dynamic-robot.hh>

#include <jrl_controller_manager/FreeFlyerState.h>
#include <jrl_controller_manager/JrlControl.h>

namespace jrl_controller_manager{

  class ControllerManager{

  public:
    ControllerManager(CjrlDynamicRobot * in_robot,
		      const ros::NodeHandle& nh=ros::NodeHandle());

    virtual ~ControllerManager();

    void update();

    CjrlDynamicRobot * robot_;

    std::vector<double> command_;

    bool initialize();

  private:
    void publishRobotState();

    ros::NodeHandle controller_node_;

    void commandCB(const JrlControlConstPtr& msg);

    realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_joint_state_;
    realtime_tools::RealtimePublisher<FreeFlyerState> pub_free_flyer_state_;
    ros::Duration publish_period_joint_state_;
    ros::Time last_published_joint_state_;
    
    std::vector<unsigned int> joint_position_in_configuration_vector;

    /* Does the spinner need to be run in a different thread?*/
    void ControllerManagerROSThread();
    boost::thread ros_spinner_thread_;
    
  };

}




#endif
