#include <sstream>
#include <string>
#include <jrl/mal/matrixabstractlayer.hh>
#include "jrl_controller_manager/controller_manager.h"

using namespace ros;
namespace jrl_controller_manager {
  ControllerManager::ControllerManager(CjrlDynamicRobot * in_robot,const ros::NodeHandle& nh):
    robot_(in_robot),
    controller_node_(nh),
    pub_joint_state_(nh,"joint_states",10),
    publish_period_joint_state_(0),
    last_published_joint_state_(ros::Time::now())
  {}

  ControllerManager::~ControllerManager()
  {
  }

  bool ControllerManager::initialize()
  {
    if(!robot_) {
      ROS_ERROR("robot is not properly initialized");
      return false;
    }
    if(!robot_->rootJoint()) {
      ROS_ERROR("robot is not properly initialized");
      return false;
    }
    std::vector<CjrlJoint *> joint_vector = robot_->jointVector();
    std::vector<CjrlJoint *> actuated_joint_vector = robot_->getActuatedJoints();
    unsigned int nb_dofs = robot_->numberDof();

    pub_joint_state_.msg_.name.resize(nb_dofs);
    pub_joint_state_.msg_.position.resize(nb_dofs);
    pub_joint_state_.msg_.velocity.resize(nb_dofs);
    pub_joint_state_.msg_.effort.resize(0);

    command_.resize(actuated_joint_vector.size());
    for (unsigned int i = 0; i < actuated_joint_vector.size(); ++i)
      command_[i] = 0;

    sub_ = controller_node_.subscribe<JrlControl>("command",1,&ControllerManager::commandCB,this);
     
    ros_spinner_thread_ =  
      boost::thread( boost::bind( &ControllerManager::ControllerManagerROSThread,this ) );
    
    publish_period_joint_state_ = Duration(0.005);
    
    std::cout << "Exiting controller manager::initialize()\n";
    return true;
  }

  void ControllerManager::update()
  {
    //Publish state
    publishRobotState();
  }

  void ControllerManager::publishRobotState()
  {
    std::vector<CjrlJoint *> joint_vector = robot_->jointVector();

    vectorN current_configuration = robot_->currentConfiguration();
    vectorN current_velocity = robot_->currentVelocity();
  
    ros::Time now = ros::Time::now();
    if (now > last_published_joint_state_ + publish_period_joint_state_) {
      if (pub_joint_state_.trylock()) {

	while (last_published_joint_state_ + publish_period_joint_state_ < now)
	  last_published_joint_state_ = 
	    last_published_joint_state_ + publish_period_joint_state_;
      
	/*
	  ROS Joints are actually DoFs. For jrl joints with several DoFs,
	  we create additional joints.
	  Note: 0 DoF joints are ignored.
	*/
	unsigned int ros_joint_id = 0;
	for (std::vector<CjrlJoint *>::iterator joint_it = joint_vector.begin();
	     joint_it != joint_vector.end();
	     ++joint_it) {
	  unsigned int joint_position = (*joint_it)->rankInConfiguration();
	  if ((*joint_it)->numberDof() > 1) {
	    for(unsigned int dof_id = 0; dof_id < (*joint_it)->numberDof(); ++dof_id) {
	      std::stringstream dof_name;
	      dof_name << (*joint_it)->getName() << "-" << dof_id;
	      pub_joint_state_.msg_.name[ros_joint_id] = dof_name.str();
	      pub_joint_state_.msg_.position[ros_joint_id] =
		current_configuration(joint_position + dof_id);
	      pub_joint_state_.msg_.velocity[ros_joint_id] =
		current_velocity(joint_position + dof_id);
	      ros_joint_id++;
	    }
	  }
	  else if ((*joint_it)->numberDof() == 1) {
	    pub_joint_state_.msg_.name[ros_joint_id] = (*joint_it)->getName();
	    pub_joint_state_.msg_.position[ros_joint_id] = current_configuration(joint_position);
	    pub_joint_state_.msg_.velocity[ros_joint_id] = current_velocity(joint_position);
	    ros_joint_id++;
	  }
	}
      }
      pub_joint_state_.msg_.header.stamp = ros::Time::now();
      pub_joint_state_.unlockAndPublish();
    }
    
  }
  
  void ControllerManager::commandCB(const JrlControlConstPtr& msg)
  {
    assert(msg->torques.size() == command_.size());

    for(unsigned int i = 0; i< msg->torques.size(); ++i) {
      command_[i] = msg->torques[i];
    }
  }
  
  void ControllerManager::ControllerManagerROSThread()
  {
    ROS_INFO_STREAM("Callback thread id=" << boost::this_thread::get_id());
    ros::spin();
  }
  	  
}
