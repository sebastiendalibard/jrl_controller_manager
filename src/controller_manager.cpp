#include <jrl/mal/matrixabstractlayer.hh>
#include "jrl_controller_manager/controller_manager.h"

using namespace ros;
namespace jrl_controller_manager {
  ControllerManager::ControllerManager(CjrlDynamicRobot * in_robot,const ros::NodeHandle& nh):
    robot_(in_robot),
    controller_node_(nh),
    pub_joint_state_(nh,"joint_states",10),
    pub_free_flyer_state_(nh,"free_flyer_state",10),
    last_published_joint_state_(ros::Time::now()),
    publish_period_joint_state_(0)
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
    unsigned int joints_size = actuated_joint_vector.size();

    joint_position_in_configuration_vector.resize(joints_size);

    for(unsigned int i = 0; i < joints_size; ++i) {
      int found = 0;
      int position = 0;
      while (actuated_joint_vector[i] != joint_vector[found]) {
	position += joint_vector[found]->numberDof ();
	++found;
      }
      joint_position_in_configuration_vector[i] = position;
    }
    std::cout << "Number of actuated joints: " << joints_size << std::endl;
    for(unsigned int i = 0; i < joints_size; ++i) {
      std::cout << "Position of actuated joint " << i 
		<< " in joint vector: " << joint_position_in_configuration_vector[i]
		<< std::endl;
    }

    pub_joint_state_.msg_.name.resize(joints_size);
    pub_joint_state_.msg_.position.resize(joints_size);
    pub_joint_state_.msg_.velocity.resize(joints_size);
    pub_joint_state_.msg_.effort.resize(0);

    pub_free_flyer_state_.msg_.position.resize(6);
    pub_free_flyer_state_.msg_.velocity.resize(6);

    command_.resize(joints_size);
    for (unsigned int i = 0; i < joints_size; ++i)
      command_[i] = 0;

    controller_node_.subscribe<JrlControl>("command",1,&ControllerManager::commandCB,this);

    ros_spinner_thread_ =  
      boost::thread( boost::bind( &ControllerManager::ControllerManagerROSThread,this ) );
    
    publish_period_joint_state_ = Duration(0.005);
    
    return true;
  }

  void ControllerManager::update()
  {
    ros::Rate loop_rate(200);

    //Publish state
    publishRobotState();

    //Get command
    //ros::spinOnce();
    //loop_rate.sleep();
  }

  void ControllerManager::publishRobotState()
  {
    std::vector<CjrlJoint *> joint_vector = robot_->getActuatedJoints();
    unsigned int joints_size = joint_vector.size();

    vectorN current_configuration = robot_->currentConfiguration();
    vectorN current_velocity = robot_->currentVelocity();
  
    ros::Time now = ros::Time::now();
    if (now > last_published_joint_state_ + publish_period_joint_state_) {
      if (pub_joint_state_.trylock()) {

	while (last_published_joint_state_ + publish_period_joint_state_ < now)
	  last_published_joint_state_ = 
	    last_published_joint_state_ + publish_period_joint_state_;
      
	for (unsigned int i = 0; i <  joints_size; ++i) {
	  CjrlJoint* current_joint = joint_vector[i];
	  unsigned int current_position = joint_position_in_configuration_vector[i];
	  pub_joint_state_.msg_.name[i] = current_joint->getName();
	  pub_joint_state_.msg_.position[i] = current_configuration(current_position);
	  pub_joint_state_.msg_.velocity[i] = current_velocity(current_position);
	}
      }
      pub_joint_state_.msg_.header.stamp = ros::Time::now();
      pub_joint_state_.unlockAndPublish();
      if (pub_free_flyer_state_.trylock()) {
	for (unsigned int i = 0; i < 6; ++i) {
	  pub_free_flyer_state_.msg_.position[i] = current_configuration[i];
	  pub_free_flyer_state_.msg_.velocity[i] = current_velocity[i];
	}
      }
      pub_free_flyer_state_.msg_.header.stamp = ros::Time::now();
      pub_free_flyer_state_.unlockAndPublish();
 
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
