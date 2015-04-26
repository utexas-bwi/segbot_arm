/* Author: Ioan Sucan */
// edits: Maxwell J Svetlik
#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include <pluginlib/class_list_macros.h>
#include <map>

namespace moveit_controller_manager_example
{

class ExampleControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  ExampleControllerHandle(const std::string &name, ros::NodeHandle nh) : moveit_controller_manager::MoveItControllerHandle(name), nh_(nh)
  {
  }
  
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &t)
  {
    // do whatever is needed to actually execute this trajectory
    return true;
  }
  
  virtual bool cancelExecution()
  {   
    // do whatever is needed to cancel execution 
    return true;
  }
  
  virtual bool waitForExecution(const ros::Duration &)
  {
    // wait for the current execution to finish
    return true;
  }
  
  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return moveit_controller_manager::ExecutionStatus(moveit_controller_manager::ExecutionStatus::SUCCEEDED);
  }
  private:
	ros::NodeHandle nh_;
};


class MoveItControllerManagerExample : public moveit_controller_manager::MoveItControllerManager
{
public:

  MoveItControllerManagerExample()
  {
  }
  
  virtual ~MoveItControllerManagerExample()
  {
  }

  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
	  ros::NodeHandle n;
    return moveit_controller_manager::MoveItControllerHandlePtr(new ExampleControllerHandle(name, n));
  }
  
  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names)
  {
    names.resize(1);
    names[0] = "my_example_controller";
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal with it anyways!
   */
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    joints.clear();
    if (name == "my_example_controller")
    {
      // declare which joints this controller actuates
      joints.push_back("joint1");
      joints.push_back("joint2");
      joints.push_back("joint3");
      joints.push_back("joint4");
      joints.push_back("joint5");
      joints.push_back("joint6");
    }
  }

  /*
   * Controllers are all active and default.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) { return false; }

protected:

  ros::NodeHandle node_handle_;
  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controllers_;
};

} // end namespace moveit_controller_manager_example

PLUGINLIB_EXPORT_CLASS(moveit_controller_manager_example::MoveItControllerManagerExample,
                       moveit_controller_manager::MoveItControllerManager);
