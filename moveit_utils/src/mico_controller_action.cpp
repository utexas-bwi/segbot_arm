#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit_utils/MicoController.h>
#include <moveit_utils/FollowVelocityTrajectoryAction.h>



class TrajectoryAction
{
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<moveit_utils::FollowVelocityTrajectoryAction> as_; 
  std::string action_name_;
  ros::ServiceClient client;
  // create messages that are used to published feedback/result
  moveit_utils::FollowVelocityTrajectoryFeedback feedback_;
  moveit_utils::FollowVelocityTrajectoryResult result_;

  //learning_actionlib::FibonacciFeedback feedback_;
  //learning_actionlib::FibonacciResult result_;

public:

  TrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&TrajectoryAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
     client = nh_.serviceClient<moveit_utils::MicoController>("mico_controller");
  }

  ~TrajectoryAction(void)
  {
  }

  void executeCB(const moveit_utils::FollowVelocityTrajectoryGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    moveit_utils::MicoController srv;
    srv.request.trajectory = goal->trajectory;
    if(client.call(srv)){
      ROS_INFO("Mico controller action made service request.");
    }
    else{
      ROS_ERROR("Mico controller action couldn't contact MicoController service. Is it running?");
    }
    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

    // start executing the action
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
      }
      feedback_.done = true;
      //feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);

    if(success){
      result_.done = feedback_.done;
      as_.setSucceeded(result_);
    }
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mico_controller_action_server");

  TrajectoryAction traj(ros::this_node::getName());
  ros::spin();

  return 0;
}