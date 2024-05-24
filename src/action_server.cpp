#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <iostream>
#include <sstream>
#include <actionlib/server/simple_action_server.h>
#include <action_example/TimerAction.h>

class TimerAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<action_example::TimerAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  action_example::TimerFeedback feedback_;
  action_example::TimerResult result_;

  int goal;

public:

  TimerAction(std::string name) :
    as_(nh_, name, boost::bind(&TimerAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.registerPreemptCallback(boost::bind(&TimerAction::preemptCB, this));
    as_.start();
  }

  ~TimerAction(void)
  {
  }

  void preemptCB()
  {
    ROS_WARN("%s got preempted!", action_name_.c_str());
    result_.time_elapsed = ros::Duration(0);
    as_.setPreempted(result_,"I got Preempted");
  }

  void executeCB(const action_example::TimerGoalConstPtr &goal)
  {
    ros::Duration goal_time = goal->time_to_wait;
    ROS_INFO("%s is processing the goal %f", action_name_.c_str(), goal_time.toSec());

    // start executing the action
    double start_time =ros::Time::now().toSec();
    ros::Duration(goal_time).sleep();
    double end_time =ros::Time::now().toSec();
    result_.time_elapsed = ros::Duration(end_time - start_time);
    result_.updates_sent = 0;
    ROS_INFO("%s: Succeeded", action_name_.c_str());
    
    // set the action state to succeeded
    as_.setSucceeded(result_);
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer");
  ROS_INFO("Starting Demo Action Server");
  TimerAction timer(ros::this_node::getName());
  ros::spin();

  return 0;
}