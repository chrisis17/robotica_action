#include <ros/ros.h>
#include <std_msgs/Duration.h>
#include <iostream>
#include <sstream>
#include <actionlib/client/simple_action_client.h>
#include <action_example/TimerAction.h>
#include <action_example/TimerGoal.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "action_client");

  // create the action client, true causes the client to spin its own thread
  actionlib::SimpleActionClient<action_example::TimerAction> ac("timer", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  action_example::TimerGoal goal;
  goal.time_to_wait = ros::Duration(5);
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  ROS_INFO("Timer elapsed: %f",ac.getResult()->time_elapsed.toSec());
  ROS_INFO("Feedback :  %u",ac.getResult()->updates_sent);
  //exit
  return 0;
}