#!/usr/bin/env python3
import rospy
import time
import actionlib
from action_example.msg import TimerAction, TimerGoal, TimerResult

def do_timer(goal):
  start_time = time.time()
  time.sleep(goal.time_to_wait.to_sec())
  result = TimerResult()
  result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
  result.updates_sent = 0
  server.set_succeeded(result)

if __name__=="__main__":
  rospy.init_node('timer_action_server')
  server = actionlib.SimpleActionServer('timer', TimerAction, do_timer, False)
  server.start()
  print("Action Server has started")
  rospy.spin()
