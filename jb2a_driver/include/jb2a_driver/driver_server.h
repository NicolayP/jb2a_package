#ifndef __JB2A_MSGS_DRIVEACTION__
#define __JB2A_MSGS_DRIVEACTION__

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <jb2a_msgs/driveAction.h>

class DriverServer{
 public:
  DriverServer(std::string name);
  ~DriverServer(void);
  void executeCB(const jb2a_msgs::driveGoalConstPtr &goal);
  
 protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<jb2a_msgs::driveAction> as_;
  std::string name_;
  jb2a_msgs::driveFeedback asf_;
  jb2a_msgs::driveResult asr_;
};

#endif //__JB2A_MSGS_DRIVEACTION__

