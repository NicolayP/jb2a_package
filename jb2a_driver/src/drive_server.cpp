/**
 * @Author: Nicolay Pierre <pierre>
 * @Date:   2019-06-21T13:10:33+02:00
 * @Email:  nicolaypierre.95@gmail.com
 * @Project: JB2A
 * @Filename: drive_server.cpp
 * @Last modified by:   pierre
 * @Last modified time: 2019-06-21T13:18:31+02:00
 */



#include <jb2a_driver/driver_server.h>
#include <ros/console.h>

DriverServer::DriverServer(std::string name) :
  name_(name),
  as_(nh_, name, boost::bind(&DriverServer::executeCB, this, _1), false)
{
  // Set logger level to debug.
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    ros::console::notifyLoggerLevelsChanged();

  ROS_DEBUG_STREAM("Start as server.");
  ROS_INFO_STREAM("Starting drive server :" << name_);
  as_.start();
}

DriverServer::~DriverServer(void){}

void DriverServer::executeCB(const jb2a_msgs::driveGoalConstPtr &goal){
  bool success = true;

  ROS_DEBUG_STREAM(name_ << " : goal recieved");
}

int main(int argc, char **argv){
  ros::init(argc, argv, "drive");
  DriverServer ds(ros::this_node::getName());
  ros::spin();
  return 0;
}
