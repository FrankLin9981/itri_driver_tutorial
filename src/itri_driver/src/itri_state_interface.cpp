#include "itri_driver/itri_state_interface.h"
#include "industrial_utils/param_utils.h"

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial_utils::param::getJointNames;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

namespace itri_driver
{
namespace itri_state_interface
{

ITRI_RobotStateInterface::ITRI_RobotStateInterface()
{
  this->connection_ = NULL;
}

bool ITRI_RobotStateInterface::init(std::string default_ip, int default_port)
{
  std::string ip;
  int port;

  // override IP/port with ROS params, if available
  ros::param::param<std::string>("robot_ip_address", ip, default_ip);
  ros::param::param<int>("~port", port, default_port);

  // check for valid parameter values
  if (ip.empty())
  {
    ROS_ERROR("No valid robot IP address found.  Please set ROS 'robot_ip_address' param");
    return false;
  }
  if (port <= 0)
  {
    ROS_ERROR("No valid robot IP port found.  Please set ROS '~port' param");
    return false;
  }

  char* ip_addr = strdup(ip.c_str());  // connection.init() requires "char*", not "const char*"
  ROS_INFO("Robot state connecting to IP address: '%s:%d'", ip_addr, port);
  ras_client_.init(ip_addr, port);
  free(ip_addr);

  return init(&ras_client_);
}

bool ITRI_RobotStateInterface::init(SmplMsgConnection* connection)
{
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", "robot_description", joint_names))
  {
    ROS_ERROR("Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool ITRI_RobotStateInterface::init(SmplMsgConnection* connection, std::vector<std::string>& joint_names)
{
  this->joint_names_ = joint_names;
  this->connection_ = connection;
  connection_->makeConnect();

  this->def_comms_hndlr_.init(connection);
  this->pub_joint_control_state_ =
          this->node_.advertise<control_msgs::FollowJointTrajectoryFeedback>("feedback_states", 1);
  this->pub_joint_sensor_state_ = this->node_.advertise<sensor_msgs::JointState>("joint_states", 1);
  this->pub_robot_status_ = this->node_.advertise<industrial_msgs::RobotStatus>("robot_status", 1);

  return true;
}

bool ITRI_RobotStateInterface::create_message(std::vector<double> pdAngle,
                                              control_msgs::FollowJointTrajectoryFeedback* control_state,
                                              sensor_msgs::JointState* sensor_state)
{
  control_state->header.stamp = ros::Time::now();
  control_state->joint_names = this->joint_names_;
  control_state->actual.positions = pdAngle;

  sensor_state->header.stamp = ros::Time::now();
  sensor_state->name = this->joint_names_;
  sensor_state->position = pdAngle;

  return true;
}

void ITRI_RobotStateInterface::spinonce()
{
  if(!this->get_connection()->isConnected())
  {
    this->def_comms_hndlr_.connectionFailCB();
  }

  // Get joint states
  std::vector<double> curDegs;
  curDegs.resize(this->joint_names_.size());
  if(this->ras_client_.getCurDeg(curDegs))
  {
    control_msgs::FollowJointTrajectoryFeedback control_state;
    sensor_msgs::JointState sensor_state;
    this->create_message(curDegs, &control_state, &sensor_state);
    this->pub_joint_control_state_.publish(control_state);
    this->pub_joint_sensor_state_.publish(sensor_state);
  }
  // Get robot status
  int curStatus;
  if(this->ras_client_.getRunStatus(&curStatus))
  {
    industrial_msgs::RobotStatus robot_status;
    robot_status.header.stamp = ros::Time::now();
    robot_status.in_motion.val = curStatus;
    this->pub_robot_status_.publish(robot_status);
  }
}

void ITRI_RobotStateInterface::run()
{
  ros::Rate r(200);
  while(ros::ok())
  {
    // TODO
    this->spinonce();
    // Throttle loop speed if waiting for a re-connection
    if(!this->get_connection()->isConnected())
      sleep(5);
    
    r.sleep();
  }
}

} // itri_state_interface
} // itri_driver

using itri_driver::itri_state_interface::ITRI_RobotStateInterface;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "state_interface");

  // launch the default Robot State Interface connection/handlers
  ITRI_RobotStateInterface rsi;
  if (rsi.init())
  {
    rsi.run();
  }
  return 0;
}
