#ifndef ITRI_STATE_INTERFACE_H
#define ITRI_STATE_INTERFACE_H

#include <vector>
#include <string>
#include "simple_message/smpl_msg_connection.h"
#include "simple_message/simple_comms_fault_handler.h"
#include "control_msgs/FollowJointTrajectoryFeedback.h"
#include "sensor_msgs/JointState.h"
#include "industrial_msgs/RobotStatus.h"
#include "itri_driver/ras_client.h"

namespace itri_driver
{
namespace itri_state_interface
{

using industrial::smpl_msg_connection::SmplMsgConnection;
using industrial::simple_comms_fault_handler::SimpleCommsFaultHandler;
using itri_driver::ras_client::RAS_Client;
namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;

/**
 * \brief Generic template that reads state-data from a robot controller
 * and publishes matching messages to various ROS topics.
 *
 * Users should replace the default class members
 * to implement robot-specific behavior.
 */
//* RobotStateInterface
class ITRI_RobotStateInterface
{

public:

  ITRI_RobotStateInterface();

  /**
   * \brief Initialize robot connection using default method.
   *
   * \param default_ip default IP address to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "robot_ip_address" cannot be read
   * \param default_port default port to use for robot connection [OPTIONAL]
   *                    - this value will be used if ROS param "~port" cannot be read
   *
   * \return true on success, false otherwise
   */
  bool init(std::string default_ip = "", int default_port = StandardSocketPorts::STATE);


  /**
   * \brief Initialize robot connection using specified method.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection);

  /**
   * \brief Initialize robot connection using specified method and joint-names.
   *
   * \param connection new robot-connection instance (ALREADY INITIALIZED).
   * \param joint_names list of joint-names for ROS topic
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to skip (not publish) a joint-position
   *
   * \return true on success, false otherwise
   */
  bool init(SmplMsgConnection* connection, std::vector<std::string>& joint_names);

  /**
   * \brief Begin processing messages and publishing topics.
   */
  void run();

  /**
   * \brief get current robot-connection instance.
   *
   * \return current robot connection object
   */
  SmplMsgConnection* get_connection()
  {
    return this->connection_;
  }

  std::vector<std::string> get_joint_names()
  {
    return this->joint_names_;
  }

  void spinonce();

  bool create_message(std::vector<double> pdAngle,
                      control_msgs::FollowJointTrajectoryFeedback* control_state,
                      sensor_msgs::JointState* sensor_state);

protected:
  RAS_Client ras_client_;
  ros::Publisher pub_joint_control_state_;
  ros::Publisher pub_joint_sensor_state_;
  ros::Publisher pub_robot_status_;
  ros::NodeHandle node_;

  SimpleCommsFaultHandler def_comms_hndlr_;
  SmplMsgConnection* connection_;
  std::vector<std::string> joint_names_;

};//class ITRI_RobotStateInterface

}//itri_state_interface
}//itri_driver


#endif /* ITRI_STATE_INTERFACE_H */
