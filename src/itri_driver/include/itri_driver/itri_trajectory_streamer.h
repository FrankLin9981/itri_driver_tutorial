#ifndef ITRI_TRAJECTORY_STREAMER_H
#define ITRI_TRAJECTORY_STREAMER_H

#include <boost/thread/thread.hpp>
#include <industrial_msgs/RobotStatus.h>
#include "industrial_robot_client/joint_trajectory_interface.h"
#include "itri_driver/ras_client.h"


namespace itri_driver
{
namespace itri_trajectory_streamer
{

namespace StandardSocketPorts = industrial::simple_socket::StandardSocketPorts;
using industrial_robot_client::joint_trajectory_interface::JointTrajectoryInterface;
using industrial::joint_traj_pt_message::JointTrajPtMessage;
using industrial::smpl_msg_connection::SmplMsgConnection;
using itri_driver::ras_client::RAS_Client;

namespace TransferStates
{
enum TransferState
{
  IDLE = 0, STREAMING =1, WAITING = 2 //,STARTING, //, STOPPING
};
}
typedef TransferStates::TransferState TransferState;

/**
 * \brief Message handler that streams joint trajectories to the robot controller
 */

//* JointTrajectoryStreamer
/**
 *
 * THIS CLASS IS NOT THREAD-SAFE
 *
 */
class ITRI_JointTrajectoryStreamer : public JointTrajectoryInterface
{

public:

  // since this class defines a different init(), this helps find the base-class init()
  using JointTrajectoryInterface::init;

  /**
   * \brief Default constructor
   *
   * \param min_buffer_size minimum number of points as required by robot implementation
   */
  ITRI_JointTrajectoryStreamer(int min_buffer_size = 1) : min_buffer_size_(min_buffer_size) {};
  ITRI_JointTrajectoryStreamer(std::string ns="", std::string name="", std::vector<std::string> joints=std::vector<std::string>(), int min_buffer_size = 1)
                              : ns_(ns), name_(name), joint_names_(joints), min_buffer_size_(min_buffer_size) {};

  /**
   * \brief Class initializer
   *
   * \param connection simple message connection that will be used to send commands to robot (ALREADY INITIALIZED)
   * \param joint_names list of expected joint-names.
   *   - Count and order should match data sent to robot connection.
   *   - Use blank-name to insert a placeholder joint position (typ. 0.0).
   *   - Joints in the incoming JointTrajectory stream that are NOT listed here will be ignored.
   * \param velocity_limits map of maximum velocities for each joint
   *   - leave empty to lookup from URDF
   * \return true on success, false otherwise (an invalid message type)
   */
  virtual bool init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                    const std::map<std::string, double> &velocity_limits = std::map<std::string, double>()) override;
  virtual bool init(std::string default_ip = "", int default_port = StandardSocketPorts::MOTION) override;
  virtual bool init(SmplMsgConnection* connection) override;

  ~ITRI_JointTrajectoryStreamer();

  void robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg);

  virtual void jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg) override;
  
  virtual void jointStateCB(const sensor_msgs::JointStateConstPtr &msg) override;

  bool trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<std::string>* msgs);
  
  std::string create_message(int seq, std::vector<double> joint_pos, double velocity, double duration);

  void streamingThread();

  virtual bool send_to_robot(const std::vector<JointTrajPtMessage>& messages) {}

  bool send_to_robot(const std::vector<std::string>& messages);

protected:

  void trajectoryStop();

  ros::Subscriber sub_robot_status_;
  industrial_msgs::RobotStatusConstPtr last_robot_status_;
  RAS_Client ras_client_;
  boost::thread* streaming_thread_;
  boost::mutex mutex_;
  int current_point_;
  std::vector<std::string> current_traj_;
  TransferState state_;
  ros::Time streaming_start_;
  int min_buffer_size_;
  std::string ns_;
  std::string name_;
  std::vector<std::string> joint_names_;
};

} //itri_trajectory_streamer
} //itri_driver

#endif /* ITRI_TRAJECTORY_STREAMER_H */
