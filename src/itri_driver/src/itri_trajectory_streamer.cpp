#include "itri_driver/itri_trajectory_streamer.h"
#include "industrial_utils/param_utils.h"

#define RAD2DEG(rad) rad*180/M_PI

using namespace industrial_utils::param;
using industrial::simple_message::SimpleMessage;
typedef trajectory_msgs::JointTrajectoryPoint  ros_JointTrajPt;

namespace itri_driver
{
namespace itri_trajectory_streamer
{

bool ITRI_JointTrajectoryStreamer::init(std::string default_ip, int default_port)
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
  ROS_INFO("Joint Trajectory Interface connecting to IP address: '%s:%d'", ip_addr, port);
  ras_client_.init(ip_addr, port);
  free(ip_addr);

  return init(&ras_client_);
}

bool ITRI_JointTrajectoryStreamer::init(SmplMsgConnection* connection)
{
  std::vector<std::string> joint_names;
  if (!getJointNames("controller_joint_names", "robot_description", joint_names))
  {
    ROS_ERROR("Failed to initialize joint_names.  Aborting");
    return false;
  }

  return init(connection, joint_names);
}

bool ITRI_JointTrajectoryStreamer::init(SmplMsgConnection* connection, const std::vector<std::string> &joint_names,
                                   const std::map<std::string, double> &velocity_limits)
{
  bool rtn = true;

  ROS_INFO("JointTrajectoryStreamer: init");

  rtn &= JointTrajectoryInterface::init(connection, joint_names, velocity_limits);
  this->sub_robot_status_ = node_.subscribe("robot_status", 1, &ITRI_JointTrajectoryStreamer::robotStatusCB, this);

  this->mutex_.lock();
  this->current_point_ = 0;
  this->state_ = TransferStates::IDLE;
  this->streaming_thread_ =
      new boost::thread(boost::bind(&ITRI_JointTrajectoryStreamer::streamingThread, this));
  ROS_INFO("Unlocking mutex");
  this->mutex_.unlock();

  return rtn;
}

ITRI_JointTrajectoryStreamer::~ITRI_JointTrajectoryStreamer()
{
  delete this->streaming_thread_;
}

void ITRI_JointTrajectoryStreamer::robotStatusCB(const industrial_msgs::RobotStatusConstPtr &msg)
{
  last_robot_status_ = msg; //caching robot status for later use.
}

void ITRI_JointTrajectoryStreamer::jointTrajectoryCB(const trajectory_msgs::JointTrajectoryConstPtr &msg)
{
  ROS_INFO("Receiving joint trajectory message");

  // read current state value (should be atomic)
  int state = this->state_;

  ROS_DEBUG("Current state is: %d", state);
  if (TransferStates::IDLE != state)
  {
    if (msg->points.empty())
      ROS_INFO("Empty trajectory received, canceling current trajectory");
    else
      ROS_ERROR("Trajectory splicing not yet implemented, stopping current motion.");

	this->mutex_.lock();
    trajectoryStop();
	this->mutex_.unlock();
    return;
  }

  if (msg->points.empty())
  {
    ROS_INFO("Empty trajectory received while in IDLE state, nothing is done");
    return;
  }

  // calc new trajectory
  // std::vector<JointTrajPtMessage> new_traj_msgs;
  std::vector<std::string> new_traj_msgs;
  if (!trajectory_to_msgs(msg, &new_traj_msgs))
    return;

  // send command messages to robot
  send_to_robot(new_traj_msgs);
}

bool ITRI_JointTrajectoryStreamer::send_to_robot(const std::vector<std::string>& messages)
{
  ROS_INFO("Loading trajectory, setting state to streaming");
  this->mutex_.lock();
  {
    ROS_INFO("Executing trajectory of size: %d", (int)messages.size());
    this->current_traj_.assign(messages.begin(), messages.end());
    this->current_point_ = 0;
    this->state_ = TransferStates::STREAMING;
    this->streaming_start_ = ros::Time::now();
  }
  this->mutex_.unlock();

  return true;
}


bool ITRI_JointTrajectoryStreamer::trajectory_to_msgs(const trajectory_msgs::JointTrajectoryConstPtr &traj, std::vector<std::string>* msgs)
{
  msgs->clear();
  
  // check for valid trajectory
  if (!is_valid(*traj))
    return false;

  for (size_t i = 0; i < traj->points.size(); ++i)
  {
    ros_JointTrajPt rbt_pt, xform_pt;
    double vel, duration;

    // select / reorder joints for sending to robot
    if (!select(traj->joint_names, traj->points[i], this->all_joint_names_, &rbt_pt))
      return false;

    // transform point data (e.g. for joint-coupling)
    if (!transform(rbt_pt, &xform_pt))
      return false;

    // reduce velocity to a single scalar, for robot command
    if (!calc_speed(xform_pt, &vel, &duration))
      return false;

    std::string msg = create_message(i, xform_pt.positions, vel, duration);
    msgs->push_back(msg);
  }

  return true;
}

std::string ITRI_JointTrajectoryStreamer::create_message(int seq, std::vector<double> joint_pos, double velocity, double duration)
{
  industrial::joint_data::JointData pos;
  ROS_ASSERT(joint_pos.size() <= (unsigned int)pos.getMaxNumJoints());

  std::string msg = "MOVJ ";

  for (size_t i = 0; i < joint_pos.size(); ++i) {
    msg += std::to_string(RAD2DEG(joint_pos[i]));
    if(i != (joint_pos.size() - 1))
      msg += " ";
    // msg += " ";
  }

  // msg += "WITH_OUTPUT 10000 ON ";
  
  return msg;
}

void ITRI_JointTrajectoryStreamer::streamingThread()
{
  std::string jtpMsg = "";
  int connectRetryCount = 1;
  int timoutCount = 0;

  ROS_INFO("Starting joint trajectory streamer thread");
  while (ros::ok())
  {
    ros::Duration(0.005).sleep();

    // automatically re-establish connection, if required
    if (connectRetryCount-- > 0)
    {
      ROS_INFO("Connecting to robot motion server");
      this->connection_->makeConnect();
      ros::Duration(0.250).sleep();  // wait for connection

      if (this->connection_->isConnected())
        connectRetryCount = 0;
      else if (connectRetryCount <= 0)
      {
        ROS_ERROR("Timeout connecting to robot controller. Send new motion command to retry.");
        this->state_ = TransferStates::IDLE;
      }
      continue;
    }

    this->mutex_.lock();

    char* recvBuff = new char [4096];

    switch (this->state_)
    {
      case TransferStates::IDLE:
        ros::Duration(0.010).sleep();  //  loop while waiting for new trajectory
        break;
      case TransferStates::WAITING:        
        ros::Duration(0.5).sleep();
        if(this->last_robot_status_->in_motion.val == industrial_msgs::TriState::FALSE)
        {
          ROS_INFO("Robot is not in motion, setting state to IDLE");
          this->state_ = TransferStates::IDLE;
        }
        break;
      case TransferStates::STREAMING:
        if (this->current_point_ >= (int)this->current_traj_.size())
        {
          ROS_INFO("Trajectory streaming complete, setting state to WAITING");
          this->state_ = TransferStates::WAITING;
          break;
        }

        if (!this->connection_->isConnected())
        {
          ROS_DEBUG("Robot disconnected.  Attempting reconnect...");
          connectRetryCount = 5;
          break;
        }

        // Send these waypoints all at once
        for(int i = 0; i < (int)this->current_traj_.size(); ++i) {
          jtpMsg += this->current_traj_[this->current_point_];
          ++this->current_point_;
          if(this->current_point_ != (int)this->current_traj_.size())
            jtpMsg.back() = '\n';
          ROS_INFO("Point[%d of %d] packed into message",
                    this->current_point_, (int)this->current_traj_.size());
        }

        if(this->ras_client_.rawSendBytes((char*)jtpMsg.c_str(), jtpMsg.size()+1))
        {
          ROS_INFO("Trajectory sent to controller");
          // May need to set a timeout
          /*
          if(this->ras_client_.rawReceiveBytes(recvBuff, 4096)) {
            ROS_DEBUG("Receive %s from controller", recvBuff);
          }
          else
            ROS_WARN("Controller does not response");
          */
        }
        else
          ROS_WARN("Failed sent joint point, will try again");
        
        jtpMsg.clear();
        break;

      default:
        ROS_ERROR("Joint trajectory streamer: unknown state");
        this->state_ = TransferStates::IDLE;
        break;
    }
    
    delete [] recvBuff;
    this->mutex_.unlock();
  }

  ROS_WARN("Exiting trajectory streamer thread");
}

void ITRI_JointTrajectoryStreamer::trajectoryStop()
{
  ROS_INFO("Joint trajectory handler: entering stopping state");
  char* recvBuff = new char [4096];
  std::string stopMsg = "STOP";

  if(this->ras_client_.rawSendBytes((char*)stopMsg.c_str(), stopMsg.size()+1))
  {
    ROS_INFO("Stop signal sent to controller");
    // May need to set a timeout
    if(this->ras_client_.rawReceiveBytes(recvBuff, 4096)) {
      ROS_DEBUG("Receive %s from controller", recvBuff);
    }
    else
      ROS_WARN("Controller does not response");
  }
  else
    ROS_WARN("Failed sent stop signal");


  ROS_DEBUG("Stop command sent, entering idle mode");
  this->state_ = TransferStates::IDLE;

  delete [] recvBuff;
}

} //itri_trajectory_streamer
} //itri_driver

using itri_driver::itri_trajectory_streamer::ITRI_JointTrajectoryStreamer;

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "itri_motion_interface");

  // launch the default JointTrajectoryStreamer connection/handlers
  ITRI_JointTrajectoryStreamer motionInterface;
  motionInterface.init();
  motionInterface.run();

  return 0;
}
