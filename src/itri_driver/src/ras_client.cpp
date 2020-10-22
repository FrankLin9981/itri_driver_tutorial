#include "itri_driver/ras_client.h"
#include "simple_message/log_wrapper.h"

#define DEG2RAD(deg) M_PI*deg/180

using namespace industrial::smpl_msg_connection;
using namespace industrial::byte_array;
using namespace industrial::simple_message;
using namespace industrial::shared_types;

namespace itri_driver
{
namespace ras_client
{

RAS_Client::RAS_Client()
{
  this->local_ip_ = "192.168.1.249";
}

RAS_Client::~RAS_Client()
{
  LOG_DEBUG("Destructing RASClient");
}

bool RAS_Client::init(char *buff, int port_num)
{

  int rc;
  bool rtn;
  int disableNodeDelay = 1;
  addrinfo *result;
  addrinfo hints = {};

  rc = SOCKET(AF_INET, SOCK_STREAM, 0);
  if (this->SOCKET_FAIL != rc)
  {
    this->setSockHandle(rc);

    // The set no delay disables the NAGEL algorithm
    rc = SET_NO_DELAY(this->getSockHandle(), disableNodeDelay);
    if (this->SOCKET_FAIL == rc)
    {
      LOG_WARN("Failed to set no socket delay, sending data can be delayed by up to 250ms");
    }

    // Initialize address data structure
    memset(&this->sockaddr_, 0, sizeof(this->sockaddr_));
    this->sockaddr_.sin_family = AF_INET;

    // Check for 'buff' as hostname, and use that, otherwise assume IP address
    hints.ai_family = AF_INET;  // Allow IPv4
    hints.ai_socktype = SOCK_STREAM;  // TCP socket
    hints.ai_flags = 0;  // No flags
    hints.ai_protocol = 0;  // Any protocol
    hints.ai_canonname = NULL;
    hints.ai_addr = NULL;
    hints.ai_next = NULL;
    if (0 == GETADDRINFO(buff, NULL, &hints, &result))
    {
      this->sockaddr_ = *((sockaddr_in *)result->ai_addr);
    }
    else 
    {
      this->sockaddr_.sin_addr.s_addr = INET_ADDR(buff);
    }
    this->sockaddr_.sin_port = HTONS(port_num);

    this->robot_ip_ = std::string(buff);

    rtn = true;

  }
  else
  {
    LOG_ERROR("Failed to create socket, rc: %d", rc);
    rtn = false;
  }
  return rtn;
}

bool RAS_Client::makeConnect()
{
  bool rtn = false;
  int rc = this->SOCKET_FAIL;
  SOCKLEN_T addrSize = 0;

  if (!this->isConnected())
  {
    addrSize = sizeof(this->sockaddr_);
    rc = CONNECT(this->getSockHandle(), (sockaddr *)&this->sockaddr_, addrSize);
    if (this->SOCKET_FAIL != rc)
    {
      LOG_INFO("Connected to server");
      this->setConnected(true);
      rtn = true;
    }
    else
    {
      this->logSocketError("Failed to connect to server", rc, errno);
      rtn = false;
    }
  }

  else
  {
    LOG_WARN("Tried to connect when socket already in connected state");
  }

  return rtn;
}

int RAS_Client::rawSendBytes(char *buffer,
    industrial::shared_types::shared_int num_bytes)
{
   int rc = this->SOCKET_FAIL;

   rc = SEND(this->getSockHandle(), buffer, num_bytes, 0);

   return rc;
}

int RAS_Client::rawReceiveBytes(char *buffer, shared_int num_bytes)
{
  int rc = this->SOCKET_FAIL;
  
  rc = RECV(this->getSockHandle(), buffer, num_bytes, 0);
  
  return rc;
}

int RAS_Client::getCurDeg(std::vector<double>& pdAngle)
{
  int rc = this->SOCKET_FAIL;
  char* recvBuff = new char [4096];
  std::string msg = "NETS_GETDEG " + this->local_ip_;
  std::string temp = "";
  std::string degBuff = "";

  if(this->rawSendBytes((char*)msg.c_str(), msg.size()+1)) {
    ROS_DEBUG("NETS_GETDEG sent to controller");
    // May need to set a timeout
    /*
    if(rc = this->rawReceiveBytes(recvBuff, 4096)) {
      ROS_DEBUG("Receive %s from controller", recvBuff);
      std::string str(recvBuff+4, rc-1);
      // std::string str(recvBuff, rc-1);
      std::stringstream ss(str);
      int count = 0;
      for(double ang; ss >> ang; ) {
        if(count == 6)
          break;        
        pdAngle[count] = DEG2RAD(ang);
        if (ss.peek() == ',')
          ss.ignore();
        count++;
      }
    }
    else
      ROS_WARN("Controller does not response to NETS_GETDEG");
  }
    */
    while(true)
    {
      rc = RECV(this->getSockHandle(), recvBuff, 1, 0);
      if(recvBuff[0] != '\0') {
        temp += recvBuff[0];
      }
      else {
        ROS_DEBUG("Receive %s from controller", temp.c_str());
        if(temp != "IRA") {
          int i = 0;
          for(size_t n = 0; n != temp.size(); n++) {
            if(temp[n] != ' ' && temp[n] != ',')
              degBuff += temp[n];
            else {
              pdAngle[i] = DEG2RAD(std::stod(degBuff));
              degBuff.clear();
              ++i;
            }
          }
          pdAngle[i] = DEG2RAD(std::stod(degBuff));
          break;  
        }
        else
          temp.clear();
      }
    }
  }
  else
    ROS_WARN("Failed sent NETS_GETDEG");

  return rc;
}

int RAS_Client::getRunStatus(int *pnStatus)
{
  int rc = this->SOCKET_FAIL;
  char* recvBuff = new char [4096];
  std::string msg = "NETS_GETRUNSTATUS " + this->local_ip_;
  std::string temp = "";

  if(this->rawSendBytes((char*)msg.c_str(), msg.size()+1)) {
    ROS_DEBUG("NETS_GETRUNSTATUS sent to controller");
    // May need to set a timeout
    /*
    if(rc = this->rawReceiveBytes(recvBuff, 4096)) {
      ROS_DEBUG("Receive %s from controller", recvBuff);
      std::cout << "GetRun: " << recvBuff << std::endl;
      // std::string str(recvBuff+4, rc-1);
      std::string str(recvBuff, rc-1);
      *pnStatus = std::stoi(str);
    }
    else
      ROS_WARN("Controller does not response to NETS_GETRUNSTATUS");
    */
    while(true)
    {
      rc = RECV(this->getSockHandle(), recvBuff, 1, 0);
      if(recvBuff[0] != '\0') {
        temp += recvBuff[0];
      }
      else {
        ROS_DEBUG("Receive %s from controller", temp.c_str());
        if(temp != "IRA") {
          *pnStatus = std::stoi(temp);
          break;  
        }
        else
          temp.clear();
      }
    }
  }
  else
    ROS_WARN("Failed sent NETS_GETRUNSTATUS");

  return rc;
}

} //ras_client
} //itri_driver
