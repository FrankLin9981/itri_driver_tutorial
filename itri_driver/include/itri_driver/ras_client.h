#ifndef RAS_CLIENT_H
#define RAS_CLIENT_H

#ifndef FLATHEADERS
#include "simple_message/socket/tcp_socket.h"
#else
#include "tcp_socket.h"
#endif

namespace itri_driver
{
namespace ras_client
{

class RAS_Client : public industrial::tcp_socket::TcpSocket
{
public:

  RAS_Client();

  ~RAS_Client();

  bool init(char *buff, int port_num);

  bool makeConnect();

  int rawSendBytes(char *buffer,
    industrial::shared_types::shared_int num_bytes);
  
  int rawReceiveBytes(char *buffer,
      industrial::shared_types::shared_int num_bytes);

  int getCurDeg(std::vector<double>& pdAngle);

  int getRunStatus(int *pnStatus);

private:
  
  std::string robot_ip_;
  std::string local_ip_;

};

} //ras_client
} //itri_driver

#endif
