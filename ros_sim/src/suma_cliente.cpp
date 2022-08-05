#include "ros/ros.h"
#include "custom_pkg/suma.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suma_cliente");
  if (argc != 3)
  {
    ROS_INFO("uso: suma_cliente X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<custom_pkg::suma>("suma");
  custom_pkg::suma srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Suma: %ld", (long int)srv.response.suma);
  }
  else
  {
    ROS_ERROR("Falló llamada a servicio suma");
    return 1;
  }

  return 0;
}
