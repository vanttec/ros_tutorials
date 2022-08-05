#include "ros/ros.h"
#include "custom_pkg/suma.h"

bool add(custom_pkg::suma::Request  &req,
         custom_pkg::suma::Response &res)
{
 res.suma = req.a + req.b;
 ROS_INFO("peticion: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
 ROS_INFO("mandando respuesta: [%ld]", (long int)res.suma);
 return true;
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "suma_server");
 ros::NodeHandle n;

 ros::ServiceServer service = n.advertiseService("suma", add);
 ROS_INFO("Listo para sumar enteros.");
 ros::spin();

 return 0;
}