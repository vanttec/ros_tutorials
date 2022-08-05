#include "differential_robot.hpp"

int main(int argc, char** argv){
    // PD Gains
    std::vector<float> k_x = {1, 1, 1};
    std::vector<float> k_y = {1, 1, 1};

    // Setpoints
    // std::vector<float> pos_d = {strtof(argv[1],NULL), strtof(argv[2],NULL)};
    std::vector<float> pos_d = {0, 0};
    std::vector<float> pos_dot_d = {0, 0};
    std::vector<float> pos_dot_dot_d = {0, 0};

    const float TIME_STEP = 0.01;

    // if (argc != 3) {
    //     ROS_INFO("Uso: rosrun custom_pkg diff_robot_node X Y");
    //     return 1;
    // }

    ros::init(argc, argv, "differential_robot");
    ros::NodeHandle nh;
    ros::Rate rate(1/TIME_STEP);
    double start = ros::Time::now().toSec();
    // std::cout << start << std::endl;

    DifferentialRobot robot(TIME_STEP, nh);

    robot.SetPDGains(k_x, k_y);

    while(ros::ok()) {
        pos_d[0] = 2*sin(0.5*(ros::Time::now().toSec() - start));
        pos_d[1] = 2*cos(0.5*(ros::Time::now().toSec() - start));

        pos_dot_d[0] = cos(0.5*(ros::Time::now().toSec() - start));
        pos_dot_d[1] =-sin(0.5*(ros::Time::now().toSec() - start));

        // std::cout << ros::Time::now().toSec() - start << std::endl;

        robot.SetPDSetpoints(pos_d, pos_dot_d, pos_dot_dot_d);

        robot.CalculateForwardDynamics();
        robot.CalculateManipulation();
        robot.PublishPose();
        rate.sleep();
    }
    return 0;
}