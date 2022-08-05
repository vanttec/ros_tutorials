#ifndef DR_DYNAMIC_MODEL
#define DR_DYNAMIC_MODEL

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <math.h>
#include <eigen3/Eigen/Dense>

class DifferentialRobot{
    private:
        // ROS
        ros::NodeHandle nh_;
        ros::Publisher pose_publisher_;
        tf2_ros::TransformBroadcaster br_;
        geometry_msgs::Pose pose_;

        float time_step_;

        float m_;        // Mass
        float Iz_;       // Moment of inertia in z axis
        float a_;        // Distance to geometric center

        /* Differential robot model */

        // Inertial frame
        float theta_;
        float w_prev_;
        Eigen::Vector2f pos_;
        Eigen::Vector2f pos_dot_;
        Eigen::Vector2f pos_dot_prev_;
        Eigen::Vector2f pos_dot_dot_;
        Eigen::Vector2f pos_dot_dot_prev_;

        // Non-inertial frame
        Eigen::Vector2f vel_;    // [v, w]
        Eigen::Vector2f vel_dot_;

        // Input control
        Eigen::Vector2f T_;      //[T1; T2]
        Eigen::Vector2f u_;
        float T1_;
        float T2_;

        Eigen::Matrix2f M_;
        Eigen::Matrix2f C_;

        /* PID Control for the robot*/

        // Setpoints
        Eigen::Vector2f pos_d_;
        Eigen::Vector2f pos_dot_d_;
        Eigen::Vector2f pos_dot_dot_d_;

        // Gains
        float kp_x_;
        float kd_x_;
        float kp_y_;
        float kd_y_;

        // Errors
        Eigen::Vector2f e1_;
        Eigen::Vector2f e1_prev_;
        Eigen::Vector2f e2_;
        Eigen::Vector2f e2_prev_;

    public:
        // Default constructor
        DifferentialRobot(const float& time_step, const ros::NodeHandle& nh);

        // Destructor
        ~DifferentialRobot();

        // Methods
        void CalculateForwardKinematics();
        void CalculateForwardDynamics();

        void SetPDSetpoints(std::vector<float>& pos_d, std::vector<float>& pos_dot_d, std::vector<float>& pos_dot_dot_d);
        void SetPDGains(std::vector<float>& k_x, std::vector<float>& k_y);

        void CalculateManipulation();

        void PublishPose();
};
#endif