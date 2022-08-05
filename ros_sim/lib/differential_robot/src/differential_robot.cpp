#include "differential_robot.hpp"

DifferentialRobot::DifferentialRobot(const float& time_step, const ros::NodeHandle& nh): nh_(nh){
    // ROS Publishers
    pose_publisher_ = nh_.advertise<geometry_msgs::Pose>("/differential_robot/Pose", 10);

    time_step_ = time_step;

    m_ = 1;
    Iz_ = 0.010416667;
    a_ = 0.25;

    /* Differential robot model */
    theta_ = 0;
    w_prev_ = 0;

    pos_ << 0,
            0;
            
    pos_dot_ << 0,
                0;
    pos_dot_prev_ << 0,
                     0;

    pos_dot_dot_ << 0,
                    0;
    pos_dot_dot_prev_ << 0,
                         0;
    
    vel_ << 0,       // v
            0;       // w
    u_ << 0,
          0;

    M_ << 0, 0,
          0, 0;

    C_ << 0, 0,
          0, 0;

    /* PID Control for the robot*/
    pos_d_ << 0,
             0;

    pos_dot_d_ << 0,
                 0;
    pos_dot_dot_d_ << 0,
                     0;

    kp_x_ = 0;
    kd_x_ = 0;
    kp_y_ = 0;
    kd_y_ = 0;

    e1_ << 0,
           0;

    e1_prev_ << 0,
                0;

    e2_ << 0,
           0;

    e2_prev_ << 0,
                0;

}

DifferentialRobot::~DifferentialRobot(){}

void DifferentialRobot::CalculateForwardKinematics(){
    Eigen::Matrix2f R;
    R << cos(theta_), -a_*sin(theta_),
         sin(theta_),  a_*cos(theta_);

    vel_ = R.inverse() * pos_dot_;
}

void DifferentialRobot::CalculateForwardDynamics(){
    float div = Iz_/a_;
    w_prev_ = vel_(1);

    CalculateForwardKinematics();

    M_ << m_*cos(theta_), m_*sin(theta_),
         -div*sin(theta_), div*cos(theta_);
    
    C_ << m_*vel_(1)*sin(theta_), m_*vel_(1)*cos(theta_),
         -div*vel_(1)*cos(theta_), div*vel_(1)*sin(theta_);

    pos_dot_dot_ = M_.inverse() * (T_ - C_*pos_dot_);
    pos_dot_ += (pos_dot_dot_prev_ + pos_dot_dot_) * time_step_/2;
    pos_ += (pos_dot_prev_ + pos_dot_) * time_step_/2;

    theta_ += (vel_(1) + w_prev_)*time_step_/2;

    if(fabs(theta_) > M_PI){
        theta_ = (theta_ / fabs(theta_)) * (fabs(theta_) - 2 * M_PI);
    }
}

void DifferentialRobot::SetPDSetpoints(std::vector<float>& pos_d, std::vector<float>& pos_dot_d, std::vector<float>& pos_dot_dot_d){
    pos_d_ << pos_d[0],
              pos_d[1];

    pos_dot_d_ << pos_dot_d[0],
                  pos_dot_d[1];

    pos_dot_dot_d_ << pos_dot_dot_d[0],
                      pos_dot_dot_d[1];
}

void DifferentialRobot::SetPDGains(std::vector<float>& k_x, std::vector<float>& k_y){
    kp_x_ = k_x[0];
    kd_x_ = k_x[1];
    kp_y_ = k_y[0];
    kd_y_ = k_y[1];
}

void DifferentialRobot::CalculateManipulation(){
    e1_prev_ = e1_;
    e2_prev_ = e2_;
    
    e1_ = pos_d_ - pos_;
    e2_ = pos_dot_d_ - pos_dot_;

    u_ << e1_(0)*kp_x_ + e2_(0)*kd_x_,
          e1_(1)*kp_y_ + e2_(1)*kd_y_;

    T_ << M_*(pos_dot_dot_d_ + u_) + C_*pos_dot_;
}

void DifferentialRobot::PublishPose(){
    tf2::Quaternion q;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    // Update Pose
    pose_.position.x = pos_(0);
    pose_.position.y = pos_(1);
    q.setRPY(0, 0, theta_);
    pose_.orientation.x = q.x();
    pose_.orientation.y = q.y();
    pose_.orientation.z = q.z();
    pose_.orientation.w = q.w();

    // Update transform
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = pos_(0);
    transformStamped.transform.translation.y = pos_(1);
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    pose_publisher_.publish(pose_);
    br_.sendTransform(transformStamped);
}