#include <torch/torch.h>
#include <torch/script.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <builtin_interfaces/msg/duration.hpp>

#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <map>
#include <thread>
#include <fstream>
#include <iostream>
#include <deque>
#define M_PI 3.14159265358979323846

using namespace std::placeholders;
class JointControl: public rclcpp::Node{
public:
    JointControl(): Node("joint_controller"){

        // device_ = torch::cuda::is_available() ? torch::Device(torch::kCUDA) : torch::Device(torch::kCPU);
        RCLCPP_INFO(this->get_logger(), "Initializing Subscribers/Publishers");
        this->declare_parameter("debug", false);
        debug_ = this->get_parameter("debug").as_bool();
        this->declare_parameter("imu_topic_name", "imu");
        std::string imu_topic_name = this->get_parameter("imu_topic_name").as_string();
        this->declare_parameter("cmd_vel_topic_name", "cmd_vel");
        std::string cmd_vel_topic_name = this->get_parameter("cmd_vel_topic_name").as_string();
        this->declare_parameter("joint_state_topic_name", "joint_states");
        std::string joint_state_topic_name = this->get_parameter("joint_state_topic_name").as_string();
        this->declare_parameter("odom_topic_name", "odom");
        std::string odom_topic_name = this->get_parameter("odom_topic_name").as_string();
        this->declare_parameter("joint_command_topic_name", "joint_command");
        std::string joint_command_topic_name = this->get_parameter("joint_command_topic_name").as_string();
        this->declare_parameter("trajectory_time", 0.05);
        trj_time_ = this->get_parameter("trajectory_time").as_double();

        // Subscribers
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic_name, 10, std::bind(&JointControl::cb_imu, this, _1));
        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic_name, 10, std::bind(&JointControl::cb_cmd_vel, this, _1));
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_name, 10, std::bind(&JointControl::cb_odom, this, _1));
        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(joint_state_topic_name, 10, std::bind(&JointControl::cb_joint_states, this, _1));
        
        // Publihser
        pub_target_joint_state_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);
        pub_joint_trajectory_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_command_topic_name, 10);

        // Load URDF and parse joint limits
        this->declare_parameter("robot_description", "");
        std::string robot_description_path = this->get_parameter("robot_description").as_string();
        std::ifstream ifs(robot_description_path);
        std::stringstream buffer;
        buffer << ifs.rdbuf();
        std::string robot_description = buffer.str();
        robot_description_.initString(robot_description);

        for (const auto& joint_pair: robot_description_.joints_){
            auto& joint = joint_pair.second;

            if (joint->type == urdf::Joint::REVOLUTE){
                double lower = joint->limits->lower;
                double upper = joint->limits->upper;
                joint_limits_[joint->name] = std::make_pair(lower, upper);
            }
        }
        RCLCPP_INFO_STREAM(this->get_logger(), "Imported joint limits: \n" << joint_limits_);

        // Load torchscript
        RCLCPP_INFO(this->get_logger(), "Loading torchscript");
        this->declare_parameter("checkpoint_dir", "");

        std::string script_path = this->get_parameter("checkpoint_dir").as_string();
        policy_ = torch::jit::load(script_path, device_);
        policy_.to(device_);
        RCLCPP_INFO(this->get_logger(), "Loaded torchscript: %s", script_path.c_str());
    
        // Set periodic parameters
        phase_ = 0.5;
        left_offset_ = 0.0; right_offset_ = 0.5;
        dt_ = 0.02;
        cycle_length_s_ = 0.5;
        cycle_length_ = static_cast<int>(ceil(cycle_length_s_ / dt_));
        RCLCPP_INFO(this->get_logger(), "Cycle length: %d", cycle_length_);
        phi_ = 0;
        joint_names_ = {
            "j_pan",
            "j_pelvis_l",
            "j_pelvis_r",
            "j_shoulder_l",
            "j_shoulder_r",
            "j_tilt",
            "j_thigh1_l",
            "j_thigh1_r",
            "j_high_arm_l",
            "j_high_arm_r",
            "j_thigh2_l",
            "j_thigh2_r",
            "j_low_arm_l",
            "j_low_arm_r",
            "j_tibia_l",
            "j_tibia_r",
            "j_gripper_l",
            "j_gripper_r",
            "j_ankle1_l",
            "j_ankle1_r",
            "j_ankle2_l",
            "j_ankle2_r",
        };

        m_default_joint_pos_ = {
            {"j_pan", 0.0},
            {"j_tilt", 0.0},
            {"j_pelvis_l", 0.0},
            {"j_thigh1_l", 0.0},
            {"j_thigh2_l", 0.4},
            {"j_tibia_l", -0.6},
            {"j_ankle1_l", -0.2},
            {"j_ankle2_l", 0.0},
            {"j_pelvis_r", 0.0},
            {"j_thigh1_r", 0.0},
            {"j_thigh2_r", -0.4},
            {"j_tibia_r", 0.6},
            {"j_ankle1_r", 0.2},
            {"j_ankle2_r", 0.0},
            {"j_shoulder_l", 0.0},
            {"j_high_arm_l", 0.7},
            {"j_low_arm_l", -1.0},
            {"j_gripper_l", -0.99},
            {"j_shoulder_r", 0.0},
            {"j_high_arm_r", 0.7},
            {"j_low_arm_r", 1.0},
            {"j_gripper_r", -0.99},
        };
        
        // Same with isaaclab environment
        action_joint_ids_ = {1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 18, 19, 20, 21};
        

        RCLCPP_INFO(this->get_logger(), "Initializing tensors");
        init_obs();

        action_ = torch::zeros(
            {18},
            torch::TensorOptions().dtype(torch::kFloat32).device(device_)
        );
        
        gravity_vec_ = torch::tensor(
            {0, 0, -1},
            torch::TensorOptions().dtype(torch::kFloat32).device(device_)
        );

        RCLCPP_INFO(this->get_logger(), "Starting control loop thread");
        std::thread([this]() { this->step();}).detach();
    }   


private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_target_joint_state_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_trajectory_;

    void init_obs();
    torch::Tensor collect_obs();
    void set_default_joint_pos();
    torch::Tensor quat_rotate_inverse(torch::Tensor& q, torch::Tensor& v);
    void cb_cmd_vel(const geometry_msgs::msg::Twist& twist);
    void cb_imu(const sensor_msgs::msg::Imu& imu);
    void cb_odom(const nav_msgs::msg::Odometry& odom);
    void cb_joint_states(const sensor_msgs::msg::JointState& joint_state);

    void get_clock();
    void get_base_vel();
    void get_projected_gravity();
    void get_joint_state();
    void get_velocity_command();
    void set_joint_position();
    
    void step();

    // Policy parameters
    torch::jit::script::Module policy_;
    float phase_;
    float left_offset_, right_offset_;
    float cycle_length_s_; int cycle_length_;
    float dt_;
    int phi_;
    
    // Default joint position
    std::map<std::string, double> m_default_joint_pos_;
    std::vector<double> default_joint_pos_;
    std::vector<std::string> joint_names_;
    std::vector<double> action_joint_ids_;

    // Observation Buffers
    std::deque<std::vector<double>> buf_base_lin_vel_;
    std::deque<std::vector<double>> buf_base_ang_vel_;
    std::deque<torch::Tensor> buf_projected_gravity_;
    std::deque<std::vector<double>> buf_joint_pos_;
    std::deque<std::vector<double>> buf_joint_vel_;
    std::deque<std::vector<double>> buf_cmd_vel_;

    // Observation Tensors
    torch::Tensor obs_base_lin_vel_;
    torch::Tensor obs_base_ang_vel_;
    torch::Tensor obs_projected_gravity_;
    torch::Tensor obs_velocity_command_;
    torch::Tensor obs_joint_pos_;
    torch::Tensor obs_joint_vel_;
    torch::Tensor obs_action_;
    torch::Tensor obs_clock_;
    torch::Tensor obs_phase_;

    std::mutex obs_mutex_;

    at::Tensor action_;
    torch::Tensor gravity_vec_;

    torch::Tensor h, c;

    bool debug_;
    
    urdf::Model robot_description_;

    std::map<std::string, std::pair<double, double> > joint_limits_;
    std::vector<double> lower_limits_, upper_limits_;

    torch::Device device_ = torch::Device(torch::kCPU);

    double trj_time_;

};