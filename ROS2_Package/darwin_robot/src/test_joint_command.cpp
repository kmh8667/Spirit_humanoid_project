#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

using namespace std::chrono_literals;

class TestJointCommand: public rclcpp::Node{
public:
    TestJointCommand(): Node("test_joint_command"){
        pub_joint_command_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);


        timer_ = this->create_wall_timer(500ms, std::bind(&TestJointCommand::pub_command, this));
    }

    void pub_command(){        
        auto command = trajectory_msgs::msg::JointTrajectory();

        std::vector<double> data;
        std::vector<std::string> joint_names;        
        joint_names = {
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

        for (int i=0; i<22; i++)
            data.push_back(0.0);

        command.joint_names = joint_names;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = data;
        command.points.push_back(point);
        pub_joint_command_->publish(command);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr pub_joint_command_;
    
};



int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<TestJointCommand>());

    rclcpp::shutdown();

    return 0;
}