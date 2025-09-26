#include <torch/torch.h>
#include <torch/script.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <hri_humanoid_interfaces/srv/get_actions.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>
using namespace std::placeholders;

class InferenceService: public rclcpp::Node{
public:
    InferenceService(): Node("infernce_service"){
        this->declare_parameter("debug", false);
        debug_ = this->get_parameter("debug").as_bool();
        this->declare_parameter("num_history_steps", 20);
        // Load torchscript
        RCLCPP_INFO(this->get_logger(), "Loading torchscript");
        this->declare_parameter("checkpoint_dir", "");

        std::string script_path = this->get_parameter("checkpoint_dir").as_string();
        policy_ = torch::jit::load(script_path, device_);

        RCLCPP_INFO(this->get_logger(), "Policy structure:\n%s", policy_.dump_to_str(true, false, false).c_str());

        service_ = this->create_service<hri_humanoid_interfaces::srv::GetActions>(
            "get_actions",
            std::bind(&InferenceService::get_actions, this, _1, _2));
        
        sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&InferenceService::cbCmdVel, this, _1));
        command_velocity_.resize(3, 0.0);
        
        pub_raw_joint_cmd_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);

        num_history_steps_ = this->get_parameter("num_history_steps").as_int();
        obs_dims_ = {3, 3, 3, 18, 18, 18};
        base_ang_vel_history_.resize(obs_dims_[0] * num_history_steps_, 0.0);
        projected_gravity_history_.resize(obs_dims_[1] * num_history_steps_, 0.0);
        command_velocity_history_.resize(obs_dims_[2] * num_history_steps_, 0.0);
        joint_pos_history_.resize(obs_dims_[3] * num_history_steps_, 0.0);
        joint_vel_history_.resize(obs_dims_[4] * num_history_steps_, 0.0);
        action_history_.resize(obs_dims_[5] * num_history_steps_, 0.0);
                    
        RCLCPP_INFO(this->get_logger(), "Service ready");
    }
private:

    void get_actions(const std::shared_ptr<hri_humanoid_interfaces::srv::GetActions::Request> req,
        std::shared_ptr<hri_humanoid_interfaces::srv::GetActions::Response> res){
            
        std::vector<double> obs = req->observations;
        if (debug_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Observations: "<< obs);
        std::copy(command_velocity_.begin(), command_velocity_.end(), obs.begin()+obs_dims_[0]+obs_dims_[1]);
        std::copy(action_history_.end()-obs_dims_[5], action_history_.end(), obs.begin()+obs_dims_[0]+obs_dims_[1]+obs_dims_[2]+obs_dims_[3]+obs_dims_[4]);
    

        std::move(base_ang_vel_history_.begin() + obs_dims_[0], base_ang_vel_history_.end(), base_ang_vel_history_.begin());
        std::copy(obs.begin(), obs.begin()+obs_dims_[0], base_ang_vel_history_.end()-obs_dims_[0]);

        std::move(projected_gravity_history_.begin() + obs_dims_[1], projected_gravity_history_.end(), projected_gravity_history_.begin());
        std::copy(obs.begin()+obs_dims_[0], obs.begin()+obs_dims_[0]+obs_dims_[1], projected_gravity_history_.end()-obs_dims_[1]);

        std::move(command_velocity_history_.begin() + obs_dims_[2], command_velocity_history_.end(), command_velocity_history_.begin());
        std::copy(obs.begin()+obs_dims_[0]+obs_dims_[1], obs.begin()+obs_dims_[0]+obs_dims_[1]+obs_dims_[2], command_velocity_history_.end()-obs_dims_[2]);

        std::move(joint_pos_history_.begin() + obs_dims_[3], joint_pos_history_.end(), joint_pos_history_.begin());
        std::copy(obs.begin()+obs_dims_[0]+obs_dims_[1]+obs_dims_[2], obs.begin()+obs_dims_[0]+obs_dims_[1]+obs_dims_[2]+obs_dims_[3], joint_pos_history_.end()-obs_dims_[3]);

        std::move(joint_vel_history_.begin() + obs_dims_[4], joint_vel_history_.end(), joint_vel_history_.begin());
        std::copy(obs.begin()+obs_dims_[0]+obs_dims_[1]+obs_dims_[2]+obs_dims_[3], obs.begin()+obs_dims_[0]+obs_dims_[1]+obs_dims_[2]+obs_dims_[3]+obs_dims_[4], joint_vel_history_.end()-obs_dims_[4]);

        std::vector<double> obs_history;
            
        int total_history_size = 0;
        for(int i = 0; i < obs_dims_.size(); i++) {
            total_history_size += obs_dims_[i] * num_history_steps_;
        }
        obs_history.reserve(total_history_size);

        obs_history.insert(obs_history.end(), base_ang_vel_history_.begin(), base_ang_vel_history_.end());
        obs_history.insert(obs_history.end(), projected_gravity_history_.begin(), projected_gravity_history_.end());
        obs_history.insert(obs_history.end(), command_velocity_history_.begin(), command_velocity_history_.end());
        obs_history.insert(obs_history.end(), joint_pos_history_.begin(), joint_pos_history_.end());
        obs_history.insert(obs_history.end(), joint_vel_history_.begin(), joint_vel_history_.end());
        obs_history.insert(obs_history.end(), action_history_.begin(), action_history_.end());
        

        torch::NoGradGuard no_grad;
        torch::Tensor obs_tensor = torch::tensor(obs, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
        torch::Tensor obs_history_tensor = torch::tensor(obs_history, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
        std::vector<torch::jit::IValue> input;
        input.push_back(obs_tensor.unsqueeze(0)); input.push_back(obs_history_tensor.unsqueeze(0));
        at::Tensor output = policy_.forward(input).toTensor();
        torch::Tensor action = output.squeeze(0).squeeze(0);
        
        std::vector<double> raw_actions;
        for (int i=0; i<18; i++)
            raw_actions.push_back(action[i].item<double>());
        res->raw_actions = raw_actions;
        if (debug_)
            RCLCPP_INFO_STREAM(this->get_logger(), "Action: " << raw_actions);

        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.position = raw_actions;
        pub_raw_joint_cmd_->publish(joint_state);

        std::move(action_history_.begin() + obs_dims_[5], action_history_.end(), action_history_.begin());
        std::copy(raw_actions.begin(), raw_actions.end(), action_history_.end()-obs_dims_[5]);
        
    }

    void cbCmdVel(const geometry_msgs::msg::Twist& cmd_vel){
        command_velocity_ = {cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z};
    }

    rclcpp::Service<hri_humanoid_interfaces::srv::GetActions>::SharedPtr service_;
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_raw_joint_cmd_;

    torch::jit::script::Module policy_;
    int num_history_steps_;
    std::vector<double> obs_history_;
    std::vector<double> prev_action_, command_velocity_;
    torch::Device device_ = torch::Device(torch::kCUDA);
    bool is_first_call_, debug_;
    std::vector<double> obs_dims_;
    std::vector<double> base_ang_vel_history_, projected_gravity_history_, command_velocity_history_, joint_pos_history_, joint_vel_history_, action_history_;
};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<InferenceService>());

    rclcpp::shutdown();

    return 0;
}