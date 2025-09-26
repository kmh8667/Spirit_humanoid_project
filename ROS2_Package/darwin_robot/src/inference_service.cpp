#include <torch/torch.h>
#include <torch/script.h>
#include <rclcpp/rclcpp.hpp>
#include <hri_humanoid_interfaces/srv/get_actions.hpp>

#include <vector>
#include <memory>
#include <algorithm>
using namespace std::placeholders;

class InferenceService: public rclcpp::Node{
public:
    InferenceService(): Node("infernce_service"){
        // Load torchscript
        RCLCPP_INFO(this->get_logger(), "Loading torchscript");
        this->declare_parameter("checkpoint_dir", "");

        std::string script_path = this->get_parameter("checkpoint_dir").as_string();
        policy_ = torch::jit::load(script_path, device_);
        service_ = this->create_service<hri_humanoid_interfaces::srv::GetActions>(
            "get_actions",
            std::bind(&InferenceService::get_actions, this, _1, _2));
        for (int i=0; i<18; i++)
            prev_action_.push_back(0.0);
        RCLCPP_INFO(this->get_logger(), "Service ready");
    }
private:
    void get_actions(const std::shared_ptr<hri_humanoid_interfaces::srv::GetActions::Request> req,
        std::shared_ptr<hri_humanoid_interfaces::srv::GetActions::Response> res){
            
        std::vector<double> obs = req->observations;
        std::copy(prev_action_.begin(), prev_action_.end(), obs.begin()+68);

        torch::NoGradGuard no_grad;
        torch::Tensor obs_tensor = torch::tensor(obs, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

        std::vector<torch::jit::IValue> input;
        input.push_back(obs_tensor.unsqueeze(0));
        at::Tensor output = policy_.forward(input).toTensor();
        torch::Tensor action = output.squeeze(0).squeeze(0);
        
        for (int i=0; i<18; i++)
            prev_action_[i] = action[i].item<double>();
        res->raw_actions = prev_action_;

        
    }
    rclcpp::Service<hri_humanoid_interfaces::srv::GetActions>::SharedPtr service_;

    torch::jit::script::Module policy_;
    std::vector<double> prev_action_;
    torch::Device device_ = torch::Device(torch::kCPU);

};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<InferenceService>());

    rclcpp::shutdown();

    return 0;
}