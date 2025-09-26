#include "joint_control/torchscript.hpp"

using namespace std::chrono_literals;
using namespace torch::indexing;

void JointControl::init_obs(){
    obs_base_lin_vel_ = torch::zeros({9,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_base_ang_vel_ = torch::zeros({9,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_projected_gravity_ = torch::zeros({3,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_velocity_command_ = torch::zeros({3,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_joint_pos_ = torch::zeros({22,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_joint_vel_ = torch::zeros({22,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_action_ = torch::zeros({18,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_clock_ = torch::zeros({2,}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_phase_ = torch::full({1,}, phase_, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
}

torch::Tensor JointControl::collect_obs(){

    get_clock();
    obs_action_ = action_;

    torch::Tensor obs = torch::cat({
        obs_base_lin_vel_,
        obs_base_ang_vel_,
        obs_projected_gravity_,
        obs_velocity_command_,
        obs_joint_pos_,
        obs_joint_vel_,
        obs_action_,
        obs_clock_,
        obs_phase_});

    return obs.unsqueeze(0);
}

void JointControl::set_default_joint_pos(){
    for (auto& name: joint_names_){
        default_joint_pos_.push_back(m_default_joint_pos_[name]);
        lower_limits_.push_back(joint_limits_[name].first);
        upper_limits_.push_back(joint_limits_[name].second);
    }

    RCLCPP_INFO_STREAM(this->get_logger(), "Default poses: " << default_joint_pos_);
}

void JointControl::cb_cmd_vel(const geometry_msgs::msg::Twist& twist){
    obs_velocity_command_ = torch::tensor({twist.linear.x, twist.linear.y, twist.angular.z}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_velocity_command_ = torch::tensor({0.4, 0.0, 0.0}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
}

void JointControl::cb_imu(const sensor_msgs::msg::Imu& imu){
    torch::Tensor q = torch::tensor({static_cast<float>(imu.orientation.x), 
                                    static_cast<float>(imu.orientation.y), 
                                    static_cast<float>(imu.orientation.z), 
                                    static_cast<float>(imu.orientation.w)}, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    obs_projected_gravity_ = quat_rotate_inverse(q, gravity_vec_);
}

void JointControl::cb_odom(const nav_msgs::msg::Odometry& odom){
    
    torch::Tensor new_lin = torch::tensor({
        static_cast<float>(odom.twist.twist.linear.x),
        static_cast<float>(odom.twist.twist.linear.y),
        static_cast<float>(odom.twist.twist.linear.z)
    },
    torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    torch::Tensor new_ang = torch::tensor({
        static_cast<float>(odom.twist.twist.angular.x),
        static_cast<float>(odom.twist.twist.angular.y),
        static_cast<float>(odom.twist.twist.angular.z)
    }, torch::TensorOptions().dtype(torch::kFloat32).device(device_));

    torch::Tensor prev_obs_lin = obs_base_lin_vel_.clone();
    torch::Tensor prev_obs_ang = obs_base_ang_vel_.clone();
    obs_base_lin_vel_.slice(0, 0, 6) = prev_obs_lin.slice(0, 3, 9);
    obs_base_ang_vel_.slice(0, 0, 6) = prev_obs_ang.slice(0, 3, 9);

    obs_base_lin_vel_.slice(0, 6, 9) = new_lin;
    obs_base_ang_vel_.slice(0, 6, 9) = new_ang;
}


void JointControl::get_clock(){
    float phi = static_cast<float>(phi_ % cycle_length_) / static_cast<float>(cycle_length_);
    float phase_l = 2 * M_PI * (phi + left_offset_) / cycle_length_;
    float phase_r = 2 * M_PI * (phi + right_offset_) / cycle_length_;

    torch::Tensor sin_l = torch::sin(torch::tensor({phase_l}, torch::TensorOptions().dtype(torch::kFloat32).device(device_)));
    torch::Tensor sin_r = torch::sin(torch::tensor({phase_r}, torch::TensorOptions().dtype(torch::kFloat32).device(device_)));
    obs_clock_ = torch::cat({sin_l, sin_r}, -1);
}

void JointControl::cb_joint_states(const sensor_msgs::msg::JointState& joint_state){
    static bool is_first = true;
    if (is_first){
        set_default_joint_pos();
        is_first = false;
    }

    obs_joint_pos_ = torch::tensor(joint_state.position, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
    obs_joint_vel_ = torch::tensor(joint_state.velocity, torch::TensorOptions().dtype(torch::kFloat32).device(device_));
}

void JointControl::set_joint_position(){
    if (default_joint_pos_.size() != 22){
        return;
    }
    sensor_msgs::msg::JointState joint_command;
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_command.header.stamp = this->now();
    joint_command.name = joint_names_;
    std::vector<double> target_joint_pos = default_joint_pos_;
    for (int i=0; i<18; i++){
        int idx = action_joint_ids_[i];
        target_joint_pos[idx] = default_joint_pos_[idx] + 0.5 * action_[i].item<double>(); 
    }

    for (int i=0; i<22; i++){
        if (target_joint_pos[i] < lower_limits_[i])
            target_joint_pos[i] = lower_limits_[i];
        else if (target_joint_pos[i] > upper_limits_[i])
            target_joint_pos[i] = upper_limits_[i];
    }
    joint_command.position = target_joint_pos;

    joint_trajectory.joint_names = joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_joint_pos;
    point.time_from_start = rclcpp::Duration::from_seconds(trj_time_);
    joint_trajectory.points.push_back(point);

    pub_target_joint_state_->publish(joint_command);
    pub_joint_trajectory_->publish(joint_trajectory);

}

void JointControl::step(){
    while (default_joint_pos_.size() != 22){}
    
    while (rclcpp::ok()){
        auto start = std::chrono::steady_clock::now();
        
        if (default_joint_pos_.size() != 22){
            auto elapsed = std::chrono::steady_clock::now() - start;
            std::this_thread::sleep_for(std::chrono::duration<double>(dt_) - elapsed);
            continue;
        }
        torch::NoGradGuard no_grad;
        std::vector<torch::jit::IValue> input;
        torch::Tensor obs = collect_obs();
        input.push_back(obs);
    
        at::Tensor output = policy_.forward(input).toTensor();
        action_ = output.squeeze(0).squeeze(0);
            
        if (debug_){
            RCLCPP_INFO_STREAM(this->get_logger(), "Observation: \n" << obs.flatten());
            RCLCPP_INFO_STREAM(this->get_logger(), "Action: \n" << action_);
        }
        set_joint_position();

        phi_ = (phi_ + 1) % cycle_length_;    
        auto elapsed = std::chrono::steady_clock::now() - start;
        std::this_thread::sleep_for(std::chrono::duration<double>(dt_) - elapsed);

    }   
}

torch::Tensor JointControl::quat_rotate_inverse(torch::Tensor& q, torch::Tensor& v){

    torch::Tensor q_w = q.index({3});
    torch::Tensor q_vec = q.slice(0, 0, 3);

    torch::Tensor a = v * (2.0 * q_w * q_w - 1.0).unsqueeze(-1);
    torch::Tensor b = torch::cross(q_vec, v, -1) * q_w.unsqueeze(-1) * 2.0;
    torch::Tensor dot = (q_vec * v).sum(-1, true);
    torch::Tensor c = q_vec * dot * 2.0;
    
    return a - b + c;
}


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<JointControl>());

    rclcpp::shutdown();
}
