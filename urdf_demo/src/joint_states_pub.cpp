#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

class JointStatesPublisher : public rclcpp::Node
{
public:
    JointStatesPublisher() : Node("joint_states_pub")
    {
        RCLCPP_INFO(this->get_logger(), "joint_states_pub node is Ready!");
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        joint_state_.name = {"arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "gripper_1_joint"};
        joint_state_.position = {0, 0, 0, 0, 0, 0, 0};
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&JointStatesPublisher::timer_callback, this));
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_;
    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback()
    {
        for (int i = 0; i < 100; ++i)
        {
            joint_state_.header.stamp = this->now();
            joint_state_.position[0] += 0.02;
            joint_state_.position[2] -= 0.015;
            joint_state_.position[4] += 0.0065;
            // joint_state_.position[7] += 0.0065;
            joint_state_pub_->publish(joint_state_);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        for (int i = 0; i < 100; ++i)
        {
            joint_state_.header.stamp = this->now();
            joint_state_.position[0] -= 0.02;
            joint_state_.position[2] += 0.015;
            joint_state_.position[4] -= 0.0065;
            // joint_state_.position[7] -= 0.0065;
            joint_state_pub_->publish(joint_state_);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStatesPublisher>());
    rclcpp::shutdown();
    return 0;
}
