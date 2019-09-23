#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <actionlib/client/simple_action_client.h>
#include <kinova_msgs/ArmJointAnglesAction.h>
#include <kinova_msgs/JointTorque.h>
#include <kinova_msgs/JointVelocity.h>
#include <kinova_msgs/SetFingersPositionAction.h>

class KinovaBridge : public rclcpp::Node {
 public:
  KinovaBridge(ros::NodeHandle &nh)
      : rclcpp::Node("kinova_bridge", nh.getNamespace()),
        arm_action_client_(
            nh.getNamespace() + "/joints_action/joint_angles", 
            true),
        finger_action_client_(nh, "finger_action_client", true) {
    arm_action_client_.waitForServer();
    // finger_action_client_.waitForServer();

    arm_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "kinova_arm_joint", 10,
        std::bind(&KinovaBridge::arm_joint_callback, this,
                  std::placeholders::_1));
    finger_joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "kinova_finger_joint", 10,
        std::bind(&KinovaBridge::finger_joint_callback, this,
                  std::placeholders::_1));
    arm_velocity_pub_ =
        nh.advertise<kinova_msgs::JointVelocity>("in/joint_velocity", 10);
    arm_torque_pub_ =
        nh.advertise<kinova_msgs::JointTorque>("in/joint_torque", 10);

    std::cout << "Initialized : " << nh.getNamespace() << std::endl;
  }

 protected:
  void arm_joint_callback(sensor_msgs::msg::JointState::SharedPtr msg) {
    // Position command
    if (msg->position.size() > 0) {
      kinova_msgs::ArmJointAnglesGoal goal;

      float *joint_ptr = &goal.angles.joint1;
      for (size_t i = 0; i < msg->position.size() && i < 7; i++) {
        *joint_ptr = (float)msg->position[i];
        joint_ptr++;
      }

      arm_action_client_.sendGoal(goal);
    }  // Velocity command
    else if (msg->velocity.size() > 0) {
      kinova_msgs::JointVelocity vel;
      float *joint_ptr = &vel.joint1;

      for (size_t i = 0; i < msg->velocity.size() && i < 7; i++) {
        *joint_ptr = (float)msg->velocity[i];
        joint_ptr++;
      }

      arm_velocity_pub_.publish(vel);
    }
    // Force command
    else if (msg->effort.size() > 0) {
      kinova_msgs::JointTorque torque;
      float *joint_ptr = &torque.joint1;

      for (size_t i = 0; i < msg->effort.size() && i < 7; i++) {
        *joint_ptr = (float)msg->effort[i];
        joint_ptr++;
      }

      arm_torque_pub_.publish(torque);
    }
  }

  void finger_joint_callback(sensor_msgs::msg::JointState::SharedPtr msg) {
    kinova_msgs::SetFingersPositionGoal goal;
    float *joint_ptr = &goal.fingers.finger1;
    for (size_t i = 0; i < msg->position.size() && i < 3; i++) {
      *joint_ptr = (float)msg->position[i];
      joint_ptr++;
    }
    finger_action_client_.sendGoal(goal);
  }

 private:
  rclcpp::SubscriptionBase::SharedPtr arm_joint_sub_;
  rclcpp::SubscriptionBase::SharedPtr finger_joint_sub_;
  actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction>
      arm_action_client_;
  actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>
      finger_action_client_;
  ros::Publisher arm_velocity_pub_;
  ros::Publisher arm_torque_pub_;
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "kinova_bridge", ros::init_options::NoSigintHandler);
  rclcpp::init(argc, argv);
  if (argc != 2) {
    std::cout << "Run by kinova_bridge ${robot_name}" << std::endl;
    return 0;
  }

  std::string ns(argv[1]);
  ns = ns + "_driver";
  ros::NodeHandle nh(ns);

  KinovaBridge::SharedPtr kinova_bridge = std::make_shared<KinovaBridge>(nh);

  ros::AsyncSpinner ros1_spinner(1);
  ros1_spinner.start();
  try {
    rclcpp::spin(kinova_bridge);
  } catch (...) {
    std::cout << "Terminate" << std::endl;
  }
  ros1_spinner.stop();
  ros::shutdown();
  rclcpp::shutdown();
  return 1;
}
