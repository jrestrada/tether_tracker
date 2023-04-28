#include "Ros.h"
#include <thread>
#include <signal.h>
#include <cv_bridge/cv_bridge.h>

void kill(int /*sig*/) {
}

Ros *Ros::s_self = nullptr;

Ros::Ros(int argc, char *argv[], const std::string &node_name) {
    // Initilize ROS
    rclcpp::init(argc, argv);
    // Create ROS executer and node
    m_executor = rclcpp::executors::StaticSingleThreadedExecutor::make_shared();
    m_node = rclcpp::Node::make_shared(node_name);
    m_executor->add_node(m_node);
    // Add ROS publisher and subscribers
    m_image = m_node->create_subscription<sensor_msgs::msg::Image>("reel_image",
                                                        rclcpp::SystemDefaultsQoS(),
                                                        std::bind(&Ros::imageCallback, this, std::placeholders::_1));
    m_publisher = m_node->create_publisher<std_msgs::msg::Int16>("tether_ft_deployed",
                                                        rclcpp::SystemDefaultsQoS());
    if (s_self) {
        LOG("Ops, only one instance of 'Ros' can be created!");
    }
    else {
        s_self = this; 
        LOG("Ros created...");
    }
    signal(SIGINT, kill);
}

Ros::~Ros() {
    rclcpp::shutdown(); 
    LOG("ROS shutdown!");
}

void Ros::setTetherTracker(TetherTracker *TetherTracker){
    m_TetherTracker = TetherTracker;
}

void Ros::spin(void) {
    m_executor->spin();
}

void Ros::spinOnBackground(void) {
    std::thread thread(std::bind(&rclcpp::executors::StaticSingleThreadedExecutor::spin, m_executor));
    thread.detach();
}

void Ros::shutdown(void) {
    m_executor->cancel();
}

void Ros::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){ 
    if (cv::waitKey(30) == 27){
        rclcpp::shutdown(); 
    }
    m_TetherTracker->processImage(cv_bridge::toCvShare(msg, "bgr8")->image);
    if (m_feet_deployed.data != m_TetherTracker->getCount()){
        m_feet_deployed.data = m_TetherTracker->getCount();
        m_publisher->publish(m_feet_deployed);
    }
}
