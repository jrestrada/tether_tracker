#ifndef NODE_H
#define NODE_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/int16.hpp>
#include "TetherTracker.h"

#define LOG(...) RCLCPP_INFO(rclcpp::get_logger("image_subscriber"), __VA_ARGS__)

class Ros {
public:
    static Ros *instance(void) { return s_self; }
    static void quit(void) { s_self->shutdown(); }
    Ros(int argc, char *argv[], const std::string &node_name);
    ~Ros();
    // Execute ROS
    void spin(void);
    void spinOnBackground(void);
    void shutdown(void);
    void setTetherTracker(TetherTracker * m_TetherTracker);
    void setWindow(const std::string &window);
    void setThresholds(int *g_lh, int *g_ls, int *g_lv, int *g_uh, int *g_us, int *g_uv);
    rclcpp::Node::SharedPtr node(void) { return m_node; }
protected:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
private:
    rclcpp::Node::SharedPtr m_node;
    rclcpp::executors::StaticSingleThreadedExecutor::SharedPtr m_executor;
    // Publishers, subscribers and servers
    std::string m_window;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr m_publisher;
    std_msgs::msg::Int16 m_feet_deployed;
    // Composed objects
    TetherTracker *m_TetherTracker;
    int * m_lh = 0; int * m_ls = 0; int * m_lv = 0; // Lower color values
    int * m_uh = 0; int * m_us = 0; int * m_uv = 0; // Upper color values
    // Instance
    static Ros *s_self;
};

#endif // NODE_H
