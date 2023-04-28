#include <cstdio>
#include "Ros.h"
#include <opencv2/opencv.hpp>
#include "TetherTracker.h"

// Window name
const std::string window = "tracker_test";
#define KEY_ESC 27

// int g_lh = 88, g_ls = 82, g_lv = 100; // Lower color values
// int g_uh = 102, g_us = 119, g_uv = 200; // Upper color values
int g_lh = 0, g_ls = 0, g_lv = 105; // Lower color values
int g_uh = 180, g_us = 96, g_uv = 280; // Upper color values

void addTrackbars(std::string) {
    // Creating trackbars
    cv::createTrackbar("Lower H", window, &g_lh, 180);
    cv::createTrackbar("Upper H", window, &g_uh, 180);
    cv::createTrackbar("Lower S", window, &g_ls, 255);
    cv::createTrackbar("Upper S", window, &g_us, 255);
    cv::createTrackbar("Lower V", window, &g_lv, 255);
    cv::createTrackbar("Upper V", window, &g_uv, 255);
}

int main(int argc, char ** argv)
{
    Ros ros(argc, argv, "tether_tracker_node");
    TetherTracker * tracker = new TetherTracker(window);
    addTrackbars(window);
    tracker->setThresholds(&g_lh,&g_ls,&g_lv,&g_uh,&g_us,&g_uv);
    ros.setTetherTracker(tracker);
    ros.spin();
}
