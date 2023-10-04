#include <cstdio>
#include "Ros.h"
#include <opencv2/opencv.hpp>
#include "TetherTracker.h"

// Window name
const std::string window = "tracker_test";
#define KEY_ESC 27

int g_lh = 68, g_ls = 54, g_lv = 140; // Lower color values
int g_uh = 180, g_us = 255, g_uv = 228; // Upper color values
int x = 69, y = 124, w = 178, h = 27;

void addTrackbars(std::string) {
    // Creating trackbars
    cv::createTrackbar("Lower H", window, &g_lh, 180);
    cv::createTrackbar("Upper H", window, &g_uh, 180);
    cv::createTrackbar("Lower S", window, &g_ls, 255);
    cv::createTrackbar("Upper S", window, &g_us, 255);
    cv::createTrackbar("Lower V", window, &g_lv, 255);
    cv::createTrackbar("Upper V", window, &g_uv, 255);
    cv::createTrackbar("X", window, &x, 200);
    cv::createTrackbar("Y", window, &y, 200);
    cv::createTrackbar("Width", window, &w, 400);
    cv::createTrackbar("Height", window, &h, 400);

}

int main(int argc, char ** argv)
{
    Ros ros(argc, argv, "tether_tracker_node");
    TetherTracker * tracker = new TetherTracker(window);
    tracker->setThresholds(&g_lh,&g_ls,&g_lv,&g_uh,&g_us,&g_uv);
    addTrackbars(window);
    tracker->setRoi(&x, &y, &w, &h);
    ros.setTetherTracker(tracker);
    ros.spin();
}
