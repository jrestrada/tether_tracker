#include "TetherTracker.h"

TetherTracker::TetherTracker(std::string window) : m_window(window){
    cv::namedWindow(m_window, cv::WINDOW_NORMAL);
    cv::namedWindow("test", cv::WINDOW_NORMAL);
}

void TetherTracker::setThresholds(int *g_lh, int *g_ls, int *g_lv, int *g_uh, int *g_us, int *g_uv){
    m_lh = g_lh; 
    m_ls = g_ls;
    m_lv = g_lv; // Lower color values
    m_uh = g_uh; 
    m_us = g_us; 
    m_uv = g_uv; // Upper color values
}

cv::Mat TetherTracker::createMask(const cv::Mat &image, 
                    const cv::Scalar &lower,
                    const cv::Scalar &upper,
                    bool clean = true ) {
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    // Create a mask using calibrated color
    cv::Mat mask;
    cv::inRange(image, lower, upper, mask);
    if (clean) {
        // Clean isolated pixels
        cv::erode(mask, mask, kernel);
        // Enhance surviving pixels
        cv::dilate(mask, mask, kernel);
    }
    return mask;
}

bool TetherTracker::isEqual(double a, double b, double epsilon = 1.0e-8 ) {
    return std::abs(a - b) <= epsilon;
}

cv::Point TetherTracker::computeCentroid(const cv::Mat &mask) {
    // find moments of the image
    cv::Moments m = cv::moments(mask, true);
        if ( isEqual(m.m00, 0.0) )
            return cv::Point(-100, -100);
    return cv::Point(m.m10/m.m00, m.m01/m.m00);
}

void TetherTracker::processImage(const cv::Mat& image_in){
    cv::Mat hsv;
    cv::cvtColor(image_in, hsv, cv::COLOR_BGR2HSV);
    // Define region of interest
    // cv::Mat roi_finder_mask = createMask(hsv,{5,63,97},{18,106,121}, false);
    // cv::Mat roi_finder_result;
    // cv::bitwise_and(image_in, roi_finder_mask, roi_finder_result);
    // cv::Point roi_mark = computeCentroid(roi_finder_result);
    // cv::Rect roi_rect(roi_mark.x - 30, roi_mark.y, 50, 60); // x, y, width, height
    cv::Rect roi_rect(100, 100, 120, 80); // x, y, width, height
    cv::Rect rect_out(roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height/2); // x, y, width, height
    cv::Rect rect_in(roi_rect.x, roi_rect.y + roi_rect.height/2, roi_rect.width, roi_rect.height/2); // x, y, width, height
    // Set threshold from trackbars
    cv::Scalar lower(*m_lh, *m_ls, *m_lv);
    cv::Scalar upper(*m_uh, *m_us, *m_uv);
    // Create color mask mask    
    cv::Mat mask = createMask(hsv, lower, upper, true);
    cv::Mat result;
    cv::bitwise_and(image_in, image_in, result, mask);
    // Create region of interest mask
    cv::Mat roi_mask = cv::Mat::zeros(mask.size(), mask.type());
    roi_mask(roi_rect) = 255;
    cv::Mat masked_mask;
    cv::bitwise_and(mask, roi_mask, masked_mask);
    cv::Point c = computeCentroid(masked_mask);
    if (c.inside(rect_out)){
        if (m_point_in){
            m_count ++;
            std::cout << "m_count =" << m_count << std::endl;
        }
        m_point_out = true;
    }
    if (c.inside(rect_in)){
        if (m_point_out){
            m_count --;
            std::cout << "m_count =" << m_count << std::endl;
        }
        m_point_out = false;
    }
    m_point_in = c.inside(rect_in);
    m_point_out = c.inside(rect_out);
    cv::circle(image_in, c, 10, cv::Scalar(0,0,255), 10);
    cv::rectangle(image_in, rect_out, cv::Scalar(0, 0, 255), 2); 
    cv::rectangle(image_in, rect_in, cv::Scalar(0, 255, 0), 2); 
    std::string text = std::to_string(m_count) + " ft. deployed";
    cv::putText(image_in, text, cv::Point(200, 30), 2, 0.5, CV_RGB(255,0,0));
    cv::imshow(m_window, masked_mask);
    cv::imshow("test", image_in);
}