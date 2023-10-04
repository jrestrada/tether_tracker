#ifndef TETHERTRACKER_H
#define TETHERTRACKER_H
#include <string>
#include <opencv2/opencv.hpp>

class TetherTracker{
    public: 
    TetherTracker();
    TetherTracker(std::string window);
    void setThresholds(int *g_lh, int *g_ls, int *g_lv, int *g_uh, int *g_us, int *g_uv);
    void setRoi(int *x, int *y, int *w, int *h);
    void processImage(const cv::Mat& image_in);
    int getCount(){ return m_count;};
    private:
    bool isEqual(double a, double b, double epsilon);
    cv::Mat createMask(const cv::Mat &image, const cv::Scalar &lower, const cv::Scalar &upper, bool clean);
    cv::Point computeCentroid(const cv::Mat &mask);
    std::string m_window;
    int * m_lh = 0;
    int * m_ls = 0;
    int * m_lv = 0; 
    int * m_uh = 0;
    int * m_us = 0;
    int * m_uv = 0; 
    int * m_x = 0;
    int * m_y = 0;
    int * m_w = 0;
    int * m_h = 0;
    bool m_point_out = false;
    bool m_point_in = false;
    int m_count = 0;
};

#endif //TETHERTRACKER_H