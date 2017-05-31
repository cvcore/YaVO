#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <string>
#include <utility>
#include <vector>
#include <spdlog/spdlog.h>

#include "rgbd_frame.h"

class RGBDFrame;

class RGBDCamera {
public:
    double ax, ay, x0, y0, s; // camera calibration matrix
    double scale;             // scale for z-axis

    /* default parameter taken from https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats */
    RGBDCamera(double ax = 525.0, double ay = 525.0, double x0 = 319.5, double y0 = 239.5, double s = 0.0, double scale = 5000);
    int ReadAssociatedSequence(std::string path);
    RGBDFrame RequestFrame(int frame_id);
    cv::Mat IntrinsicsMatrix() const;
    cv::Mat DistortionCoefficients() const;

private:
    std::vector<std::string> rgb_paths_;
    std::vector<std::string> depth_paths_;
    std::vector<double> rgb_timestamps_;
    std::vector<double> depth_timestamps_;
    int frame_count_;
    std::vector<RGBDFrame> frames_;
    static std::shared_ptr<spdlog::logger> logger_;

};
