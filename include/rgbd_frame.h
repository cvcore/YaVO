/* rgbd_frame.h
   Image frame and related operations
   Author: Charles Wang
   Date: 28.03.2017
*/
#pragma once

#include <vector>
#include <memory>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <Eigen/Geometry>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <spdlog/spdlog.h>

#include "share.h"
#include "rgbd_camera.h"

/* Forward declarations */
class RGBDCamera;

class RGBDFrame {
public:

    struct Transformation {
        cv::Mat rotation_vector;
        cv::Mat translation_vector;
        g2o::VertexSE3* from_vertex;
        g2o::VertexSE3* to_vertex;
        int num_matches;
        static std::shared_ptr<spdlog::logger> logger_;

        // implicit cast: Isometry transform (rotation & translation, aka Eigen::Transform<double, 3, 1, 0>) is a subset of Affine
        // required by g2o vertex
        operator Eigen::Affine3f() const;
        operator Eigen::Isometry3f() const;
        g2o::EdgeSE3* AsEdge(); // easy conversion for g2o::SparseOptimizer where this pointer will be freed by g2o lib.
        bool IsRobustTransform();
    };

    bool is_valid;
    bool is_keyframe;

    int id;

    cv::Mat rgb;
    cv::Mat depth;

    double rgb_timestamp;
    double depth_timestamp;
    RGBDFrame();

    RGBDFrame(cv::Mat& rgb, cv::Mat& depth, double rgb_timestamp, double depth_timestamp,
              const RGBDCamera* source_camera, int id);
    DMatchesPtr Match(const RGBDFrame &train_frame) const;
    Transformation TransformTo(RGBDFrame &frame);
    RGBDCloudPtr ComputeRGBDCloud(bool force_regenerate = false);
    cv::Point3f Request3DPoint(int u, int v) const;
    cv::Point3f Request3DPoint(cv::Point2f index) const;
    g2o::VertexSE3* AsVertex(); // easy conversion for g2o::SparseOptimizer where this pointer will be managed by RGBDOdometer
    Eigen::Isometry3f Pose(); // pose in world coordinate system (assume initial frame is in origin)
    void SetPoseEstimation(Eigen::Isometry3f pose); // set pose according to world coordinate system
    friend std::ostream& operator<<(std::ostream& os, RGBDFrame& frame); // easy printing
    static void SetAlgorithms(std::string detector, std::string descriptor, std::string matcher);
    cv::Mat RGBImageWithFeature();


private:
    std::vector< cv::KeyPoint > rgb_keypoints_;
    cv::Mat rgb_descriptors_;
    g2o::VertexSE3* vertex_;
    Eigen::Isometry3f pose_; // use Isometry to represent SE(3) pose
    const RGBDCamera* source_camera_;
    RGBDCloudPtr pointcloud_buffer_;
    double feature_extraction_time_;
    static std::shared_ptr<spdlog::logger> logger_;
    static cv::Ptr<cv::Feature2D> feature_detector_;
    static cv::Ptr<cv::Feature2D> feature_descriptor_;

    static cv::Ptr<cv::DescriptorMatcher> descriptor_matcher_;
    void ComputeKeypoints();
};


