/* rgbd_frame.cc
   Image frame and related operations
   Author: Charles Wang
   Date: 28.03.2017
*/
#include "rgbd_frame.h"

#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <cmath>
#include <limits>
#include <cxeigen.hpp>
#include <spdlog/spdlog.h>
#include <pcl/filters/voxel_grid.h>
#include <ctime>

#include "share.h"
#include "rgbd_parameter_reader.h"

std::shared_ptr<spdlog::logger> RGBDFrame::logger_ = spdlog::stdout_color_mt("frame");
cv::Ptr<cv::Feature2D> RGBDFrame::feature_detector_;
cv::Ptr<cv::Feature2D> RGBDFrame::feature_descriptor_;
cv::Ptr<cv::DescriptorMatcher> RGBDFrame::descriptor_matcher_;

RGBDFrame::RGBDFrame() :
    is_valid(false),
    source_camera_(NULL)
{}

RGBDFrame::RGBDFrame(cv::Mat& rgb, cv::Mat& depth, double rgb_timestamp, double depth_timestamp,
                     const RGBDCamera* source_camera, int id):
    rgb(rgb),
    depth(depth),
    rgb_timestamp(rgb_timestamp),
    depth_timestamp(depth_timestamp),
    source_camera_(source_camera),
    id(id),
    vertex_(NULL),
    pointcloud_buffer_(NULL),
    is_keyframe(false),
    pose_(Eigen::Isometry3f::Identity())
{
    std::clock_t start = std::clock();
    this->ComputeKeypoints();
    feature_extraction_time_ = (std::clock() - start) / (double) CLOCKS_PER_SEC;

    this->is_valid = true;
}

RGBDCloudPtr RGBDFrame::ComputeRGBDCloud(bool force_regenerate)
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    assert(this->is_valid == true);

    if(!force_regenerate && pointcloud_buffer_)
        return pointcloud_buffer_;

    RGBDCloudPtr new_cloud;

    if((rgb.rows != depth.rows) || (rgb.cols != depth.cols)) {
        logger_->error("rgb and depth dimension not equal!");
        throw std::runtime_error(0);
    }

    new_cloud = RGBDCloudPtr(new RGBDCloud);

    for(int i = 0; i < rgb.rows; i++) {
        for(int j = 0; j < rgb.cols; j++) {
            if(depth.at<uint16_t>(i, j) == 0)
                continue;

            RGBDPoint new_point;
            new_point.z = (float) (depth.at<uint16_t>(i, j) / source_camera_->scale);

            if(new_point.z < params->Value<double>("min_depth") || new_point.z > params->Value<double>("max_depth"))
                continue;

            new_point.x = (float) ((j - source_camera_->x0) * new_point.z / source_camera_->ax);
            new_point.y = (float) ((i - source_camera_->y0) * new_point.z / source_camera_->ay);

            new_point.b = rgb.at<uint8_t>(i, j*3);
            new_point.g = rgb.at<uint8_t>(i, j*3 + 1);
            new_point.r = rgb.at<uint8_t>(i, j*3 + 2);
            new_cloud->push_back(new_point);
        }
    }

    if(params->Value<int>("debug_info_frame"))
        logger_->info("new RGBD cloud with {0:d} points generated.", new_cloud->points.size());

    new_cloud->width = (uint32_t) new_cloud->points.size();
    new_cloud->height = 1;
    new_cloud->is_dense = false;

    pointcloud_buffer_ = new_cloud;

    return new_cloud;
}

void RGBDFrame::ComputeKeypoints()
{
    feature_detector_->detect(rgb, rgb_keypoints_);
    feature_descriptor_->compute(rgb, rgb_keypoints_, rgb_descriptors_);
}

DMatchesPtr RGBDFrame::Match(const RGBDFrame &train_frame) const
{
    DMatchesPtr matches_ptr = DMatchesPtr(new std::vector<cv::DMatch>);

    if(rgb_descriptors_.type() != CV_32F || train_frame.rgb_descriptors_.type() != CV_32F) {
        cv::Mat query_descriptors_converted, train_descriptors_converted;
        rgb_descriptors_.convertTo(query_descriptors_converted, CV_32F);
        train_frame.rgb_descriptors_.convertTo(train_descriptors_converted, CV_32F);

        descriptor_matcher_->match(query_descriptors_converted, train_descriptors_converted, *matches_ptr);
    } else {
        descriptor_matcher_->match(rgb_descriptors_, train_frame.rgb_descriptors_, *matches_ptr);
    }

    DMatchesPtr good_matches_ptr = DMatchesPtr(new std::vector<cv::DMatch>);
    std::vector<cv::DMatch>::iterator min_distance_match_it = std::min_element(matches_ptr->begin(), matches_ptr->end(), dmatches_min_operator);
    for(cv::DMatch match : *matches_ptr) {
        if(match.distance < 4 * min_distance_match_it->distance) {
            good_matches_ptr->push_back(match);
        }
    }

    return good_matches_ptr;
}

std::shared_ptr<spdlog::logger> RGBDFrame::Transformation::logger_ = spdlog::stdout_color_mt("transformation");

RGBDFrame::Transformation RGBDFrame::TransformTo(RGBDFrame &frame)
{
    RGBDFrame::Transformation tf;
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    tf.from_vertex = this->AsVertex();
    tf.to_vertex = frame.AsVertex();
    tf.num_matches = 0;

    DMatchesPtr matches_ptr = this->Match(frame);

    float min_match_dist = std::numeric_limits<float>::max();
    for(cv::DMatch match: *matches_ptr) {
        min_match_dist = std::min(min_match_dist, match.distance);
    }
    min_match_dist = std::max(10.f, min_match_dist);

    /* transform from 3d object frame(query) to 2d image frame(train) */
    std::vector< cv::Point3f > query_points_3d;
    std::vector< cv::Point2f > train_points_2d;

    double max_dist_ratio = params->Value<double>("max_dist_ratio");
    for(cv::DMatch match: *matches_ptr) {
        if(match.distance > max_dist_ratio * min_match_dist)
            continue;

        cv::Point3f query_point_3d = this->Request3DPoint(this->rgb_keypoints_[match.queryIdx].pt);
        if(std::isnan(query_point_3d.z))
            continue;

        query_points_3d.push_back(query_point_3d);
        train_points_2d.push_back(frame.rgb_keypoints_[match.trainIdx].pt);
        tf.num_matches++;
    }

    if(tf.num_matches == 0)
        return tf;

    cv::Mat inliners;

    if(params->Value<std::string>("pnp_method") == "EPNP") {
        cv::solvePnPRansac(query_points_3d,
                           train_points_2d,
                           this->source_camera_->IntrinsicsMatrix(),
                           this->source_camera_->DistortionCoefficients(),
                           tf.rotation_vector,
                           tf.translation_vector,
                           false,
                           100,
                           8.0,
                           0.99,
                           inliners,
                           CV_EPNP);

    } else {
        cv::solvePnPRansac(query_points_3d,
                           train_points_2d,
                           this->source_camera_->IntrinsicsMatrix(),
                           this->source_camera_->DistortionCoefficients(),
                           tf.rotation_vector,
                           tf.translation_vector,
                           false,
                           100,
                           1.0,
                           0.99,
                           inliners);
    }


    tf.num_matches = inliners.rows;

    return tf;
}

cv::Point3f RGBDFrame::Request3DPoint(int u, int v) const
{
    cv::Point3f requested_point;

    if(depth.at<uint16_t>(u, v) == 0) {
        requested_point.x = requested_point.y = requested_point.z = std::nanf("");
        return requested_point;
    }

    requested_point.z = (float) (depth.at<uint16_t>(u, v) / source_camera_->scale);
    requested_point.y = (float) ((u - source_camera_->y0) * requested_point.z / source_camera_->ay);
    requested_point.x = (float) ((v - source_camera_->x0) * requested_point.z / source_camera_->ax);

    return requested_point;
}

cv::Point3f RGBDFrame::Request3DPoint(cv::Point2f index) const
{
    return Request3DPoint(index.y, index.x);
}

g2o::VertexSE3 *RGBDFrame::AsVertex()
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    if(vertex_)
        return vertex_;

    vertex_ = new g2o::VertexSE3();
    vertex_->setId(this->id);
    vertex_->setEstimate(Eigen::Isometry3d(pose_));

    if(params->Value<int>("debug_info_frame"))
        logger_->info("verted id {0:d} created", this->id);

    return vertex_;
}

Eigen::Isometry3f RGBDFrame::Pose()
{
    if(vertex_) // frame has been added to g2o optimizer; use the value from g2o
        pose_ = Eigen::Isometry3f(vertex_->estimate());

    return pose_;
}

void RGBDFrame::SetPoseEstimation(Eigen::Isometry3f pose)
{
    pose_ = pose;
    if(vertex_) // frame has been added to g2o optimizer; update the estimation in g2o
        vertex_->setEstimate(Eigen::Isometry3d(pose));
}

std::ostream& operator<<(std::ostream &os, RGBDFrame& frame)
{
    Eigen::Quaternionf rotation(frame.Pose().rotation());
    auto translation = frame.Pose().translation();

    os << std::fixed << std::setw(16) << std::setprecision(6)
       << frame.depth_timestamp
       << " "
       //<< frame.rgb_timestamp
       //<< " "
       //<< frame.feature_extraction_time_
       //<< " "
       //<< frame.rgb_keypoints_.size();
       //<< ", "
       //<< vertex_->estimate()
       << translation[0] << " " << translation[1]<< " " << translation[2] << " "
       << " " << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w();

    return os;
}

cv::Mat RGBDFrame::RGBImageWithFeature()
{
    cv::Mat rgb_image_with_feature;

    cv::drawKeypoints(rgb, rgb_keypoints_, rgb_image_with_feature);

    return rgb_image_with_feature;
}

/* Helper Transformation */
RGBDFrame::Transformation::operator Eigen::Affine3f() const
{
    cv::Mat rotation_matrix;
    cv::Rodrigues(rotation_vector, rotation_matrix);

    Eigen::Matrix3f rotation_matrix_eigen;
    cv::cv2eigen(rotation_matrix, rotation_matrix_eigen);

    Eigen::Matrix4f transformation_matrix_eigen;
    transformation_matrix_eigen.block<3, 3>(0, 0) = rotation_matrix_eigen;
    transformation_matrix_eigen(0, 3) = translation_vector.at<double>(0);
    transformation_matrix_eigen(1, 3) = translation_vector.at<double>(1);
    transformation_matrix_eigen(2, 3) = translation_vector.at<double>(2);
    transformation_matrix_eigen(3, 3) = 1.f;

    return Eigen::Affine3f(transformation_matrix_eigen);
}


RGBDFrame::Transformation::operator Eigen::Isometry3f() const
{
    float angle = cv::norm(rotation_vector);

    Eigen::Vector3f axis;
    cv::cv2eigen(rotation_vector, axis);
    axis = axis / angle;

    Eigen::AngleAxisf rotation_transformation(angle, axis);
    Eigen::Translation3f translation_transformation(translation_vector.at<double>(0),
                                                    translation_vector.at<double>(1),
                                                    translation_vector.at<double>(2));

    Eigen::Isometry3f transformation(rotation_transformation * translation_transformation);
    return transformation;
}

g2o::EdgeSE3* RGBDFrame::Transformation::AsEdge()
{
    g2o::EdgeSE3 *new_edge = new g2o::EdgeSE3;

    new_edge->vertices()[1] = from_vertex; // note: observation direction is the reverse of transformation direction
    new_edge->vertices()[0] = to_vertex;

    Eigen::Isometry3f frame_transformation = this->operator Eigen::Isometry3f();
    new_edge->setMeasurement(frame_transformation.cast<double>());

    Eigen::Matrix<double, 6, 6> information_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    information_matrix = 100 * information_matrix;
    new_edge->setInformation(information_matrix);

    return new_edge;
}

bool RGBDFrame::Transformation::IsRobustTransform()
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    /* Keyframe criterions */
    if(this->num_matches < params->Value<double>("min_num_matches"))
        return false;

    double rot_angle = cv::norm(this->rotation_vector);
    rot_angle = std::abs(std::min(rot_angle, 2*M_PI - rot_angle));

    double trans_dist = std::abs(cv::norm(this->translation_vector));

    if(rot_angle < params->Value<double>("min_rot_angle") ||
            rot_angle > params->Value<double>("max_rot_angle") ||
            trans_dist < params->Value<double>("min_trans_dist") ||
            trans_dist > params->Value<double>("max_trans_dist")) {
        return false;
    }

    if(params->Value<int>("debug_info_frame"))
        logger_->info("Robust angle: {1:f} translation: {0:f}", trans_dist, rot_angle);

    return true;
}

void RGBDFrame::SetAlgorithms(std::string detector, std::string descriptor, std::string matcher)
{
    if(detector == "SIFT")
        feature_detector_ = cv::xfeatures2d::SIFT::create();
    else if(detector == "SURF")
        feature_detector_ = cv::xfeatures2d::SURF::create();
    else if(detector == "ORB")
        feature_detector_ = cv::ORB::create();
    else if(detector == "FAST")
        feature_detector_ = cv::FastFeatureDetector::create();
    else if(detector == "KAZE")
        feature_detector_ = cv::KAZE::create();
    else {
        logger_->error("Unsupported feature detector {0}!", detector);
        throw std::runtime_error(0);
    }
    logger_->info("Using {0} feature detector.", detector);

    if(descriptor == "SIFT")
        feature_descriptor_ = cv::xfeatures2d::SIFT::create();
    else if(descriptor == "SURF")
        feature_descriptor_ = cv::xfeatures2d::SURF::create();
    else if(descriptor == "ORB")
        feature_descriptor_ = cv::ORB::create();
    else if(descriptor == "KAZE")
        feature_descriptor_ = cv::KAZE::create();
    else if(descriptor == "BRISK")
        feature_descriptor_ = cv::BRISK::create();
    else if(descriptor == "Brief")
        feature_descriptor_ = cv::xfeatures2d::BriefDescriptorExtractor::create();
    else {
        logger_->error("Unsupported feature descriptor {0}!", descriptor);
        throw std::runtime_error(0);
    }
    logger_->info("Using {0} feature descriptor.", descriptor);

    if(matcher == "BruteForce")
        descriptor_matcher_ = new cv::BFMatcher();
    else if(matcher == "FLANN")
        descriptor_matcher_ = new cv::FlannBasedMatcher();
    else {
        logger_->error("Unsupported descriptor matcher {0}!", matcher);
        throw std::runtime_error(0);
    }
    logger_->info("Using {0} descriptor matcher.", matcher);
}
