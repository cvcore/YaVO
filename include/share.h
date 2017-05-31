/* share.h
   Shared common definitions
   Author: Charles Wang
   Date: 28.03.2017
*/

#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

/* Points */
// RGBD
typedef pcl::PointXYZRGB RGBDPoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> RGBDCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBDCloudPtr;
// 2D / 3D points
typedef std::vector<cv::Point2f> Points2D;
typedef std::shared_ptr< std::vector<cv::Point2f> > Points2DPtr;
typedef std::vector<cv::Point3f> Points3D;
typedef std::shared_ptr< std::vector<cv::Point3f> > Points3DPtr;

/* Matches */
typedef std::shared_ptr< std::vector<cv::DMatch> > DMatchesPtr;
inline bool dmatches_min_operator(cv::DMatch op1, cv::DMatch op2) {
    return op1.distance < op2.distance;
}
