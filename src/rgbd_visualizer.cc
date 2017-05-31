//
// Created by Charles Wang on 24.05.17.
//

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "rgbd_visualizer.h"
#include "rgbd_parameter_reader.h"

std::shared_ptr<spdlog::logger> RGBDVisualizer::logger_ = spdlog::stdout_color_mt("visualizer");

RGBDVisualizer::RGBDVisualizer(bool draw_feature_points, bool draw_camera_pose)
{
    buffer_cloud_ = RGBDCloudPtr(new RGBDCloud());
    need_redraw_ = false;
    pose_need_clearing_ = false;
    draw_feature_points_ = draw_feature_points;
    draw_camera_pose_ = draw_camera_pose;

    /* note: due to macOS system limitations, we have to create PCLVisualizer on main thread */
    /* ref.: https://github.com/PointCloudLibrary/pcl/issues/253 */
    pcl_visualizer_ = new pcl::visualization::PCLVisualizer("Cloud Viewer");
    //vis_worker_thread_ = std::thread(&RGBDVisualizer::VisualizationWorker, this);
    if(draw_camera_pose_)
        this->InitializeCVWindow();
}

void RGBDVisualizer::Spin() {
    this->VisualizationWorker();
}

void RGBDVisualizer::VisualizationWorker()
{
    logger_->info("visualization thread created.");

    mt_visualizer_.lock();
    pcl_visualizer_->addPointCloud<RGBDPoint>(buffer_cloud_);
    mt_visualizer_.unlock();

    while(!pcl_visualizer_->wasStopped()) {
        bool mt_acquired = mt_visualizer_.try_lock();

        if(mt_acquired) {
            if(need_redraw_) {
                //pcl_visualizer_->updatePointCloud<RGBDPoint>(buffer_cloud_);
                pcl_visualizer_->removePointCloud();
                pcl_visualizer_->addPointCloud<RGBDPoint>(buffer_cloud_);

                /* clearing added poses */
                if (pose_need_clearing_) {
                    while (pose_added_.size() > 0) {
                        pcl_visualizer_->removeCoordinateSystem(pose_added_.back());
                        pose_added_.pop_back();
                    }
                    pose_need_clearing_ = false;
                }

                /* add new poses */
                while (pose_to_add_.size() > 0) {
                    auto name_pose_pair = pose_to_add_.back();

                    pcl_visualizer_->addCoordinateSystem(0.1, name_pose_pair.second, name_pose_pair.first);
                    pose_added_.push_back(name_pose_pair.first);
                    pose_to_add_.pop_back();
                }

                need_redraw_ = false;
            }

            mt_visualizer_.unlock();
        }

        pcl_visualizer_->spinOnce();

        if(draw_feature_points_)
            cv::waitKey(1);
    }

    mt_visualizer_.lock();
    pcl_visualizer_->removePointCloud();
    delete pcl_visualizer_;
    pcl_visualizer_ = NULL;
    mt_visualizer_.unlock();

    if(draw_feature_points_)
        cv::destroyAllWindows();
}

RGBDVisualizer::~RGBDVisualizer()
{
}

void RGBDVisualizer::AddFrame(RGBDFrame& frame)
{
    static int i = 0;

    RGBDCloudPtr transformed_cloud(new RGBDCloud());
    pcl::transformPointCloud(*frame.ComputeRGBDCloud(), *transformed_cloud, frame.Pose(), true);

    std::stringstream ss_frame_id;
    ss_frame_id << "frame_id=" << frame.id;

    if(draw_feature_points_)
        cv::imshow("Feature", frame.RGBImageWithFeature());

    mt_visualizer_.lock();

    if(pcl_visualizer_ == NULL) {
        logger_->error("trying to add frame to terminated visualizer!");
        throw std::runtime_error(0);
    }

    *buffer_cloud_ += *transformed_cloud;

    if((i++) % 5 == 0) {
        voxel_filter_.setInputCloud(buffer_cloud_);
        voxel_filter_.filter(*buffer_cloud_);
    }

    if(draw_camera_pose_) {
        pose_to_add_.push_back(std::make_pair(ss_frame_id.str(), frame.Pose()));
    }
    need_redraw_ = true;

    mt_visualizer_.unlock();
}

void RGBDVisualizer::ClearBuffer(bool force_redraw, bool clear_pose)
{
    mt_visualizer_.lock();

    buffer_cloud_ = RGBDCloudPtr(new RGBDCloud());
    need_redraw_ = force_redraw;
    pose_need_clearing_ = clear_pose;

    mt_visualizer_.unlock();
}

void RGBDVisualizer::SetGridSize(float x, float y, float z)
{
    std::lock_guard<std::mutex> lock(mt_visualizer_);
    voxel_filter_.setLeafSize(x, y, z);
}

void RGBDVisualizer::InitializeCVWindow()
{
    cv::namedWindow("Feature");
    cv::resizeWindow("Feature", 640, 480);
    cv::setWindowTitle("Feature", "Feature");
}

void RGBDVisualizer::SavePointCloud(std::string path)
{
    std::lock_guard<std::mutex> lock(mt_visualizer_);

    pcl::io::savePCDFileASCII(path, *buffer_cloud_);
}

