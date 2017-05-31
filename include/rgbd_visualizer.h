//
// Created by Charles Wang on 24.05.17.
//

#ifndef TF_ESTIMATION_RGBD_VISUALIZER_H
#define TF_ESTIMATION_RGBD_VISUALIZER_H

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>
#include <thread>
#include <mutex>

#include "share.h"
#include "rgbd_frame.h"

class RGBDVisualizer {
public:
    RGBDVisualizer(bool draw_feature_points, bool draw_camera_pose);
    ~RGBDVisualizer();
    void AddFrame(RGBDFrame& frame);
    void ClearBuffer(bool force_redraw = false, bool clear_pose = true);
    void Spin();
    void SetGridSize(float x, float y, float z);
    void SavePointCloud(std::string path);

private:
    std::mutex mt_visualizer_;
    RGBDCloudPtr buffer_cloud_;
    bool need_redraw_;
    bool pose_need_clearing_;
    /* std::thread vis_worker_thread_; */
    static std::shared_ptr<spdlog::logger> logger_;
    std::vector< std::pair<std::string, Eigen::Isometry3f> > pose_to_add_;
    std::vector<std::string> pose_added_;
    pcl::visualization::PCLVisualizer *pcl_visualizer_;
    pcl::VoxelGrid<RGBDPoint> voxel_filter_;
    bool draw_feature_points_;
    bool draw_camera_pose_;

    void VisualizationWorker();
    void InitializeCVWindow();
    bool CVWindowExists();
};


#endif //TF_ESTIMATION_RGBD_VISUALIZER_H
