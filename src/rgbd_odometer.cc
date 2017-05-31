//
// Created by Charles Wang on 30.03.17.
//

#include "rgbd_odometer.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <spdlog/spdlog.h>
#include <iostream>

#include "rgbd_parameter_reader.h"

std::shared_ptr<spdlog::logger> RGBDOdometer::logger_ = spdlog::stdout_color_mt("odometer");

RGBDOdometer::RGBDOdometer()
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    // G2O
    SlamLinearSolver* linear_solver_ptr_ = new SlamLinearSolver();
    linear_solver_ptr_->setBlockOrdering(false);

    SlamBlockSolver* block_solver_ptr_ = new SlamBlockSolver(linear_solver_ptr_);
    g2o::OptimizationAlgorithmLevenberg* levenberg_solver_ = new g2o::OptimizationAlgorithmLevenberg(block_solver_ptr_);

    global_optimizer_ = std::make_shared<g2o::SparseOptimizer>();
    global_optimizer_->setAlgorithm(levenberg_solver_);

    // Point Cloud
    point_cloud_needs_redraw_ = false;

    // Configurations
    enable_downsampling_ = true;
    downsampling_grid_size_ = params->Value<double>("grid_size");

    rgbd_visualizer_ = NULL;
}

void RGBDOdometer::AddFrame(RGBDFrame new_frame)
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();
    int debug_info_odometer = params->Value<int>("debug_info_odometer");

    // First frame
    if(keyframes_.size() == 0) {
        // *point_cloud_ += *new_frame.ComputeRGBDCloud();
        new_frame.SetPoseEstimation(Eigen::Isometry3f(Eigen::AngleAxisf(- M_PI, Eigen::Vector3f::UnitX())));
        auto new_vertex = new_frame.AsVertex();
        new_vertex->setFixed(true);
        global_optimizer_->addVertex(new_vertex);
        new_frame.is_keyframe = true;
        keyframes_.push_back(new_frame);

        if(rgbd_visualizer_) {
            rgbd_visualizer_->AddFrame(new_frame);
        }

        return;
    }

    auto frame_transformation = new_frame.TransformTo(keyframes_.back());

    if(debug_info_odometer) {
        logger_->info("num_matches: {0:d}", frame_transformation.num_matches);
    }

    if(!frame_transformation.IsRobustTransform())
        return;

    new_frame.is_keyframe = true;
    new_frame.SetPoseEstimation(keyframes_.back().Pose() * frame_transformation);

    //if(enable_downsampling_)
    //    this->DownSamplePointCloud();

    // Add new vertex / edge to global optimizer
    global_optimizer_->addVertex(new_frame.AsVertex());
    global_optimizer_->addEdge(frame_transformation.AsEdge());

    int local_vertices = params->Value<int>("local_vertices");
    for(int local_idx = keyframes_.size() - 1; local_idx >= 0 && keyframes_.size() - local_idx <= local_vertices; local_idx--) {
        auto local_tf = new_frame.TransformTo(keyframes_[local_idx]);
        if(local_tf.IsRobustTransform()) {
            global_optimizer_->addEdge(local_tf.AsEdge());
            if(params->Value<int>("debug_info_odometer")) {
                logger_->info("local edge added");
            }
        }
    }

    int rand_vertices = params->Value<int>("random_vertices");
    std::map<int, bool> selected;
    for(int rand_cnt = 0; rand_cnt < (int)keyframes_.size() - local_vertices && rand_cnt < rand_vertices; rand_cnt++) {
        int rand_idx = std::rand() % (keyframes_.size() - local_vertices);

        if(selected[rand_idx]) /* vertex have been chosen */
            continue;

        selected[rand_idx] = true;
        auto rand_tf = new_frame.TransformTo(keyframes_[rand_idx]);
        if(rand_tf.IsRobustTransform()) {
            global_optimizer_->addEdge(rand_tf.AsEdge());
            if(params->Value<int>("debug_info_odometer")) {
                logger_->info("random edge added");
            }
        }
    }

    if(rgbd_visualizer_) {
        rgbd_visualizer_->AddFrame(new_frame);
    }

    keyframes_.push_back(new_frame);
}

void RGBDOdometer::OptimizeGlobalGraph()
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    global_optimizer_->save("initial_graph.g2o");
    global_optimizer_->initializeOptimization();
    global_optimizer_->optimize(params->Value<double>("g2o_iterations"));
    global_optimizer_->save("optimized_graph.g2o");

    point_cloud_needs_redraw_ = true;
}

void RGBDOdometer::RedrawPointCloud()
{
    RGBDParameterReader *params = RGBDParameterReader::Instance();

    if(rgbd_visualizer_) {
        rgbd_visualizer_->ClearBuffer(false, true);
    }

    for(RGBDFrame frame: keyframes_) {
        RGBDCloud::Ptr frame_cloud = frame.ComputeRGBDCloud();

        if(rgbd_visualizer_) {
            rgbd_visualizer_->AddFrame(frame);
        }
    }

    point_cloud_needs_redraw_ = false;
}


void RGBDOdometer::SetVisualizer(RGBDVisualizer *visualizer)
{
    rgbd_visualizer_ = visualizer;
}

void RGBDOdometer::ExportKeyFrames(std::string path)
{
    std::ofstream filestream(path);

    for(auto keyframe: keyframes_) {
        filestream << keyframe << "\n";
    }

    filestream.close();
}
