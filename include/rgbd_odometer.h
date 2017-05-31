// For global camera trajectory optimization and point cloud registration
// Created by Charles Wang on 30.03.17.
//

#ifndef TF_ESTIMATION_RGBD_ODOMETER_H
#define TF_ESTIMATION_RGBD_ODOMETER_H

#include <vector>
#include <memory>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

#include "rgbd_frame.h"
#include "rgbd_camera.h"
#include "rgbd_visualizer.h"
#include "share.h"

class RGBDOdometer {
public:
    RGBDOdometer();
    void AddFrame(RGBDFrame new_frame);
    void OptimizeGlobalGraph();
    void RedrawPointCloud();
    void SetVisualizer(RGBDVisualizer* visualizer);
    void ExportKeyFrames(std::string path);

private:

    typedef g2o::BlockSolver_6_3 SlamBlockSolver;
    typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    std::shared_ptr<g2o::SparseOptimizer> global_optimizer_;
    bool point_cloud_needs_redraw_;
    bool enable_downsampling_;
    double downsampling_grid_size_;
    static std::shared_ptr<spdlog::logger> logger_;
    std::vector<RGBDFrame> keyframes_;
    RGBDVisualizer* rgbd_visualizer_;

    void DownSamplePointCloud();
};


#endif //TF_ESTIMATION_RGBD_ODOMETER_H
