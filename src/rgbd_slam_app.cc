#include <iostream>
#include <cstdlib>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>
#include <ctime>

#include "rgbd_camera.h"
#include "rgbd_odometer.h"
#include "rgbd_parameter_reader.h"
#include "rgbd_visualizer.h"

RGBDVisualizer rgbd_visualizer(true, true);

void slam_worker(int argc, const char *argv[]) {
    RGBDParameterReader *params = RGBDParameterReader::Instance();
    params->ReadFromFile(argv[4]);

    RGBDCamera camera(params->Value<double>("camera_ax"),
                      params->Value<double>("camera_ay"),
                      params->Value<double>("camera_x0"),
                      params->Value<double>("camera_y0"),
                      params->Value<double>("camera_s"),
                      params->Value<double>("camera_scale"));

    rgbd_visualizer.SetGridSize(params->Value<double>("grid_size"),
                                params->Value<double>("grid_size"),
                                params->Value<double>("grid_size"));

    RGBDOdometer odometer;
    odometer.SetVisualizer(&rgbd_visualizer);

    int from_frame, until_frame;
    from_frame = atoi(argv[2]);
    until_frame = atoi(argv[3]);

    auto logger = spdlog::stdout_color_mt("SLAM");

    logger->info("reading from file {0} from frame {1:d} to {2:d}", argv[1], from_frame, until_frame - 1);

    camera.ReadAssociatedSequence(argv[1]);

    RGBDFrame::SetAlgorithms(params->Value<std::string>("detector"),
                             params->Value<std::string>("descriptor"),
                             params->Value<std::string>("matcher"));

    if(params->Value<int>("interactive")) {
        logger->info("Hit enter to start...");
        std::getchar();
    }

    bool benchmark_mode = params->Value<int>("benchmark_mode");

    for(int i = 0; i < until_frame - from_frame; i++) {
        std::clock_t start = std::clock();
        // RGBDCamera & RGBDFrame test
        //visualizer.addPointCloud<RGBDPoint>(camera.frames[i].ComputeRGBDCloud());
        //visualizer.spin();
        ////while(!viewer.wasStopped());
        //visualizer.removePointCloud();

        //cv::Mat keypoint_image;
        //cv::drawKeypoints(camera.frames[i].rgb, camera.frames[i].rgb_keypoints_, keypoint_image);
        //cv::imshow("keypoints", keypoint_image);
        //cv::waitKey(0);

        odometer.AddFrame(camera.RequestFrame(from_frame + i));

        double frame_processing_time = (std::clock() - start) / (double) CLOCKS_PER_SEC;

        if(benchmark_mode) {
            logger->info("frame processing time: {0:f}", frame_processing_time);
        }
    }

    odometer.ExportKeyFrames("keyframes.txt");

    if(params->Value<int>("interactive")) {
        logger->info("Hit enter to start graph optimization");
        std::getchar();
    }

    logger->info("optimizing global pose graph");
    odometer.OptimizeGlobalGraph();

    odometer.ExportKeyFrames("keyframes_opt.txt");

    logger->info("redrawing optimized pointcloud");
    odometer.RedrawPointCloud();

    rgbd_visualizer.SavePointCloud("point_cloud.pcd");
}

/* TestRGBDCamera associate.txt from_frame num_frames params.txt */
int main(int argc, const char *argv[])
{

    std::thread slam_thread(slam_worker, argc, argv);

    rgbd_visualizer.Spin();
    //DMatchesPtr matches = camera.frames[0].Match(camera.frames[1]);
    //std::cout << "matches:\n";
    //for(cv::DMatch match : *matches) {
    //    std::cout << "query: " << match.queryIdx << " train: " << match.trainIdx << " dist: " << match.distance << '\n';
    //}
    //cv::Mat matches_image;
    //cv::drawMatches(camera.frames[0].rgb, camera.frames[0].rgb_keypoints_, camera.frames[1].rgb, camera.frames[1].rgb_keypoints_, *matches, matches_image);
    //cv::imshow("matches", matches_image);
    //cv::waitKey(0);
    slam_thread.join();

    return 0;
}
