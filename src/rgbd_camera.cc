#include "rgbd_camera.h"

#include <cassert>
#include <map>
#include <spdlog/spdlog.h>

using namespace pcl;

std::shared_ptr<spdlog::logger> RGBDCamera::logger_ = spdlog::stdout_color_mt("camera");

RGBDCamera::RGBDCamera(double ax, double ay, double x0, double y0, double s, double scale):
    ax(ax), ay(ay), x0(x0), y0(y0), s(s), scale(scale)
{
}

/*x = KR[I | -C]X
 *
 * K = [ax s x0]
 *     [0 ay y0]
 *     [0  0  1]
 *
 * x = ax*X/Z + x0
 * y = ay*Y/Z + y0
 * X = (x - x0) * Z / ax;
 * Y = (y - y0) * Z / ay;
 */

int RGBDCamera::ReadAssociatedSequence(std::string path)
{
    std::ifstream sequence_file(path);
    std::array<char, 256> path_line;
    std::string base_path;

    if(!sequence_file.is_open()) {
        logger_->error("cannot open sequence file: {0}", path);
        throw std::runtime_error(0);
    }

    size_t pos_last_slash = path.find_last_of("/");
    base_path = path.substr(0, pos_last_slash + 1);

    frame_count_ = 0;

    while(sequence_file.good()) {
        sequence_file.getline(&path_line[0], 256);

        if(path_line[0] == 0) /* end of file */
            break;

        std::stringstream pathline_stream(&path_line[0]);
        std::string rgb_path;
        std::string depth_path;
        std::string rgb_timestamp;
        std::string depth_timestamp;

        pathline_stream >> depth_timestamp >> depth_path >> rgb_timestamp >> rgb_path;

        depth_path = base_path + depth_path;
        depth_paths_.push_back(depth_path);
        depth_timestamps_.push_back(std::stod(depth_timestamp));
        rgb_path = base_path + rgb_path;
        rgb_paths_.push_back(rgb_path);
        rgb_timestamps_.push_back(std::stod(rgb_timestamp));

        frame_count_++;
    }

    return frame_count_;
}

cv::Mat RGBDCamera::IntrinsicsMatrix() const
{
    cv::Mat intrinsics_matrix = cv::Mat(3, 3, CV_32F);
    intrinsics_matrix.at<float>(0, 0) = ax;
    intrinsics_matrix.at<float>(0, 2) = x0;
    intrinsics_matrix.at<float>(1, 1) = ay;
    intrinsics_matrix.at<float>(1, 2) = y0;
    return intrinsics_matrix;
}

cv::Mat RGBDCamera::DistortionCoefficients() const
{
    return cv::Mat(1, 5, CV_32F, cv::Scalar(0.0));
}

RGBDFrame RGBDCamera::RequestFrame(int frame_id)
{
    static std::map<int, int> idx_map;
    assert(frame_id < frame_count_);

    std::map<int, int>::iterator idx_it = idx_map.find(frame_id);
    if(idx_it != idx_map.end()) {
        return frames_[idx_it->second];
    }

    logger_->info("reading RGBD image pair #{0:d}", frame_id);

    cv::Mat rgb_image = cv::imread(rgb_paths_[frame_id]);
    if(!rgb_image.data) {
        logger_->error("can't read rgb image {0}", rgb_paths_[frame_id]);
        throw std::runtime_error(0);
    }
    double rgb_timestamp_d = rgb_timestamps_[frame_id];

    cv::Mat depth_image = cv::imread(depth_paths_[frame_id], cv::IMREAD_UNCHANGED);
    if(!depth_image.data) {
        logger_->error("can't read depth image {0}", depth_paths_[frame_id]);
        throw std::runtime_error(0);
    }
    double depth_timestamp_d = depth_timestamps_[frame_id];

    RGBDFrame new_frame(rgb_image, depth_image, rgb_timestamp_d, depth_timestamp_d, this, frame_id);
    frames_.push_back(new_frame);
    idx_map[frame_id] = frames_.size() - 1;

    return new_frame;
}
