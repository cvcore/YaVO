//
// Created by Charles Wang on 06.05.17.
//

#ifndef TF_ESTIMATION_RGBD_PARAMETER_READER_H
#define TF_ESTIMATION_RGBD_PARAMETER_READER_H

#include <string>
#include <map>
#include <spdlog/spdlog.h>

class RGBDParameterReader {

public:
    static RGBDParameterReader* Instance();
    template <typename T> T Value(std::string key);
    void ReadFromFile(std::string path);

private:
    static RGBDParameterReader* instance_;
    static std::shared_ptr<spdlog::logger> logger_;
    std::map<std::string, std::string> kv_map_;
};


#endif //TF_ESTIMATION_RGBD_PARAMETER_READER_H
