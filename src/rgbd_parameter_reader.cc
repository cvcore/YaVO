//
// Created by Charles Wang on 06.05.17.
//

#include "rgbd_parameter_reader.h"

#include <fstream>
#include <iostream>
#include <exception>
#include <algorithm>
#include <spdlog/spdlog.h>

RGBDParameterReader* RGBDParameterReader::instance_ = 0;
std::shared_ptr<spdlog::logger> RGBDParameterReader::logger_ = spdlog::stdout_color_mt("reader");


RGBDParameterReader *RGBDParameterReader::Instance()
{
    if(instance_ == NULL) {
        instance_ = new RGBDParameterReader();
    }

    return instance_;
}


void RGBDParameterReader::ReadFromFile(std::string path)
{
    std::ifstream param_stream(path);

    if(!param_stream.is_open()) {
        logger_->error("Parameter file does not exist!");
        throw std::runtime_error(0);
    }

    std::string line;
    while(param_stream.good()) {
        std::getline(param_stream, line);

        /* skip comments */
        if(line[0] == '#')
            continue;

        /* skip empty lines */
        if(line.size() == 0)
            continue;

        std::string::iterator delim = std::find(line.begin(), line.end(), ':');
        if(delim != line.end()) {
            std::string key(line.begin(), delim);
            std::string value_string(delim + 2, line.end());

            kv_map_[key] = value_string;
        }
    }
}


template <>
std::string RGBDParameterReader::Value<std::string> (std::string key)
{
    return kv_map_[key];
}

template <>
double RGBDParameterReader::Value<double> (std::string key)
{
    return std::stod(kv_map_[key]);
}

template <>
int RGBDParameterReader::Value<int> (std::string key)
{
    return std::stoi(kv_map_[key]);
}
