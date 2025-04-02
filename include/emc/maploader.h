#include <opencv2/opencv.hpp>
#include <string>

struct MapData{
    cv::Mat map;
    double resolution;
};

MapData loadmap(std::string yaml_filename);

