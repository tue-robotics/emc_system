#include "emc/maploader.h"
#include <opencv2/opencv.hpp>

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "Usage: test_maploader YAML_FILENAME" << std::endl;
        return 0;
    }
    MapData mapdata = loadmap(argv[1]);
    std::cout << "resolution is " << mapdata.resolution << std::endl;
    cv::imshow("map", mapdata.map);
    cv::waitKey();
    return 1;
}