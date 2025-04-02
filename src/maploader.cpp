#include "emc/maploader.h"
#include <yaml-cpp/yaml.h>
#include <filesystem>

MapData loadmap(std::string yaml_filename){
    std::filesystem::path yaml_filepath(yaml_filename);
    YAML::Node config = YAML::LoadFile(yaml_filename);

    double resolution = config["resolution"].as<double>();

    // find the image file
    std::string filename = config["image"].as<std::string>();
    std::filesystem::path image_filepath = yaml_filepath;
    image_filepath.replace_filename(filename);
    if (!std::filesystem::exists(image_filepath))
    {
        std::cerr << "Image not found at " << image_filepath.string() << std::endl;
        return MapData();
    }
    
    cv::Mat map = cv::imread(image_filepath.string(), cv::IMREAD_GRAYSCALE);
    if (map.empty())
    {
        std::cerr << "Failed to read image at " << image_filepath.string() << std::endl;
        return MapData();
    }

    MapData mapdata;
    mapdata.map = map;
    mapdata.resolution = resolution;
    return mapdata;
}