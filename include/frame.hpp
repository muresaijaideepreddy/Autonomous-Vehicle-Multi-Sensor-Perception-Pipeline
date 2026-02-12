#include <vector>
#include "pointcloud_loader.hpp"
struct Frame
{
    double timestamp;
    std::vector<PointsXYZ> points;

};
std::vector<std::string> getFrameFileList(const std::string& directory);