#include <vector>
#include "pointcloud_loader.hpp"
struct Frame
{
    double timestamp;
    std::vector<PointsXYZI> points;

};
class FrameSequenceLoader
{
    public :
        explicit FrameSequenceLoader(const std::string& directory);
        bool hasNext() const;
        std::vector<PointsXYZI> next();
        void reset();

    private:
        std::vector<std::string> frame_paths_;
        size_t current_index_;
        std::vector<std::string> getFrameFileList(const std::string& folder_path);
        std::vector<PointsXYZI> loadPointCloudBIN(const std::string& file_path);
};