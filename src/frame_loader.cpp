#include "frame.hpp"
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iostream>

std::vector<std::string> getFrameFileList(const std::string& directory)
{
    std::vector<std::string> file_list;
    for (const auto& entry : std::filesystem::directory_iterator(directory))
    {
        if (entry.is_regular_file())
        {
            file_list.push_back(entry.path().string());
        }
    }
    return file_list;
}


namespace fs = std::filesystem;

FrameSequenceLoader::FrameSequenceLoader(const std::string& folder_path)
    : current_index_(0)
{
    frame_paths_ = getFrameFileList(folder_path);
}

bool FrameSequenceLoader::hasNext() const
{
    return current_index_ < frame_paths_.size();
}

std::vector<PointsXYZI> FrameSequenceLoader::next()
{
    if (!hasNext())
    {
        return {};
    }

    const std::string& file_path = frame_paths_[current_index_++];
    return loadPointCloudBIN(file_path);
}

void FrameSequenceLoader::reset()
{
    current_index_ = 0;
}

std::vector<std::string> FrameSequenceLoader::getFrameFileList(const std::string& folder_path)
{
    std::vector<std::string> files;

    for (const auto& entry : fs::directory_iterator(folder_path))
    {
        if (entry.path().extension() == ".bin")
        {
            files.push_back(entry.path().string());
        }
    }

    std::sort(files.begin(), files.end());
    return files;
}

std::vector<PointsXYZI> FrameSequenceLoader::loadPointCloudBIN(const std::string& file_path)
{
    std::vector<PointsXYZI> points;

    std::ifstream file(file_path, std::ios::binary);
    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return points;
    }

    while (file.good() && !file.eof())
    {
        PointsXYZI point;
        file.read(reinterpret_cast<char*>(&point), sizeof(PointsXYZI));

        if (file.gcount() == sizeof(PointsXYZI))
        {
            points.push_back(point);
        }
    }

    file.close();
    return points;
}
