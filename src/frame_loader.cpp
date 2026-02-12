#include "frame.hpp"
#include <filesystem>

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