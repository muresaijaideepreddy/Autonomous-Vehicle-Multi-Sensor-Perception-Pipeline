#include "object_classification.hpp"

std::string classifyObject(const BoundingBox& box)
{
    const float length = box.max_x - box.min_x;
    const float width  = box.max_y - box.min_y;
    const float height = box.max_z - box.min_z;
    const float ratio  = (width > 0.01f) ? (length / width) : 0.0f;

    if (length > 6.0f && width > 2.0f && height > 2.0f)
        return "TRUCK";

    if (length > 2.5f && length < 7.0f &&
        width  > 1.2f && width  < 3.5f &&
        height > 0.5f && height < 2.5f &&
        ratio  > 1.0f && ratio  < 4.0f)
        return "CAR";

    if (length > 1.0f && length < 3.5f &&
        width  > 0.3f && width  < 1.8f &&
        height > 0.8f && height < 2.5f)
        return "CYCLIST";

    if (length > 0.2f && length < 1.5f &&
        width  > 0.2f && width  < 1.5f &&
        height > 0.8f && height < 2.5f)
        return "PEDESTRIAN";

    return "UNKNOWN";
}