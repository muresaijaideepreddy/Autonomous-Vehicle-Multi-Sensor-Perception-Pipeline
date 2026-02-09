#include "object_classification.hpp"

std::string classifyObject(const BoundingBox& box)
{
    float length = box.max_x - box.min_x;
    float width  = box.max_y - box.min_y;
    float height = box.max_z - box.min_z;

    if (length > 2.5f && length < 6.0f &&
        width  > 1.2f && width  < 3.0f &&
        height > 1.2f && height < 2.5f)
        return "CAR";

    if (length > 0.2f && length < 1.0f &&
        width  > 0.2f && width  < 1.0f &&
        height > 1.2f && height < 2.2f)
        return "PEDESTRIAN";

    if (length > 1.0f && length < 3.0f &&
        width  > 0.4f && width  < 1.5f &&
        height > 1.2f && height < 2.5f)
        return "CYCLIST";

    if (length > 6.0f &&
        width  > 2.0f &&
        height > 2.5f)
        return "TRUCK";

    return "UNKNOWN";
}
