#pragma once
#include <cstdint>
struct PointsXYZ{
    float X ;
    float Y ;
    float Z;
};

struct PointsXYZI{
    float X ;
    float Y ;
    float Z;
    float Intensity;
};
struct PointsXYZRGB {
    float x, y, z;
    uint8_t r, g, b;
};