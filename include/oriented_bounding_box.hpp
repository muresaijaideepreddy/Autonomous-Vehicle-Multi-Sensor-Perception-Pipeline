#pragma once
struct OrientedBoundingBox {
    float center_x;
    float center_y;
    float center_z;
    float length;
    float width;
    float height;
    float yaw; // Angle in radians
};