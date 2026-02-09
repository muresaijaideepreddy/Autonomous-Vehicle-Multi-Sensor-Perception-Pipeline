#pragma once
#include "bounding_box.hpp"
#include "euclidean_clustering.hpp"

BoundingBox computeAABB(const Cluster& cluster);

bool isValidCluster(const Cluster& cluster);