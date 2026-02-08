#include "visuvalize_clusters.hpp"
#include <fstream>

void saveClusters(const std::vector<Cluster>& clusters)
{
    std::ofstream out("clusters.xyz");
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        for (const auto& p : clusters[i].points)
        {
            out << p.X << " " << p.Y << " " << p.Z << " " << i << "\n";
        }
    }
}
