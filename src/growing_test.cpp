
#include "riparian_region_growing/riparian_grower.hpp"


int main(int argc, char *argv[])
{
    // Load User Input Parameters
    std::string channel_name = argv[1];
    int stream_order = std::atoi(argv[2]);

    RiparianGrower<pcl::Point2DGround, pcl::PointVeg> grower;
    // Load Channel Data
    grower.readChannelNetworkNHDPlus("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/flowlines/flowlines_santa_marg_lower.shp", "flowlines_santa_marg_lower");
    // Filter to only include streams of Strahler order 3 or more, and only from the Santa Margarita River
    grower.filterByChannelName(channel_name, "GNIS_Nm");
    grower.filterByChannelOrder(stream_order, "StrmOrd");
    // Load Point Cloud Data
    grower.readDEMPCD("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/62122030_ground_filtered.pcd");
    grower.readVegCloudPCD("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/62122030_vegetation.pcd");
    grower.generateGroundTIN();
    // Run Analysis
    grower.extractVegetationStatistics("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/vegetation_statistics/santa_margarita_veg_stats.csv");
}  