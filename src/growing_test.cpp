
#include "riparian_region_growing/riparian_grower.hpp"


int main(int argc, char *argv[])
{
//    readChannelNetwork("/mnt/d/serdp/data/pendleton/NHDPlus/derived/flowlines_vaa_ca.shp", 1);

    RiparianGrower<pcl::Point2DGround, pcl::PointVeg> grower;
    // Load Channel Data
    grower.readChannelNetworkNHDPlus("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/flowlines/flowlines_santa_marg_lower.shp", "flowlines_santa_marg_lower");
    // Filter to only include streams of Strahler order 3 or more, and only from the Santa Margarita River
    grower.filterByChannelName("Santa Margarita River", "GNIS_Nm");
    grower.filterByChannelOrder(3, "StreamOrd");
    // Load Point Cloud Data
    grower.readDEMPCD("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/62122030_ground_filtered.pcd");
    grower.readVegCloudPCD("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/62122030_vegetation.pcd");
    grower.generateGroundTIN();
    // Run Analysis
    grower.extractVegetationStatistics("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/vegetation_statistics/santa_margarita_veg_stats.csv");
}  