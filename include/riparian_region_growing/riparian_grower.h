
// ** GDAL Includes **
//   Opening Shapefile Vector files, GeoTIFF Raster files
#include <gdal/ogrsf_frmts.h>

// ** Point Cloud Library Includes **
//   Handling unordered clouds of points in XYZ space
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/io/pcd_io.h>
#include <dirt_or_leaf/las_point_types.h>
//#include <dirt_or_leaf/las_filtering.hpp>         included in dirt_or_leaf/ground_tin.hpp
//#include <dirt_or_leaf/las_triangulation.hpp>     included in dirt_or_leaf/ground_tin.hpp
#include <dirt_or_leaf/ground_tin.hpp>
// Instantiate new PCL point types
#include "riparian_region_growing/reli.h"
#include "riparian_region_growing/point_channel.h"
#include <pcl/impl/instantiate.hpp>
// Eigen - Linear Math Library
#include <Eigen/Dense>
#include <cmath>

template <typename DEMType, typename VegType, typename ChannelType>
class RiparianGrower{

public:
    // Defines for PCL PointCloud templates on custom point types
    typedef typename pcl::PointCloud<DEMType> GC;
    typedef typename pcl::PointCloud<DEMType>::Ptr GCP;
    typedef typename pcl::PointCloud<VegType> VC;
    typedef typename pcl::PointCloud<VegType>::Ptr VCP;
    typedef typename pcl::PointCloud<ChannelType> CC;
    typedef typename pcl::PointCloud<ChannelType>::Ptr CCP;
    // Defines for PCL KD Search Tree templates on custom point types
    typedef typename pcl::KdTreeFLANN<DEMType> GT;
    typedef typename pcl::KdTreeFLANN<DEMType>::Ptr GTP;
    typedef typename pcl::KdTreeFLANN<VegType> VT;
    typedef typename pcl::KdTreeFLANN<VegType>::Ptr VTP;

    // Constructor
    RiparianGrower();
    // Destructor
    ~RiparianGrower();

    // Stream Channel Segment
    struct LineSegment
    {
        GC geometry_gc;
        CC geometry_cc;
        int stream_order;
        int segment_id;
        std::string channel_name;
        std::vector<int> swath_indices_veg;
        std::vector<int> swath_indices_grd;
    };

// *** Member Functions ***
// Load Input Data
    //   Load a .shp channel network, assuming it to have a structure similar to NHDPlus. Optionally, filter channels to be over a certain Strahler order (only if order > 0)
    void readChannelNetworkNHDPlus(std::string filename, std::string layer_name);
    //   Load a Digital Elevation Model containing local terrain height (recommend to use at least 5 m resolution) 
    bool readDEMPCD(std::string filename);
    //   Load Vegetation Point Cloud 
    bool readVegCloudPCD(std::string filename);

// Filters on Channel Network 
    //   Set to filter channels to include only those with a certain name (e.g. "Santa Margarita River")
    void filterByChannelName(std::string channel_name, std::string attribute_name);
    //   Set to filter channels to include only those with at least a certain Strahler order (e.g. 3)
    void filterByChannelOrder(int channel_order, std::string attribute_name);

// Get Ground TIN
    void generateGroundTIN();

// Set Growing Parameters
    // Only a nearest K neighbor search OR a search to find all neighbors in R distance will be performed
    // Which of these is used is toggled when the below functions are called
    void setNeighborSearchDistance(float search_distance);
    void setNeighborSearchCount(int neighbor_count);

// Extract Vegetation Statistics
    void extractVegetationStatistics(std::string output_file_name);

private:
// *** Member Objects ***
// Data for Stream Flowlines OGR Object
    GDALDataset *flowlines_dataset_;
    OGRLayer *flowlines_layer_;
    OGRFeature *first_segment_;
    OGRField *flowline_field_;
    std::string channel_name_;
    int channel_name_attribute_index_;
    int channel_order_;
    int channel_order_attribute_index_;

// Data for DEM
    GCP dem_cloud_;
    std::vector<int> dem_cloud_index_map_;
    GTP dem_tree_;
    GroundTIN<DEMType> TIN_;

// Data for Vegetation Points
    VCP veg_cloud_;
    std::vector<int> veg_cloud_index_map_;
    VTP veg_tree_;

// Growing Search Parameters
    float search_distance_;
    int neighbor_count_;
    bool search_by_distance_;

// *** Private Member Functions ***
// Operations on Channel Network Segments
    // Cartesian direction of local channel segment
    void getSegmentDirection(LineSegment &segment);
    // Get swath of vegetation point cloud perpendicular to channel
    void getPerpendicularSwath(LineSegment &segment);
    // Region Growing Function
    void regionGrowingVeg(ChannelType &point, std::vector<int> &index_list);
    void regionGrowingRecursorVeg(ChannelType &point, VegType new_point, int new_ind, std::vector<int> &index_list);
    // Region Growing Condition - to be checked for each new candidate point
    bool growingConditionVeg(ChannelType point, VegType point_new);
    // Get indices of points within perpendicular swath
    void getSwathPointIndices(LineSegment &segment);
    // Get elevation of an individual point in the channel network, using DEM
    float getChannelElevation(DEMType point);

    // Basic Geometric Functions
    template <typename FirstType, typename SecondType>
    float point2DDistance(FirstType point_1, SecondType point_2);
    template <typename FirstType, typename SecondType>
    float point3DDistance(FirstType point_1, SecondType point_2);
    template <typename FirstType, typename SecondType>
    float pointDistanceAlongVector(FirstType point_1, SecondType point_2, Eigen::Vector3f vec);
    // Get a simple unit vector in XY plane pointing in direction of a cartesian angle 
    Eigen::Vector3f angleFromAzimuth(float angle);

};
