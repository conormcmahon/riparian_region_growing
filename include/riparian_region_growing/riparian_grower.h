
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

void readChannelNetwork(std::string filename, float raster_object);

template <typename DEMType, typename VegType>
class RiparianGrower{

public:
    // Defines for PCL PointCloud templates on custom point types
    typedef typename pcl::PointCloud<DEMType> GC;
    typedef typename pcl::PointCloud<DEMType>::Ptr GCP;
    typedef typename pcl::PointCloud<VegType> VC;
    typedef typename pcl::PointCloud<VegType>::Ptr VCP;
    // Defines for PCL KD Search Tree templates on custom point types
    typedef typename pcl::KdTreeFLANN<DEMType> GT;
    typedef typename pcl::KdTreeFLANN<VegType> VT;
    typedef typename pcl::KdTreeFLANN<DEMType>::Ptr GTP;
    typedef typename pcl::KdTreeFLANN<VegType>::Ptr VTP;

    // Constructor
    RiparianGrower();
    // Destructor
    ~RiparianGrower();

    // Stream Channel Segment
    struct LineSegment
    {
        GC geometry_gc;
        VC geometry_vc;
        int stream_order;
        int segment_id;
        std::string channel_name;
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
    GTP dem_tree_;
    GroundTIN<DEMType> TIN_;

// Data for Vegetation Points
    VCP veg_cloud_;
    VTP veg_tree_;

// *** Private Member Functions ***
// Operations on Channel Network Segments
    // Cartesian direction of local channel segment
    void getSegmentDirection();
    // Get swath of vegetation point cloud perpendicular to channel
    void getPerpendicularSwath();
    // Get indices of points within perpendicular swath
    void getSwathPointIndices();
    // Get elevation of an individual point in the channel network, using DEM
    float getChannelElevation(DEMType point);
};
