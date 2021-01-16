
#include "riparian_region_growing/riparian_grower.h"

// Constructor 
template <typename DEMType, typename VegType, typename ChannelType>
RiparianGrower<DEMType, VegType, ChannelType>::RiparianGrower():
    channel_name_(""),
    channel_name_attribute_index_(-1),
    channel_order_(0),
    channel_order_attribute_index_(-1),
    search_distance_(5.0),
    neighbor_count_(5),
    search_by_distance_(true)
{
    GDALAllRegister();
    dem_cloud_.reset(new GC());
    veg_cloud_.reset(new VC());
    dem_tree_.reset(new GT());
    veg_tree_.reset(new VT());
}

// Destructor - Release GDAL Databases
template <typename DEMType, typename VegType, typename ChannelType>
RiparianGrower<DEMType, VegType, ChannelType>::~RiparianGrower()
{
    GDALClose( flowlines_dataset_ );
}

// Read Channel Network from NHDPlus .shpfile format
template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::readChannelNetworkNHDPlus(std::string filename, std::string layer_name)
{
    std::cout << " Attempting to read channel network from .shpfile at " << filename << std::endl;
    // Load Dataset from .shpfile
    flowlines_dataset_ = (GDALDataset*) GDALOpenEx(filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(flowlines_dataset_ == NULL)
    {
        printf("ERROR: GDAL Shapefile Open Failed.");
        exit(-1);
    }
    else 
        std::cout << "  Successfully loaded channel network from .shpfile" << std::endl;
    // Get Flowlines Layer
    flowlines_layer_ = flowlines_dataset_->GetLayerByName(layer_name.c_str());
    // Count and print number of segments in the network
    OGRFeature *flowline_segment_counter;
    int num_segments = 0;
    for( auto& flowline_segment_counter: flowlines_layer_ )
    {
        num_segments++;
    }
    flowlines_layer_->ResetReading();
    std::cout << "  Input channel network contains " << num_segments << " total segments." << std::endl;

    // Get first stream segment (just to have attribute names)
    first_segment_ = flowlines_layer_->GetNextFeature();
    flowlines_layer_->ResetReading();
    // Count and print number of attributes 
    int num_attributes = 0;
    for( auto&& flowline_field_: *first_segment_ )
    { 
        num_attributes++;
    }   
    std::cout << "  Input channel network contains " << num_attributes << " total attributes." << std::endl;
 
}

// Reads a Digital Elevation Model (ground height) from .PDC point cloud file (format from the Point Cloud Library)
// The recommended custom point type used (pcl::Point2DGround) contains:
//  - intensity
//  - curvature
//  - height_difference_avg (average height distance to k neightbors)
//  - norm_diff_avg (average angular normal direction deviation from k neightbors)
template <typename DEMType, typename VegType, typename ChannelType>
bool RiparianGrower<DEMType, VegType, ChannelType>::readDEMPCD(std::string filename)
{
    std::cout << "Reading an input DEM cloud from file " << filename << std::endl;
    if (pcl::io::loadPCDFile<DEMType> (filename, *dem_cloud_) == -1) 
    {
        std::cout << "\nCouldn't read file " << filename << std::endl;
        return false;
    }
    std::cout << "  Successfully read DEM cloud with size " << dem_cloud_->points.size() << std::endl;
    // Build KD Search Tree on cloud
    dem_tree_->setInputCloud(dem_cloud_);
    // Get maps between DEM cloud indices and point ids (from original cloud)
    dem_cloud_index_map_.clear();
    for(int i=0; i<dem_cloud_->points.size(); i++)
        dem_cloud_index_map_.push_back(dem_cloud_->points[i].index);
    return true;
}

// Reads a Vegetation Cloud .PDC point cloud file (format from the Point Cloud Library)
// The recommended custom point type used (pcl::PointVeg) contains:
//  - Height (over local DEM height)
//  - Intensity (of LiDAR return)
//  - Roughness (measure of local height variability)
template <typename DEMType, typename VegType, typename ChannelType>
bool RiparianGrower<DEMType, VegType, ChannelType>::readVegCloudPCD(std::string filename)
{
    std::cout << "Reading an input vegetation cloud from file " << filename << std::endl;
    if (pcl::io::loadPCDFile<VegType> (filename, *veg_cloud_) == -1) 
    {
        std::cout << "\nCouldn't read file " << filename << std::endl;
        return false;
    }
    std::cout << "  Successfully read vegetation cloud with size " << veg_cloud_->points.size() << std::endl;
    // Build KD Search Tree on cloud
    veg_tree_->setInputCloud(veg_cloud_);
    for(int i=0; i<veg_cloud_->points.size(); i++)
        veg_cloud_index_map_.push_back(veg_cloud_->points[i].index);
    return true;
}



// Get Ground TIN from DEM
template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::generateGroundTIN()
{
    TIN_.setInputCloud(dem_cloud_, dem_tree_);
    TIN_.generateTIN();
}


template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::filterByChannelName(std::string channel_name, std::string attribute_name)
{
    std::cout << "Adding request to filter by channel name " << channel_name << std::endl;
    channel_name_ = channel_name;
    
    int attribute_index = 0;
    for( auto&& flowline_field_: *first_segment_ )
    { 
        // NOTE str::compare() returns 0 if the strings are equal. Disgusting, I know
        if(attribute_name.compare(flowline_field_.GetName()) == 0)
        {
            channel_name_attribute_index_ = attribute_index;
            break;
        }
        attribute_index++;
    }   
    if(channel_name_attribute_index_ == -1)
        std::cout << "  WARNING: requested attribute field name (" << attribute_name << ") was not found, so no channel name filter will be applied." << std::endl;
}

template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::filterByChannelOrder(int channel_order, std::string attribute_name)
{
    std::cout << "Adding request to filter to disclude channels of order lower than " << channel_order << std::endl;
    channel_order_ = channel_order;   

    int attribute_index = 0;
    for( auto&& flowline_field_: *first_segment_ )
    { 
        // NOTE str::compare() returns 0 if the strings are equal. Disgusting, I know
        if(attribute_name.compare(flowline_field_.GetName()) == 0)
        {
            channel_order_attribute_index_ = attribute_index;
            break;
        }
        attribute_index++;
    }   
    if(channel_order_attribute_index_ == -1)
        std::cout << "  WARNING: requested attribute field name (" << attribute_name << ") was not found, so no channel order filter will be applied." << std::endl;
}

// This updates the region growing search distance, and compells the growing to function based on a distance search on each pass
template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::setNeighborSearchDistance(float search_distance)
{
    search_distance_ = search_distance;
    search_by_distance_ = true;
}
// This updates the region growing neighbor search count, and compells the growing to find a fixed number of neighbors on each pass
template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::setNeighborSearchCount(int neighbor_count)
{
    neighbor_count_ = neighbor_count;
    search_by_distance_ = false;
}

template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::extractVegetationStatistics(std::string output_filename)
{
    std::cout << "Beginning to extract vegetation statistics around stream flowlines." << std::endl;
    OGRGeometry *flowline_geometry;
    flowline_geometry = first_segment_->GetGeometryRef();

    // Check that input .shpfile geometry is valid and of the LineString type
    if( flowline_geometry == NULL
        || wkbFlatten(flowline_geometry->getGeometryType()) != wkbLineString )
    {
        std::cout << "  Something is wrong with the geometry of the input file. Returning without performing analysis." << std::endl;
        return;
    }

    CC channel_network_cloud;

    flowlines_layer_->ResetReading();       
    OGRFeature *flowline_segment; 
    int num_segments = 0;
    float avg_points = 0;
    int total_points = 0;
    int largest_segment = 0;
    int smallest_segment = 1e8;
    float mean_channel_height = 0;
    for( auto& input_segment: flowlines_layer_ )
    {
        // Skip features which don't match user-specified attribute filters
        if(channel_name_attribute_index_ > -1)
            if(channel_name_.compare(((*input_segment)[channel_name_attribute_index_]).GetAsString()) != 0)
                continue;
        if(channel_order_attribute_index_ > -1)
            if(((*input_segment)[channel_order_attribute_index_]).GetInteger() < channel_order_)
                continue;

        num_segments++;
        int current_points = 0;
        OGRPoint ptTemp;

        // Get Geometry
        OGRGeometry *input_geometry;
        input_geometry = input_segment->GetGeometryRef(); 
        // Reformat Geometry and get Point Count 
        OGRLineString *input_geometry_string = ( OGRLineString * )input_geometry;
        int NumberOfVertices = input_geometry_string ->getNumPoints();
        // Iterate over all points in segment, generate new data structures in memory
        LineSegment channel_segment;
        for ( int k = 0; k < NumberOfVertices; k++ )
        {
            input_geometry_string->getPoint(k,&ptTemp);
            DEMType pt_grd;
            pt_grd.x = ptTemp.getX();
            pt_grd.y = ptTemp.getY();
            pt_grd.z = 0;
            channel_segment.geometry_gc.points.push_back(pt_grd);
            ChannelType pt_channel;
            pt_channel.x = ptTemp.getX();
            pt_channel.y = ptTemp.getY();
            pt_channel.z = 0;
            pt_channel.z = pt_channel.z = 0-getChannelElevation(pt_grd);
            channel_segment.geometry_cc.points.push_back(pt_channel); 
            
            
            if(pt_channel.z < 10e5 && pt_channel.z > -100)
                mean_channel_height += pt_channel.z;
            current_points++;
        }
        // Run some geometric analysis on segment
        getSegmentDirection(channel_segment);
        for(int i=0; i<channel_segment.geometry_cc.points.size(); i++)
        {
            std::vector<int> index_list;
            regionGrowingVeg(channel_segment.geometry_cc.points[i], index_list);
            channel_network_cloud.push_back(channel_segment.geometry_cc.points[i]);
        }

        total_points += current_points;
        if(largest_segment < current_points)
            largest_segment = current_points;
        if(smallest_segment > current_points)
            smallest_segment = current_points;
    }    
    mean_channel_height = mean_channel_height/float(total_points);
    avg_points = float(total_points) / float(num_segments);
    std::cout << "  Finished brief run through of segments. In " << num_segments << " segments there were a total of " << total_points << " points for an average of " << avg_points << " points/segment." << std::endl;
    std::cout << "  Smallest segment has " << smallest_segment << " points. Largest segment has " << largest_segment << " points." << std::endl;
    std::cout << "  Average point elevation was " << mean_channel_height << "ft." << std::endl;

    pcl::PCDWriter writer;
    writer.write<ChannelType>("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/rip_height.pcd", channel_network_cloud, true);
}



// Get local channel height based on DEM
template <typename DEMType, typename VegType, typename ChannelType>
float RiparianGrower<DEMType, VegType, ChannelType>::getChannelElevation(DEMType point)
{
    return TIN_.getPointHeight(dem_cloud_, point);
}



template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::getSegmentDirection(LineSegment &segment)
{
    // Iterating over points in the segment
    for(int i=0; i<segment.geometry_cc.points.size(); i++)
    {
        Eigen::Vector2f point_1;
        Eigen::Vector2f point_2;
        Eigen::Vector2f flow;
        // For the first point, compare it to the next point
        if(i == 0)
        {
            point_1 << segment.geometry_cc.points[i].x, segment.geometry_cc.points[i].y;
            point_2 << segment.geometry_cc.points[i+1].x, segment.geometry_cc.points[i+1].y;
            segment.geometry_cc.points[i].fore_swath_dist = (point_2 - point_1).norm();
            segment.geometry_cc.points[i].rear_swath_dist = 0;
        }
        // For the last point, compare it to the previous point
        else if(i == segment.geometry_cc.points.size()-1)
        {
            point_1 << segment.geometry_cc.points[i-1].x, segment.geometry_cc.points[i-1].y;
            point_2 << segment.geometry_cc.points[i].x, segment.geometry_cc.points[i].y;
            segment.geometry_cc.points[i].fore_swath_dist = 0;
            segment.geometry_cc.points[i].rear_swath_dist = (point_2 - point_1).norm();
        }
        // For other points, compare the previous and next point
        else
        {
            point_1 << segment.geometry_cc.points[i-1].x, segment.geometry_cc.points[i-1].y;
            point_2 << segment.geometry_cc.points[i+1].x, segment.geometry_cc.points[i+1].y;
            Eigen::Vector2f point_mid;
            point_mid << segment.geometry_cc.points[i].x, segment.geometry_cc.points[i].y;
            segment.geometry_cc.points[i].fore_swath_dist = (point_2 - point_mid).norm();
            segment.geometry_cc.points[i].rear_swath_dist = (point_mid - point_1).norm();
        }
        // Flow as an XY vector
        flow = point_2 - point_1;
        // Flow as the cartesian angle of this vector
        segment.geometry_cc.points[i].flow_direction = atan2(flow[1], flow[0]);
        segment.geometry_cc.points[i].normal_direction = atan2(flow[1], flow[0]) + M_PI/2;
        segment.geometry_cc.points[i].normal_direction = std::fmod(segment.geometry_cc.points[i].flow_direction, float(M_PI));
    }
}


template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::getPerpendicularSwath(LineSegment &segment)
{
    for(int i=0; segment.geometry_cc.points.size(); i++)
    {
        
    }
}

template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::regionGrowingVeg(ChannelType &point, std::vector<int> &index_list)
{
    VegType point_veg;
    point_veg.x = point.x;
    point_veg.y = point.y;
    point_veg.z = point.z;
    std::cout << "Starting a new search for veg neighbors of point: " << point.x << " " << point.y << std::setprecision(8) << std::endl;

    // Get the nearest seed points proceed from here
    std::vector<int> nearest_indices;
    std::vector<float> dist_squareds;
    veg_tree_->nearestKSearch(point_veg, 1, nearest_indices, dist_squareds);
    std::cout << nearest_indices.size() << " " << nearest_indices[0] << std::endl;
    regionGrowingRecursorVeg(point, veg_cloud_->points[nearest_indices[0]], nearest_indices[0], index_list);
    std::cout << "made it out of there... " << index_list.size() << std::endl;

    // Once you get back to the top level, do some kind of stats over points? 
    std::vector<float> veg_height;
    for(int i=0; i<index_list.size(); i++)
    {
        veg_height.push_back(veg_cloud_->points[index_list[i]].height);
    }
    std::sort(veg_height.begin(), veg_height.end());
    
    if(veg_height.size() == 0)
    {
        point.vegetation_height_100th = 0;
        point.vegetation_height_75th = 0;
        point.vegetation_height_50th = 0;
        point.vegetation_density = 0;
    }
    else
    {
        point.vegetation_height_100th = veg_height[veg_height.size()-1];
        point.vegetation_height_75th = veg_height[veg_height.size()/4*3];
        point.vegetation_height_50th = veg_height[veg_height.size()/2];
        point.vegetation_density = veg_height.size();
    }

    std::cout << "  Finished finding subpoints for the above. Stats: " << point.vegetation_height_100th << " " << point.vegetation_height_75th << " " << point.vegetation_height_50th << " " << point.vegetation_density << std::endl;
}

template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::regionGrowingRecursorVeg(ChannelType &point, VegType new_point, int new_ind, std::vector<int> &index_list)
{
    // Check whether new point is already in the region
    bool already_included = false;
    for(int i=0; i<index_list.size(); i++)
        if(index_list[i] == new_ind)
        {
            already_included = true;
            return;
        }
    // Check if new point is a good candidate to be added to region
    if(growingConditionVeg(point, new_point))
    {
        // Add new point to list
        index_list.push_back(new_ind);
        // Get New Candidate Points
        std::vector<int> nearest_indices;
        std::vector<float> dist_squareds;
        if(search_by_distance_)
            veg_tree_->radiusSearch(new_point, search_distance_, nearest_indices, dist_squareds);
        else
            veg_tree_->nearestKSearch(new_point, neighbor_count_, nearest_indices, dist_squareds);
        for(int i=0; i<nearest_indices.size(); i++)
            regionGrowingRecursorVeg(point, veg_cloud_->points[nearest_indices[i]], nearest_indices[i], index_list);
    }

    return;
}

template <typename DEMType, typename VegType, typename ChannelType>
bool RiparianGrower<DEMType, VegType, ChannelType>::growingConditionVeg(ChannelType point, VegType point_new)
{
    // For now, threshold not to include points further than 50 ft
    if(point2DDistance(point, point_new) > 50)
        return false;
    // Check return number? 
    // Check height?
    // Check local structure of crown?
    // Check ground height over source stream point
    // Check location relative to swath 
    float distance_along_channel = pointDistanceAlongVector(point, point_new, angleFromAzimuth(point.flow_direction));
    if(distance_along_channel > point.fore_swath_dist*1.5 || distance_along_channel < -point.rear_swath_dist*1.5)
        return false;
    return true;
}


template <typename DEMType, typename VegType, typename ChannelType>
void RiparianGrower<DEMType, VegType, ChannelType>::getSwathPointIndices(LineSegment &segment)
{

}

// Get 2D (XY) distance between two points of arbitrary PCL type
template <typename DEMType, typename VegType, typename ChannelType>
template <typename FirstType, typename SecondType>
float RiparianGrower<DEMType, VegType, ChannelType>::point2DDistance(FirstType point_1, SecondType point_2)
{
    return sqrt(pow(point_1.x-point_2.x,2) + pow(point_1.y-point_2.y,2));
}

// Get 3D (XYZ) distance between two points of arbitrary PCL type
template <typename DEMType, typename VegType, typename ChannelType>
template <typename FirstType, typename SecondType>
float RiparianGrower<DEMType, VegType, ChannelType>::point3DDistance(FirstType point_1, SecondType point_2)
{
    return sqrt(pow(point_1.x-point_2.x,2) + pow(point_1.y-point_2.y,2) + pow(point_1.z-point_2.z,2));
}

// Get distance between two points along a particular 3D vector
template <typename DEMType, typename VegType, typename ChannelType>
template <typename FirstType, typename SecondType>
float RiparianGrower<DEMType, VegType, ChannelType>::pointDistanceAlongVector(FirstType point_1, SecondType point_2, Eigen::Vector3f vec)
{ 
    Eigen::Vector3f point_1_eig;
    Eigen::Vector3f point_2_eig;
    point_1_eig << point_1.x, point_1.y, point_1.z;
    point_2_eig << point_2.x, point_2.y, point_2.z;

    Eigen::Vector3f distance = point_2_eig - point_1_eig;

    return distance.dot(vec)/vec.norm();
}

// Get Eigen::Vector3f unit vector in XY plane based on cartesian angle input
template <typename DEMType, typename VegType, typename ChannelType>
Eigen::Vector3f RiparianGrower<DEMType, VegType, ChannelType>::angleFromAzimuth(float angle)
{ 
    Eigen::Vector3f vec;
    vec << cos(angle), sin(angle), 0;
    return vec;
}