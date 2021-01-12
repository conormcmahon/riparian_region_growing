
#include "riparian_region_growing/riparian_grower.h"

// Constructor 
template <typename DEMType, typename VegType>
RiparianGrower<DEMType, VegType>::RiparianGrower():
    channel_name_(""),
    channel_name_attribute_index_(0),
    channel_order_(0),
    channel_order_attribute_index_(0)
{
    GDALAllRegister();
    dem_cloud_.reset(new GC());
    veg_cloud_.reset(new VC());
    dem_tree_.reset(new GT());
    veg_tree_.reset(new VT());
}

// Destructor - Release GDAL Databases
template <typename DEMType, typename VegType>
RiparianGrower<DEMType, VegType>::~RiparianGrower()
{
    GDALClose( flowlines_dataset_ );
}

// Read Channel Network from NHDPlus .shpfile format
template <typename DEMType, typename VegType>
void RiparianGrower<DEMType, VegType>::readChannelNetworkNHDPlus(std::string filename, std::string layer_name)
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
template <typename DEMType, typename VegType>
bool RiparianGrower<DEMType, VegType>::readDEMPCD(std::string filename)
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
    return true;
}

// Reads a Vegetation Cloud .PDC point cloud file (format from the Point Cloud Library)
// The recommended custom point type used (pcl::PointVeg) contains:
//  - Height (over local DEM height)
//  - Intensity (of LiDAR return)
//  - Roughness (measure of local height variability)
template <typename DEMType, typename VegType>
bool RiparianGrower<DEMType, VegType>::readVegCloudPCD(std::string filename)
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
    return true;
}



// Get Ground TIN from DEM
template <typename DEMType, typename VegType>
void RiparianGrower<DEMType, VegType>::generateGroundTIN()
{
    TIN_.setInputCloud(dem_cloud_, dem_tree_);
    TIN_.generateTIN();
}


template <typename DEMType, typename VegType>
void RiparianGrower<DEMType, VegType>::filterByChannelName(std::string channel_name, std::string attribute_name)
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

template <typename DEMType, typename VegType>
void RiparianGrower<DEMType, VegType>::filterByChannelOrder(int channel_order, std::string attribute_name)
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



template <typename DEMType, typename VegType>
void RiparianGrower<DEMType, VegType>::extractVegetationStatistics(std::string output_filename)
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
    VC test_cloud;

    flowlines_layer_->ResetReading();       
    OGRFeature *flowline_segment; 
    int num_segments = 0;
    float avg_points = 0;
    int total_points = 0;
    int largest_segment = 0;
    int smallest_segment = 1e8;
    float mean_channel_height = 0;
    for( auto& flowline_segment: flowlines_layer_ )
    {
        num_segments++;
        int current_points = 0;
        //OGRFeature *poFeature;
        OGRPoint ptTemp;

        //poFeature = flowlines_layer_->GetNextFeature();
        OGRGeometry *poGeometry;
        poGeometry = flowline_segment->GetGeometryRef(); //poFeature->GetGeometryRef();
        
        OGRLineString *poLineString = ( OGRLineString * )poGeometry;
        //Polyline.LinesOfFeature.resize(1);
        int NumberOfVertices = poLineString ->getNumPoints();
        LineSegment pcl_segment;
        //Polyline.LinesOfFeature.at(0).LineString.resize(NumberOfVertices);
        for ( int k = 0; k < NumberOfVertices; k++ )
        {
            poLineString->getPoint(k,&ptTemp);
            DEMType pt_grd;
            pt_grd.x = ptTemp.getX();
            pt_grd.y = ptTemp.getY();
            pt_grd.z = 0;
            pcl_segment.geometry_gc.points.push_back(pt_grd);
            VegType pt_veg;
            pt_veg.x = ptTemp.getX();
            pt_veg.y = ptTemp.getY();
            pt_veg.z = 0;
            pt_veg.z = pt_veg.z = 0-getChannelElevation(pt_grd);
            pcl_segment.geometry_vc.points.push_back(pt_veg); 
            test_cloud.points.push_back(pt_veg);
            
            if(pt_veg.z < 10e5 && pt_veg.z > -100)
                mean_channel_height += pt_veg.z;
            //Polyline.LinesOfFeature.at(0).LineString.at(k) = pt;
            current_points++;
        }
        //LineLayer.push_back(Polyline);

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
    writer.write<VegType>("/mnt/d/serdp/data/pendleton/LiDAR/hydrology/raw_point_clouds/output/rip_height.pcd", test_cloud, true);
}



// Get local channel height based on DEM
template <typename DEMType, typename VegType>
float RiparianGrower<DEMType, VegType>::getChannelElevation(DEMType point)
{
    return TIN_.getPointHeight(dem_cloud_, point);
}


// *** Read Channel Network ***
// Takes input flowline information (e.g. from NHDPlus) from a shapefile
void readChannelNetwork(std::string filename, float raster_object)
{
    // Register GDAL Drivers
    GDALAllRegister();

    GDALDataset *flowlines_dataset;
    flowlines_dataset = (GDALDataset*) GDALOpenEx(filename.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
    if(flowlines_dataset == NULL)
    {
        printf("ERROR: GDAL Shapefile Open Failed.");
        exit(1);
    }
    
    OGRLayer *flowlines;
    flowlines = flowlines_dataset->GetLayerByName("flowlines_vaa_ca");

    OGRFeature *flowline_segment_counter;
    int num_segments = 0;
    for( auto& flowline_segment_counter: flowlines )
    {
        num_segments++;
    }
    std::cout << "total number of segments: " << num_segments << std::endl;

    flowlines->ResetReading(); 
    OGRFeature *first_segment;
    first_segment = flowlines->GetNextFeature();

    OGRField *flowline_field;
    for( auto&& flowline_field: *first_segment )
    { 
        std::cout << flowline_field.GetName() << " " << flowline_field.GetAsString() << std::endl;
    }

    OGREnvelope envelope;
    flowlines->GetExtent(&envelope, true);

    flowlines->ResetReading();

    OGRGeometry *flowline_geometry;
    flowline_geometry = first_segment->GetGeometryRef();


    OGRFeature *flowline_segment;
    if( flowline_geometry != NULL
        && wkbFlatten(flowline_geometry->getGeometryType()) == wkbLineString )
    {
        std::cout << "Got in here!" << std::endl;
        OGRFeature *poFeature;
        OGRPoint ptTemp;

        poFeature = flowlines ->GetNextFeature();
        OGRGeometry *poGeometry;
        poGeometry = poFeature ->GetGeometryRef();
        if ( poGeometry != NULL && wkbFlatten ( poGeometry ->getGeometryType() ) == wkbLineString  )
        {
            OGRLineString *poLineString = ( OGRLineString * )poGeometry;
            //Polyline.LinesOfFeature.resize(1);
            int NumberOfVertices = poLineString ->getNumPoints();
            //Polyline.LinesOfFeature.at(0).LineString.resize(NumberOfVertices);
            for ( int k = 0; k < NumberOfVertices; k++ )
            {
                poLineString ->getPoint(k,&ptTemp);
                pcl::PointXYZ pt;
                pt.x = ptTemp.getX();
                pt.y = ptTemp.getY();
                std::cout << pt.x << " " << pt.y << std::endl;
                //Polyline.LinesOfFeature.at(0).LineString.at(k) = pt;
            }
            //LineLayer.push_back(Polyline);
        }
        else if ( poGeometry != NULL && wkbFlatten ( poGeometry ->getGeometryType() ) == wkbMultiLineString )
        {
            OGRMultiLineString *poMultiLineString = ( OGRMultiLineString * )poGeometry;
            int NumberOfGeometries = poMultiLineString ->getNumGeometries();
            //Polyline.LinesOfFeature.resize(NumberOfGeometries);
            for ( int j = 0; j < NumberOfGeometries; j++ )
            {
                OGRGeometry *poLineGeometry = poMultiLineString ->getGeometryRef(j);
                OGRLineString *poLineString = ( OGRLineString * )poLineGeometry;
                int NumberOfVertices = poLineString ->getNumPoints();
                //Polyline.LinesOfFeature.at(j).LineString.resize(NumberOfVertices);
                for ( int k = 0; k < NumberOfVertices; k++ )
                {
                    poLineString ->getPoint(k,&ptTemp);
                    pcl::PointXYZ pt;
                    pt.x = ptTemp.getX();
                    pt.y = ptTemp.getY();
                    std::cout << pt.x << " " << pt.y << " although, else " << std::endl;
                    //Polyline.LinesOfFeature.at(j).LineString.at(k) = pt;
                }
            }
            //LineLayer.push_back(Polyline);
        }
    }

    std::cout << "fish!" << std::endl;

    GDALClose( flowlines_dataset );
}