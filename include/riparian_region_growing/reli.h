
#ifndef RELI_
#define RELI_
#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_representation.h>

namespace pcl{

struct RELI
  {
    PCL_ADD_POINT4D                     // Macro quad-word XYZ
    float flow_direction;
    float normal_direction;
    float fore_swath_dist;
    float rear_swath_dist;
    float vegetation_height_100th;
    float vegetation_height_75th;
    float vegetation_height_50th;
    float vegetation_density;
    float histogram[30];                  /* Data contained by index:
                                            00 - vegetation height (5th percentile)
                                            01 - vegetation height (25th percentile)
                                            02 - vegetation height (50th percentile)
                                            03 - vegetation height (75th percentile)
                                            04 - vegetation height (95th percentile)
                                            05 - vegetation height (mean)
                                            06 - vegetation height (standard deviation)
                                            07 - vegetation density (fraction of overall returns)
                                            08 - ground elevation (mean)
                                            09 - ground elevation (min)
                                            10 - ground elevation (max)
                                            11 - ground elevation (standard deviation)
                                            12 - building height (mean)
                                            13 - building density (fraciton of overall returns)
                                            14 - ground slope (mean)
                                            15 - 
    */  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // Ensure proper alignment
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::RELI,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, flow_direction, flow_direction)
                                  (float, normal_direction, normal_direction)
                                  (float, fore_swath_dist, fore_swath_dist)
                                  (float, rear_swath_dist, rear_swath_dist)
                                  (float, vegetation_height_100th, vegetation_height_100th)
                                  (float, vegetation_height_75th, vegetation_height_75th)
                                  (float, vegetation_height_50th, vegetation_height_50th)
                                  (float, vegetation_density, vegetation_density)
)

namespace pcl {
template <>
class DefaultPointRepresentation<RELI> : public PointRepresentation<RELI>
{
public:
  DefaultPointRepresentation ()
  {
    nr_dimensions_ = 3;
  }
  
  virtual void
  copyToFloatArray (const RELI &p, float * out) const
  {
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
  }
};
}
#endif // RELI_
