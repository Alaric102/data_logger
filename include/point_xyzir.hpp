#include <pcl/pcl_macros.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pcl {
struct PointXYZIR {
  PCL_ADD_POINT4D;
  float intensity;
  std::uint16_t ring;
  inline PointXYZIR(const float _x, const float _y, const float _z, const float _intensity, const std::uint16_t _ring)
    : x(_x), y(_y), z(_z), intensity(_intensity), ring(_ring){};

  inline PointXYZIR(const float _x, const float _y, const float _z) : PointXYZIR(_x, _y, _z, 0.f, 0){};

  inline PointXYZIR() : PointXYZIR(0.f, 0.f, 0.f, 0.f, 0){};

  ~PointXYZIR() = default;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t,
                                                                                                       ring, ring))
