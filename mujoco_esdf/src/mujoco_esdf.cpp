#include <mujoco_esdf/mujoco_esdf.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
  double boxDistance(const Eigen::Vector3d& point, const mjsGeom& geometry)
  {
    const Eigen::Vector3d halfSize(geometry.size[0], geometry.size[1], geometry.size[2]);
    const Eigen::Vector3d delta = point.cwiseAbs() - halfSize;
    const Eigen::Vector3d outside = delta.cwiseMax(Eigen::Vector3d::Zero());
    return outside.norm() + std::min(std::max(delta.x(), std::max(delta.y(), delta.z())), 0.0);
  }

  double cylinderDistance(const Eigen::Vector3d& point, const mjsGeom& geometry)
  {
    const Eigen::Vector2d delta(
      point.head<2>().norm() - geometry.size[0], std::abs(point.z()) - geometry.size[1]);
    const Eigen::Vector2d outside = delta.cwiseMax(Eigen::Vector2d::Zero());
    return outside.norm() + std::min(std::max(delta.x(), delta.y()), 0.0);
  }

  double geometryDistance(const Eigen::Vector3d& point, const mjsGeom& geometry)
  {
    const Eigen::Vector3d position(geometry.pos[0], geometry.pos[1], geometry.pos[2]);
    const Eigen::Quaterniond quaternion(
      geometry.quat[0], geometry.quat[1], geometry.quat[2], geometry.quat[3]);
    const Eigen::Vector3d localPoint = quaternion.normalized().conjugate() * (point - position);
    if(geometry.type == mjGEOM_BOX) {
      return boxDistance(localPoint, geometry);
    }
    if(geometry.type == mjGEOM_CYLINDER) {
      return cylinderDistance(localPoint, geometry);
    }
    throw std::invalid_argument("mujoco_esdf supports box and cylinder geometry only");
  }
}

namespace mujoco_esdf {
  std::shared_ptr<voxblox::EsdfMap> createEsdfMap(
    const std::vector<const mjsGeom*>& geometries,
    const Eigen::Vector3d& minimum,
    const Eigen::Vector3d& maximum,
    double voxelSize,
    std::size_t voxelsPerSide)
  {
    if(geometries.empty() || voxelSize <= 0.0 || voxelsPerSide == 0 ||
       (maximum.array() <= minimum.array()).any()) {
      throw std::invalid_argument("invalid MuJoCo ESDF input");
    }
    voxblox::EsdfMap::Config config;
    config.esdf_voxel_size = static_cast<voxblox::FloatingPoint>(voxelSize);
    config.esdf_voxels_per_side = voxelsPerSide;
    std::shared_ptr<voxblox::EsdfMap> map = std::make_shared<voxblox::EsdfMap>(config);
    voxblox::Layer<voxblox::EsdfVoxel>* layer = map->getEsdfLayerPtr();

    // ESDF values belong to voxel centres, not arbitrary points inside each
    // voxel. Align the sampling grid even when minimum is not a grid multiple.
    const double cellSize = layer->voxel_size();
    const Eigen::Array3i first = (minimum/cellSize).array().floor().cast<int>();
    const Eigen::Array3i count = (maximum/cellSize).array().ceil().cast<int>()-first;
    for(int x = 0; x < count.x(); ++x) {
      for(int y = 0; y < count.y(); ++y) {
        for(int z = 0; z < count.z(); ++z) {
          const Eigen::Vector3d point = cellSize*(first.cast<double>()+
            Eigen::Array3d(double(x)+0.5,double(y)+0.5,double(z)+0.5)).matrix();
          const voxblox::Point voxelPoint = point.cast<voxblox::FloatingPoint>();
          layer->allocateBlockPtrByCoordinates(voxelPoint);
          voxblox::EsdfVoxel* voxel = layer->getVoxelPtrByCoordinates(voxelPoint);
          double distance = std::numeric_limits<double>::infinity();
          for(std::size_t geometryIndex = 0; geometryIndex < geometries.size(); ++geometryIndex) {
            if(geometries[geometryIndex] == nullptr) {
              throw std::invalid_argument("null MuJoCo geometry");
            }
            distance = std::min(distance, geometryDistance(point, *geometries[geometryIndex]));
          }
          voxel->distance = static_cast<voxblox::FloatingPoint>(distance);
          voxel->observed = true;
        }
      }
    }
    return map;
  }
}
