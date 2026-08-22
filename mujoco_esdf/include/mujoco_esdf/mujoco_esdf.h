#pragma once

#include <mujoco/mujoco.h>
#include <voxblox/core/esdf_map.h>

#include <Eigen/Core>

#include <memory>
#include <vector>

namespace mujoco_esdf {
  std::shared_ptr<voxblox::EsdfMap> createEsdfMap(
    const std::vector<const mjsGeom*>& geometries,
    const Eigen::Vector3d& minimum,
    const Eigen::Vector3d& maximum,
    double voxelSize,
    std::size_t voxelsPerSide = 16);
}
