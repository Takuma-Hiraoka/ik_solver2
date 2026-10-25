#pragma once

#include <ik_constraint/CollisionConstraint.h>

#include <pinocchio/multibody/geometry.hpp>
#include <voxblox/core/esdf_map.h>

#include <Eigen/Geometry>

#include <memory>
#include <vector>

namespace ik_constraint_esdf {
  class EsdfCollisionConstraint : public ik_constraint::CollisionConstraint
  {
  public:
    EsdfCollisionConstraint(
      const std::shared_ptr<const pinocchio::Model>& model,
      const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel,
      pinocchio::JointIndex jointId);
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;

    const std::shared_ptr<const pinocchio::GeometryModel>& getGeometryModel() const {
      return this->geometryModel_;
    }
    void setGeometryModel(
      const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel);
    const std::shared_ptr<voxblox::EsdfMap>& getField() const {
      return this->field_;
    }
    void setField(const std::shared_ptr<voxblox::EsdfMap>& field) {
      this->field_ = field;
    }
    const Eigen::Isometry3d& getFieldOrigin() const { return this->fieldOrigin_; }
    void setFieldOrigin(const Eigen::Isometry3d& fieldOrigin) {
      this->fieldOrigin_ = fieldOrigin;
    }
    const double& getResolution() const { return this->resolution_; }
    void setResolution(const double& resolution);
    const double& getMinDistance() const { return this->minDistance_; }
    void setMinDistance(const double& minDistance) {
      this->minDistance_ = minDistance;
    }
    const double& getMaxDistance() const { return this->maxDistance_; }
    void setMaxDistance(const double& maxDistance) {
      this->maxDistance_ = maxDistance;
    }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;

  protected:
    bool computeDistance(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      pinocchio::JointIndex AJointId,
      const std::shared_ptr<const pinocchio::Model>& BModel,
      pinocchio::JointIndex BJointId,
      double& distance,
      Eigen::Vector3d& direction,
      Eigen::Vector3d& ALocalPosition,
      Eigen::Vector3d& BLocalPosition) override;

  private:
    void updateSurfacePoints();
    void invalidateSurfacePoints();
    void appendGeometrySurfacePoints(
      const pinocchio::GeometryObject& geometryObject,
      std::vector<Eigen::Vector3d>& points) const;

    std::shared_ptr<const pinocchio::GeometryModel> geometryModel_;
    std::shared_ptr<voxblox::EsdfMap> field_ = nullptr;
    Eigen::Isometry3d fieldOrigin_ = Eigen::Isometry3d::Identity();
    double resolution_ = 0.01;
    double minDistance_ = -0.02;
    double maxDistance_ = 0.5;
    const pinocchio::Data* data_ = nullptr;
    std::vector<Eigen::Vector3d> surfacePoints_;
    std::shared_ptr<const pinocchio::Model> initializedModel_ = nullptr;
    std::shared_ptr<const pinocchio::GeometryModel> initializedGeometryModel_ = nullptr;
    pinocchio::JointIndex initializedJointId_ = 0;
    double initializedResolution_ = 0.0;
    Eigen::Vector3d previousALocalPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousBWorldPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousDirection_ = Eigen::Vector3d::UnitX();
    bool surfacePointsInitialized_ = false;
    bool hasPreviousValidGradient_ = false;
  };
}
