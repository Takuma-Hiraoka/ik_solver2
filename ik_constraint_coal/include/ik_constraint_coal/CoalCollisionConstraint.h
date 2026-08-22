#pragma once

#include <ik_constraint/CollisionConstraint.h>

#include <coal/distance.h>

#include <pinocchio/multibody/geometry.hpp>

#include <memory>
#include <vector>

namespace ik_constraint_coal{
  class CoalCollisionConstraint : public ik_constraint::CollisionConstraint
  {
  public:
    CoalCollisionConstraint(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      const std::shared_ptr<const pinocchio::GeometryModel>& AGeometryModel,
      pinocchio::JointIndex AJointId,
      const std::shared_ptr<const pinocchio::Model>& BModel,
      const std::shared_ptr<const pinocchio::GeometryModel>& BGeometryModel,
      pinocchio::JointIndex BJointId);
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;

    const std::shared_ptr<const pinocchio::GeometryModel>& getAGeometryModel() const {
      return this->AGeometryModel_;
    }
    void setAGeometryModel(
      const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel);
    const std::shared_ptr<const pinocchio::GeometryModel>& getBGeometryModel() const {
      return this->BGeometryModel_;
    }
    void setBGeometryModel(
      const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel);

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

    std::shared_ptr<const pinocchio::GeometryModel> AGeometryModel_;
    std::shared_ptr<const pinocchio::GeometryModel> BGeometryModel_;

  private:
    void initializeDistanceQueries();
    void invalidateDistanceQueries();
    const pinocchio::Data& getData(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    pinocchio::SE3 getGeometryWorldPlacement(
      const pinocchio::Data& data,
      const pinocchio::GeometryObject& geometryObject) const;
    coal::Transform3s convertTransform(const pinocchio::SE3& transform) const;

    const pinocchio::Data* AData_ = nullptr;
    const pinocchio::Data* BData_ = nullptr;
    std::vector<const pinocchio::GeometryObject*> AGeometryObjects_;
    std::vector<const pinocchio::GeometryObject*> BGeometryObjects_;
    std::vector<size_t> queryAGeometryIndices_;
    std::vector<size_t> queryBGeometryIndices_;
    std::vector<std::shared_ptr<coal::ComputeDistance> > distanceQueries_;
    std::shared_ptr<const pinocchio::Model> initializedAModel_ = nullptr;
    std::shared_ptr<const pinocchio::Model> initializedBModel_ = nullptr;
    pinocchio::JointIndex initializedAJointId_ = 0;
    pinocchio::JointIndex initializedBJointId_ = 0;
    bool distanceQueriesInitialized_ = false;
  };
}
