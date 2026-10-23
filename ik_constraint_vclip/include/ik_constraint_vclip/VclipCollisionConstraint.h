#pragma once

#include <ik_constraint/CollisionConstraint.h>

#include <pinocchio/multibody/geometry.hpp>

#include <memory>
#include <vector>

namespace Vclip {
  class Polyhedron;
}

namespace ik_constraint_vclip {
  class VclipCollisionConstraint : public ik_constraint::CollisionConstraint
  {
  public:
    VclipCollisionConstraint(
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

  private:
    struct VclipGeometry {
      const pinocchio::GeometryObject* geometryObject = nullptr;
      std::shared_ptr<Vclip::Polyhedron> model = nullptr;
    };

    // Shapes shared by constraints and their clones; treated as read-only.
    // Pointer types match the existing vclip_eigen API. The source
    // GeometryModel must not be modified while this cache is in use.
    struct VclipGeometryModel {
      std::shared_ptr<const pinocchio::GeometryModel> geometryModel;
      std::vector<std::shared_ptr<Vclip::Polyhedron> > models;
    };

    static std::shared_ptr<const VclipGeometryModel> getVclipGeometryModel(
      const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel);
    void initializeVclipModels();
    void invalidateVclipModels();
    static std::vector<Eigen::Vector3d> getGeometryVertices(
      const pinocchio::GeometryObject& geometryObject);
    const pinocchio::Data& getData(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    pinocchio::SE3 getGeometryWorldPlacement(
      const pinocchio::Data& data,
      const pinocchio::GeometryObject& geometryObject) const;

    std::shared_ptr<const pinocchio::GeometryModel> AGeometryModel_;
    std::shared_ptr<const pinocchio::GeometryModel> BGeometryModel_;
    std::shared_ptr<const VclipGeometryModel> AVclipGeometryModel_;
    std::shared_ptr<const VclipGeometryModel> BVclipGeometryModel_;
    const pinocchio::Data* AData_ = nullptr;
    const pinocchio::Data* BData_ = nullptr;
    std::vector<VclipGeometry> AVclipGeometries_;
    std::vector<VclipGeometry> BVclipGeometries_;
    std::shared_ptr<const pinocchio::Model> initializedAModel_ = nullptr;
    std::shared_ptr<const pinocchio::Model> initializedBModel_ = nullptr;
    std::shared_ptr<const pinocchio::GeometryModel> initializedAGeometryModel_ = nullptr;
    std::shared_ptr<const pinocchio::GeometryModel> initializedBGeometryModel_ = nullptr;
    pinocchio::JointIndex initializedAJointId_ = 0;
    pinocchio::JointIndex initializedBJointId_ = 0;
    Eigen::Vector3d previousALocalPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousBLocalPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousDirection_ = Eigen::Vector3d::UnitX();
    bool vclipModelsInitialized_ = false;
    bool hasPreviousSeparatedResult_ = false;
  };
}
