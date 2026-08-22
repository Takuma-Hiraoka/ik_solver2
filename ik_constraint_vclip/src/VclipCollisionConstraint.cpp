#include <ik_constraint_vclip/VclipCollisionConstraint.h>

#include <vclip_eigen/vclip_eigen.h>

#include <coal/BVH/BVH_model.h>
#include <coal/shape/geometric_shapes.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ik_constraint_vclip {
  std::shared_ptr<ik_constraint::IKConstraint> VclipCollisionConstraint::clone() const {
    std::shared_ptr<VclipCollisionConstraint> result =
      std::make_shared<VclipCollisionConstraint>(*this);
    result->AData_ = nullptr;
    result->BData_ = nullptr;
    result->invalidateVclipModels();
    return result;
  }

  VclipCollisionConstraint::VclipCollisionConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    const std::shared_ptr<const pinocchio::GeometryModel>& AGeometryModel,
    pinocchio::JointIndex AJointId,
    const std::shared_ptr<const pinocchio::Model>& BModel,
    const std::shared_ptr<const pinocchio::GeometryModel>& BGeometryModel,
    pinocchio::JointIndex BJointId)
    : CollisionConstraint(AModel, AJointId, BModel, BJointId),
      AGeometryModel_(AGeometryModel),
      BGeometryModel_(BGeometryModel) {
    if(!this->AGeometryModel_ || !this->BGeometryModel_) {
      throw std::invalid_argument("VclipCollisionConstraint geometry model is null");
    }
  }

  void VclipCollisionConstraint::setAGeometryModel(
    const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel) {
    if(!geometryModel) {
      throw std::invalid_argument("VclipCollisionConstraint A geometry model is null");
    }
    this->AGeometryModel_ = geometryModel;
    this->invalidateVclipModels();
  }

  void VclipCollisionConstraint::setBGeometryModel(
    const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel) {
    if(!geometryModel) {
      throw std::invalid_argument("VclipCollisionConstraint B geometry model is null");
    }
    this->BGeometryModel_ = geometryModel;
    this->invalidateVclipModels();
  }

  void VclipCollisionConstraint::invalidateVclipModels() {
    this->AVclipGeometries_.clear();
    this->BVclipGeometries_.clear();
    this->vclipModelsInitialized_ = false;
    this->hasPreviousSeparatedResult_ = false;
  }

  std::vector<Eigen::Vector3d> VclipCollisionConstraint::getGeometryVertices(
    const pinocchio::GeometryObject& geometryObject) const {
    if(!geometryObject.geometry) {
      throw std::invalid_argument("VclipCollisionConstraint geometry is null");
    }

    std::vector<Eigen::Vector3d> vertices;
    const coal::BVHModelBase* bvhModel =
      dynamic_cast<const coal::BVHModelBase*>(geometryObject.geometry.get());
    if(bvhModel && bvhModel->vertices) {
      vertices.reserve(bvhModel->num_vertices);
      for(size_t i=0; i<bvhModel->num_vertices; i++) {
        vertices.push_back((*bvhModel->vertices)[i]);
      }
      return vertices;
    }

    const coal::ConvexBase* convex =
      dynamic_cast<const coal::ConvexBase*>(geometryObject.geometry.get());
    if(convex && convex->points) {
      vertices.reserve(convex->num_points);
      for(size_t i=0; i<convex->num_points; i++) {
        vertices.push_back((*convex->points)[i]);
      }
      return vertices;
    }

    const coal::Box* box =
      dynamic_cast<const coal::Box*>(geometryObject.geometry.get());
    if(box) {
      vertices.reserve(8);
      for(int xSign=-1; xSign<=1; xSign+=2) {
        for(int ySign=-1; ySign<=1; ySign+=2) {
          for(int zSign=-1; zSign<=1; zSign+=2) {
            vertices.push_back(Eigen::Vector3d(
              xSign * box->halfSide.x(),
              ySign * box->halfSide.y(),
              zSign * box->halfSide.z()));
          }
        }
      }
      return vertices;
    }

    const size_t circleSegments = 16;
    const size_t sphereLatitudeSegments = 8;
    const double pi = 3.14159265358979323846;
    const coal::Sphere* sphere =
      dynamic_cast<const coal::Sphere*>(geometryObject.geometry.get());
    const coal::Ellipsoid* ellipsoid =
      dynamic_cast<const coal::Ellipsoid*>(geometryObject.geometry.get());
    if(sphere || ellipsoid) {
      const Eigen::Vector3d radii = sphere
        ? Eigen::Vector3d::Constant(sphere->radius)
        : ellipsoid->radii;
      vertices.reserve(2 + (sphereLatitudeSegments - 1) * circleSegments);
      vertices.push_back(Eigen::Vector3d(0.0, 0.0, radii.z()));
      vertices.push_back(Eigen::Vector3d(0.0, 0.0, -radii.z()));
      for(size_t latitudeIndex=1;
          latitudeIndex<sphereLatitudeSegments;
          latitudeIndex++) {
        const double latitude = pi * static_cast<double>(latitudeIndex) /
          static_cast<double>(sphereLatitudeSegments);
        for(size_t longitudeIndex=0;
            longitudeIndex<circleSegments;
            longitudeIndex++) {
          const double longitude = 2.0 * pi * static_cast<double>(longitudeIndex) /
            static_cast<double>(circleSegments);
          vertices.push_back(Eigen::Vector3d(
            radii.x() * std::sin(latitude) * std::cos(longitude),
            radii.y() * std::sin(latitude) * std::sin(longitude),
            radii.z() * std::cos(latitude)));
        }
      }
      return vertices;
    }

    const coal::Cylinder* cylinder =
      dynamic_cast<const coal::Cylinder*>(geometryObject.geometry.get());
    if(cylinder) {
      vertices.reserve(2 * circleSegments);
      for(size_t i=0; i<circleSegments; i++) {
        const double angle = 2.0 * pi * static_cast<double>(i) /
          static_cast<double>(circleSegments);
        const double x = cylinder->radius * std::cos(angle);
        const double y = cylinder->radius * std::sin(angle);
        vertices.push_back(Eigen::Vector3d(x, y, cylinder->halfLength));
        vertices.push_back(Eigen::Vector3d(x, y, -cylinder->halfLength));
      }
      return vertices;
    }

    const coal::Cone* cone =
      dynamic_cast<const coal::Cone*>(geometryObject.geometry.get());
    if(cone) {
      vertices.reserve(circleSegments + 1);
      vertices.push_back(Eigen::Vector3d(0.0, 0.0, cone->halfLength));
      for(size_t i=0; i<circleSegments; i++) {
        const double angle = 2.0 * pi * static_cast<double>(i) /
          static_cast<double>(circleSegments);
        vertices.push_back(Eigen::Vector3d(
          cone->radius * std::cos(angle),
          cone->radius * std::sin(angle),
          -cone->halfLength));
      }
      return vertices;
    }

    const coal::Capsule* capsule =
      dynamic_cast<const coal::Capsule*>(geometryObject.geometry.get());
    if(capsule) {
      vertices.reserve(2 + (sphereLatitudeSegments - 1) * circleSegments);
      vertices.push_back(Eigen::Vector3d(
        0.0, 0.0, capsule->halfLength + capsule->radius));
      vertices.push_back(Eigen::Vector3d(
        0.0, 0.0, -capsule->halfLength - capsule->radius));
      for(size_t latitudeIndex=1;
          latitudeIndex<sphereLatitudeSegments;
          latitudeIndex++) {
        const double latitude = pi * static_cast<double>(latitudeIndex) /
          static_cast<double>(sphereLatitudeSegments);
        const double centerZ = latitude < pi * 0.5
          ? capsule->halfLength : -capsule->halfLength;
        for(size_t longitudeIndex=0;
            longitudeIndex<circleSegments;
            longitudeIndex++) {
          const double longitude = 2.0 * pi * static_cast<double>(longitudeIndex) /
            static_cast<double>(circleSegments);
          vertices.push_back(Eigen::Vector3d(
            capsule->radius * std::sin(latitude) * std::cos(longitude),
            capsule->radius * std::sin(latitude) * std::sin(longitude),
            centerZ + capsule->radius * std::cos(latitude)));
        }
      }
      return vertices;
    }

    throw std::invalid_argument(
      "VclipCollisionConstraint received unsupported collision geometry");
  }

  void VclipCollisionConstraint::initializeVclipModels() {
    if(!this->AGeometryModel_ || !this->BGeometryModel_) {
      throw std::invalid_argument("VclipCollisionConstraint geometry model is null");
    }
    this->invalidateVclipModels();

    for(size_t i=0; i<this->AGeometryModel_->geometryObjects.size(); i++) {
      const pinocchio::GeometryObject& geometryObject =
        this->AGeometryModel_->geometryObjects[i];
      if(geometryObject.parentJoint == this->getAJointId() &&
         geometryObject.geometry && !geometryObject.disableCollision) {
        VclipGeometry vclipGeometry;
        vclipGeometry.geometryObject = &geometryObject;
        vclipGeometry.model = vclip_eigen::convertToVClipModel(
          this->getGeometryVertices(geometryObject));
        this->AVclipGeometries_.push_back(vclipGeometry);
      }
    }
    for(size_t i=0; i<this->BGeometryModel_->geometryObjects.size(); i++) {
      const pinocchio::GeometryObject& geometryObject =
        this->BGeometryModel_->geometryObjects[i];
      if(geometryObject.parentJoint == this->getBJointId() &&
         geometryObject.geometry && !geometryObject.disableCollision) {
        VclipGeometry vclipGeometry;
        vclipGeometry.geometryObject = &geometryObject;
        vclipGeometry.model = vclip_eigen::convertToVClipModel(
          this->getGeometryVertices(geometryObject));
        this->BVclipGeometries_.push_back(vclipGeometry);
      }
    }
    if(this->AVclipGeometries_.empty() || this->BVclipGeometries_.empty()) {
      throw std::invalid_argument(
        "VclipCollisionConstraint joint does not have supported collision geometry");
    }

    this->initializedAModel_ = this->getAModel();
    this->initializedBModel_ = this->getBModel();
    this->initializedAGeometryModel_ = this->AGeometryModel_;
    this->initializedBGeometryModel_ = this->BGeometryModel_;
    this->initializedAJointId_ = this->getAJointId();
    this->initializedBJointId_ = this->getBJointId();
    this->vclipModelsInitialized_ = true;
  }

  void VclipCollisionConstraint::updateBounds(
    const std::vector<const pinocchio::Data*>& data) {
    const std::vector<std::shared_ptr<const pinocchio::Model> >& models =
      this->getModels();
    if(data.size() != models.size()) {
      throw std::invalid_argument("VclipCollisionConstraint received an invalid data vector");
    }
    this->AData_ = nullptr;
    this->BData_ = nullptr;
    for(size_t i=0; i<models.size(); i++) {
      if(models[i].get() == this->getAModel().get()) this->AData_ = data[i];
      if(models[i].get() == this->getBModel().get()) this->BData_ = data[i];
    }
    if(!this->AData_ || !this->BData_) {
      throw std::invalid_argument("VclipCollisionConstraint model data is not found");
    }
    if(!this->vclipModelsInitialized_ ||
       this->initializedAModel_.get() != this->getAModel().get() ||
       this->initializedBModel_.get() != this->getBModel().get() ||
       this->initializedAGeometryModel_.get() != this->AGeometryModel_.get() ||
       this->initializedBGeometryModel_.get() != this->BGeometryModel_.get() ||
       this->initializedAJointId_ != this->getAJointId() ||
       this->initializedBJointId_ != this->getBJointId()) {
      this->initializeVclipModels();
    }
    CollisionConstraint::updateBounds(data);
  }

  const pinocchio::Data& VclipCollisionConstraint::getData(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    if(model.get() == this->getAModel().get() && this->AData_) return *this->AData_;
    if(model.get() == this->getBModel().get() && this->BData_) return *this->BData_;
    throw std::invalid_argument("VclipCollisionConstraint model data is not available");
  }

  pinocchio::SE3 VclipCollisionConstraint::getGeometryWorldPlacement(
    const pinocchio::Data& data,
    const pinocchio::GeometryObject& geometryObject) const {
    if(geometryObject.parentJoint == 0) return geometryObject.placement;
    return data.oMi[geometryObject.parentJoint] * geometryObject.placement;
  }

  bool VclipCollisionConstraint::computeDistance(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    pinocchio::JointIndex AJointId,
    const std::shared_ptr<const pinocchio::Model>& BModel,
    pinocchio::JointIndex BJointId,
    double& distance,
    Eigen::Vector3d& direction,
    Eigen::Vector3d& ALocalPosition,
    Eigen::Vector3d& BLocalPosition) {
    (void)AJointId;
    (void)BJointId;
    if(!this->vclipModelsInitialized_) return false;

    const pinocchio::Data& AData = this->getData(AModel);
    const pinocchio::Data& BData = this->getData(BModel);
    distance = std::numeric_limits<double>::max();
    bool distanceComputed = false;
    bool separatedResultFound = false;
    Eigen::Vector3d bestAJointLocalPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d bestBJointLocalPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d bestDirection = this->previousDirection_;

    for(size_t i=0; i<this->AVclipGeometries_.size(); i++) {
      const VclipGeometry& AGeometry = this->AVclipGeometries_[i];
      const pinocchio::SE3 AWorldPlacement =
        this->getGeometryWorldPlacement(AData, *AGeometry.geometryObject);
      for(size_t j=0; j<this->BVclipGeometries_.size(); j++) {
        const VclipGeometry& BGeometry = this->BVclipGeometries_[j];
        const pinocchio::SE3 BWorldPlacement =
          this->getGeometryWorldPlacement(BData, *BGeometry.geometryObject);
        double pairDistance = 0.0;
        Eigen::Vector3d AGeometryLocalPosition = Eigen::Vector3d::Zero();
        Eigen::Vector3d BGeometryLocalPosition = Eigen::Vector3d::Zero();
        const bool pairComputed = vclip_eigen::computeDistance(
          AGeometry.model, AWorldPlacement.translation(), AWorldPlacement.rotation(),
          BGeometry.model, BWorldPlacement.translation(), BWorldPlacement.rotation(),
          pairDistance, AGeometryLocalPosition, BGeometryLocalPosition);
        if(!pairComputed) continue;
        distanceComputed = true;
        if(pairDistance > 1e-6 && pairDistance < distance) {
          const Eigen::Vector3d AWorldPosition =
            AWorldPlacement.act(AGeometryLocalPosition);
          const Eigen::Vector3d BWorldPosition =
            BWorldPlacement.act(BGeometryLocalPosition);
          const Eigen::Vector3d difference = AWorldPosition - BWorldPosition;
          if(difference.norm() <= 1e-12) continue;
          distance = pairDistance;
          bestDirection = difference.normalized();
          bestAJointLocalPosition = AGeometry.geometryObject->placement.act(
            AGeometryLocalPosition);
          bestBJointLocalPosition = BGeometry.geometryObject->placement.act(
            BGeometryLocalPosition);
          separatedResultFound = true;
        } else if(!separatedResultFound && pairDistance < distance) {
          distance = pairDistance;
        }
      }
    }

    if(!distanceComputed) return false;
    if(separatedResultFound) {
      direction = bestDirection;
      ALocalPosition = bestAJointLocalPosition;
      BLocalPosition = bestBJointLocalPosition;
      this->previousDirection_ = direction;
      this->previousALocalPosition_ = ALocalPosition;
      this->previousBLocalPosition_ = BLocalPosition;
      this->hasPreviousSeparatedResult_ = true;
      return true;
    }

    if(!this->hasPreviousSeparatedResult_) return false;
    const Eigen::Vector3d AWorldPosition = AData.oMi[this->getAJointId()].act(
      this->previousALocalPosition_);
    const Eigen::Vector3d BWorldPosition = BData.oMi[this->getBJointId()].act(
      this->previousBLocalPosition_);
    distance = std::min(
      1e-6, (AWorldPosition - BWorldPosition).dot(this->previousDirection_));
    direction = this->previousDirection_;
    ALocalPosition = this->previousALocalPosition_;
    BLocalPosition = this->previousBLocalPosition_;
    return true;
  }
}
