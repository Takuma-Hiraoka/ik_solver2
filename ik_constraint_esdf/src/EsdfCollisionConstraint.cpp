#include <ik_constraint_esdf/EsdfCollisionConstraint.h>

#include <coal/BVH/BVH_model.h>
#include <coal/BV/OBBRSS.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/shape/geometric_shape_to_BVH_model.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace ik_constraint_esdf {
  std::shared_ptr<ik_constraint::IKConstraint> EsdfCollisionConstraint::clone() const {
    std::shared_ptr<EsdfCollisionConstraint> result =
      std::make_shared<EsdfCollisionConstraint>(*this);
    result->data_ = nullptr;
    result->hasPreviousValidGradient_ = false;
    return result;
  }

  EsdfCollisionConstraint::EsdfCollisionConstraint(
    const std::shared_ptr<const pinocchio::Model>& model,
    const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel,
    pinocchio::JointIndex jointId)
    : CollisionConstraint(model, jointId, model, 0),
      geometryModel_(geometryModel) {
    if(!this->geometryModel_) {
      throw std::invalid_argument("EsdfCollisionConstraint geometry model is null");
    }
  }

  void EsdfCollisionConstraint::setGeometryModel(
    const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel) {
    if(!geometryModel) {
      throw std::invalid_argument("EsdfCollisionConstraint geometry model is null");
    }
    this->geometryModel_ = geometryModel;
    this->invalidateSurfacePoints();
  }

  void EsdfCollisionConstraint::setResolution(const double& resolution) {
    if(!std::isfinite(resolution) || resolution <= 0.0) {
      throw std::invalid_argument("EsdfCollisionConstraint resolution must be positive");
    }
    this->resolution_ = resolution;
    this->invalidateSurfacePoints();
  }

  void EsdfCollisionConstraint::invalidateSurfacePoints() {
    this->surfacePoints_.clear();
    this->surfacePointsInitialized_ = false;
    this->hasPreviousValidGradient_ = false;
  }

  void EsdfCollisionConstraint::appendGeometrySurfacePoints(
    const pinocchio::GeometryObject& geometryObject,
    std::vector<Eigen::Vector3d>& points) const {
    if(!geometryObject.geometry) return;
    std::vector<Eigen::Vector3d> geometryPoints;
    const coal::BVHModelBase* bvhModel =
      dynamic_cast<const coal::BVHModelBase*>(geometryObject.geometry.get());
    const coal::Box* box = dynamic_cast<const coal::Box*>(geometryObject.geometry.get());
    // Conversion and sampling run only when the local surface-point cache is
    // initialized or invalidated. Keep the shared GeometryModel unchanged.
    std::unique_ptr<coal::BVHModel<coal::OBBRSS> > boxModel;
    if(box) {
      boxModel = std::make_unique<coal::BVHModel<coal::OBBRSS> >();
      coal::generateBVHModel(*boxModel,*box,coal::Transform3s::Identity());
      bvhModel = boxModel.get();
    }
    if(bvhModel && bvhModel->vertices) {
      if(bvhModel->tri_indices && bvhModel->num_tris > 0) {
        for(size_t i=0; i<bvhModel->num_tris; i++) {
          const coal::Triangle& triangle = (*bvhModel->tri_indices)[i];
          const Eigen::Vector3d v0 = (*bvhModel->vertices)[triangle[0]];
          const Eigen::Vector3d v1 = (*bvhModel->vertices)[triangle[1]];
          const Eigen::Vector3d v2 = (*bvhModel->vertices)[triangle[2]];
          const Eigen::Vector3d edge1 = v1 - v0;
          const Eigen::Vector3d edge2 = v2 - v0;
          const double longestEdge = std::max(edge1.norm(),std::max(edge2.norm(),(v2-v1).norm()));
          const size_t divisions = std::max(size_t(1),size_t(std::ceil(longestEdge/this->resolution_)));
          // A uniform barycentric grid bounds all three edge directions by
          // resolution. Small triangles contribute only their three vertices.
          for(size_t j=0; j<=divisions; j++) {
            const double ratio1 = double(j)/double(divisions);
            for(size_t k=0; k<=divisions-j; k++) {
              const double ratio2 = double(k)/double(divisions);
              geometryPoints.push_back(v0 + ratio1 * edge1 + ratio2 * edge2);
            }
          }
        }
      } else {
        for(size_t i=0; i<bvhModel->num_vertices; i++) {
          geometryPoints.push_back((*bvhModel->vertices)[i]);
        }
      }
    }

    const coal::ConvexBase* convex =
      dynamic_cast<const coal::ConvexBase*>(geometryObject.geometry.get());
    if(geometryPoints.empty() && convex && convex->points) {
      for(size_t i=0; i<convex->num_points; i++) {
        geometryPoints.push_back((*convex->points)[i]);
      }
    }

    const double pi = 3.14159265358979323846;
    const size_t circleSegments = std::max(
      size_t(12), size_t(std::ceil(2.0 * pi * 0.1 / this->resolution_)));
    const coal::Sphere* sphere =
      dynamic_cast<const coal::Sphere*>(geometryObject.geometry.get());
    if(geometryPoints.empty() && sphere) {
      const size_t latitudeSegments = std::max(
        size_t(6), size_t(std::ceil(pi * sphere->radius / this->resolution_)));
      for(size_t i=0; i<=latitudeSegments; i++) {
        const double latitude = pi * double(i) / double(latitudeSegments);
        const size_t longitudeCount = i == 0 || i == latitudeSegments
          ? size_t(1) : circleSegments;
        for(size_t j=0; j<longitudeCount; j++) {
          const double longitude = 2.0 * pi * double(j) / double(longitudeCount);
          geometryPoints.push_back(Eigen::Vector3d(
            sphere->radius * std::sin(latitude) * std::cos(longitude),
            sphere->radius * std::sin(latitude) * std::sin(longitude),
            sphere->radius * std::cos(latitude)));
        }
      }
    }

    const coal::Cylinder* cylinder =
      dynamic_cast<const coal::Cylinder*>(geometryObject.geometry.get());
    if(geometryPoints.empty() && cylinder) {
      const size_t axialSegments = std::max(
        size_t(1), size_t(std::ceil(
          2.0 * cylinder->halfLength / this->resolution_)));
      for(size_t i=0; i<=axialSegments; i++) {
        const double z = -cylinder->halfLength +
          2.0 * cylinder->halfLength * double(i) / double(axialSegments);
        for(size_t j=0; j<circleSegments; j++) {
          const double angle = 2.0 * pi * double(j) / double(circleSegments);
          geometryPoints.push_back(Eigen::Vector3d(
            cylinder->radius * std::cos(angle),
            cylinder->radius * std::sin(angle), z));
        }
      }
    }

    if(geometryPoints.empty()) {
      throw std::invalid_argument(
        "EsdfCollisionConstraint received unsupported collision geometry");
    }
    for(size_t i=0; i<geometryPoints.size(); i++) {
      points.push_back(geometryObject.placement.act(geometryPoints[i]));
    }
  }

  void EsdfCollisionConstraint::updateSurfacePoints() {
    if(!this->geometryModel_) {
      throw std::invalid_argument("EsdfCollisionConstraint geometry model is null");
    }
    std::vector<Eigen::Vector3d> sampledPoints;
    for(size_t i=0; i<this->geometryModel_->geometryObjects.size(); i++) {
      const pinocchio::GeometryObject& geometryObject =
        this->geometryModel_->geometryObjects[i];
      if(geometryObject.parentJoint == this->getAJointId() &&
         geometryObject.geometry && !geometryObject.disableCollision) {
        this->appendGeometrySurfacePoints(geometryObject, sampledPoints);
      }
    }
    if(sampledPoints.empty()) {
      throw std::invalid_argument(
        "EsdfCollisionConstraint joint does not have collision geometry");
    }

    std::set<std::array<long,3> > occupiedBins;
    this->surfacePoints_.clear();
    for(size_t i=0; i<sampledPoints.size(); i++) {
      const std::array<long,3> bin{
        long(std::floor(sampledPoints[i].x() / this->resolution_)),
        long(std::floor(sampledPoints[i].y() / this->resolution_)),
        long(std::floor(sampledPoints[i].z() / this->resolution_))};
      if(occupiedBins.insert(bin).second) this->surfacePoints_.push_back(sampledPoints[i]);
    }
    this->initializedModel_ = this->getAModel();
    this->initializedGeometryModel_ = this->geometryModel_;
    this->initializedJointId_ = this->getAJointId();
    this->initializedResolution_ = this->resolution_;
    this->surfacePointsInitialized_ = true;
  }

  void EsdfCollisionConstraint::updateBounds(
    const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != this->getModels().size() || data.empty()) {
      throw std::invalid_argument("EsdfCollisionConstraint received an invalid data vector");
    }
    this->data_ = data[0];
    if(!this->surfacePointsInitialized_ ||
       this->initializedModel_.get() != this->getAModel().get() ||
       this->initializedGeometryModel_.get() != this->geometryModel_.get() ||
       this->initializedJointId_ != this->getAJointId() ||
       this->initializedResolution_ != this->resolution_) {
      this->updateSurfacePoints();
    }
    CollisionConstraint::updateBounds(data);
  }

  bool EsdfCollisionConstraint::computeDistance(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    pinocchio::JointIndex AJointId,
    const std::shared_ptr<const pinocchio::Model>& BModel,
    pinocchio::JointIndex BJointId,
    double& distance,
    Eigen::Vector3d& direction,
    Eigen::Vector3d& ALocalPosition,
    Eigen::Vector3d& BLocalPosition) {
    (void)AModel;
    (void)BModel;
    (void)BJointId;
    if(!this->data_ || !this->field_) return false;
    const std::shared_ptr<voxblox::EsdfMap> field = this->field_;
    const Eigen::Isometry3d fieldOrigin = this->fieldOrigin_;
    const Eigen::Isometry3d fieldOriginInverse = fieldOrigin.inverse();
    const pinocchio::SE3& jointPlacement = this->data_->oMi[AJointId];

    double minimumDistance = this->maxDistance_;
    double minimumDistanceWithoutGradient = this->maxDistance_;
    Eigen::Vector3d closestALocalPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d closestPointField = Eigen::Vector3d::Zero();
    Eigen::Vector3d closestDirectionField = Eigen::Vector3d::UnitX();
    for(size_t i=0; i<this->surfacePoints_.size(); i++) {
      const Eigen::Vector3d worldPosition = jointPlacement.act(this->surfacePoints_[i]);
      const Eigen::Vector3d fieldPosition = fieldOriginInverse * worldPosition;
      double pointDistance = 0.0;
      Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
      if(!field->getDistanceAndGradientAtPosition(
           fieldPosition, &pointDistance, &gradient)) continue;
      minimumDistanceWithoutGradient = std::min(
        minimumDistanceWithoutGradient, pointDistance);
      if(gradient.norm() > 0.0 && pointDistance >= this->minDistance_ &&
         pointDistance < minimumDistance) {
        minimumDistance = pointDistance;
        closestALocalPosition = this->surfacePoints_[i];
        closestDirectionField = gradient.normalized();
        closestPointField = fieldPosition - closestDirectionField * pointDistance;
      }
    }

    if(minimumDistanceWithoutGradient >= this->maxDistance_) {
      distance = minimumDistanceWithoutGradient;
      direction = Eigen::Vector3d::UnitX();
      ALocalPosition = Eigen::Vector3d::Zero();
      BLocalPosition = Eigen::Vector3d::Zero();
      return true;
    }
    if(minimumDistance >= this->maxDistance_ ||
       minimumDistanceWithoutGradient < minimumDistance) {
      if(!this->hasPreviousValidGradient_) return false;
      const Eigen::Vector3d currentAWorldPosition =
        jointPlacement.act(this->previousALocalPosition_);
      distance = std::min(
        this->minDistance_,
        (currentAWorldPosition - this->previousBWorldPosition_).dot(
          this->previousDirection_));
      direction = this->previousDirection_;
      ALocalPosition = this->previousALocalPosition_;
      BLocalPosition = this->previousBWorldPosition_;
      return true;
    }

    distance = minimumDistance;
    direction = fieldOrigin.linear() * closestDirectionField;
    ALocalPosition = closestALocalPosition;
    BLocalPosition = fieldOrigin * closestPointField;
    this->previousDirection_ = direction;
    this->previousALocalPosition_ = ALocalPosition;
    this->previousBWorldPosition_ = BLocalPosition;
    this->hasPreviousValidGradient_ = true;
    return true;
  }
}
