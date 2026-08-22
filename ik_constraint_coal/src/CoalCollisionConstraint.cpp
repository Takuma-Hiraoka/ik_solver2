#include <ik_constraint_coal/CoalCollisionConstraint.h>

#include <limits>
#include <stdexcept>

namespace ik_constraint_coal{
  CoalCollisionConstraint::CoalCollisionConstraint(
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
      throw std::invalid_argument("CoalCollisionConstraint geometry model is null");
    }
  }

  void CoalCollisionConstraint::setAGeometryModel(
    const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel) {
    if(!geometryModel) {
      throw std::invalid_argument("CoalCollisionConstraint A geometry model is null");
    }
    this->AGeometryModel_ = geometryModel;
    this->invalidateDistanceQueries();
  }

  void CoalCollisionConstraint::setBGeometryModel(
    const std::shared_ptr<const pinocchio::GeometryModel>& geometryModel) {
    if(!geometryModel) {
      throw std::invalid_argument("CoalCollisionConstraint B geometry model is null");
    }
    this->BGeometryModel_ = geometryModel;
    this->invalidateDistanceQueries();
  }

  void CoalCollisionConstraint::invalidateDistanceQueries() {
    this->AGeometryObjects_.clear();
    this->BGeometryObjects_.clear();
    this->queryAGeometryIndices_.clear();
    this->queryBGeometryIndices_.clear();
    this->distanceQueries_.clear();
    this->distanceQueriesInitialized_ = false;
  }

  void CoalCollisionConstraint::initializeDistanceQueries() {
    if(!this->AGeometryModel_ || !this->BGeometryModel_) {
      throw std::invalid_argument("CoalCollisionConstraint geometry model is null");
    }
    this->invalidateDistanceQueries();
    for(size_t i=0; i<this->AGeometryModel_->geometryObjects.size(); i++) {
      const pinocchio::GeometryObject& geometryObject =
        this->AGeometryModel_->geometryObjects[i];
      if(geometryObject.parentJoint == this->getAJointId() &&
         geometryObject.geometry && !geometryObject.disableCollision) {
        geometryObject.geometry->computeLocalAABB();
        this->AGeometryObjects_.push_back(&geometryObject);
      }
    }
    for(size_t i=0; i<this->BGeometryModel_->geometryObjects.size(); i++) {
      const pinocchio::GeometryObject& geometryObject =
        this->BGeometryModel_->geometryObjects[i];
      if(geometryObject.parentJoint == this->getBJointId() &&
         geometryObject.geometry && !geometryObject.disableCollision) {
        geometryObject.geometry->computeLocalAABB();
        this->BGeometryObjects_.push_back(&geometryObject);
      }
    }
    if(this->AGeometryObjects_.empty() || this->BGeometryObjects_.empty()) {
      throw std::invalid_argument(
        "CoalCollisionConstraint link does not have collision geometry");
    }
    for(size_t i=0; i<this->AGeometryObjects_.size(); i++) {
      for(size_t j=0; j<this->BGeometryObjects_.size(); j++) {
        this->queryAGeometryIndices_.push_back(i);
        this->queryBGeometryIndices_.push_back(j);
        this->distanceQueries_.push_back(std::make_shared<coal::ComputeDistance>(
          this->AGeometryObjects_[i]->geometry.get(),
          this->BGeometryObjects_[j]->geometry.get()));
      }
    }
    this->initializedAModel_ = this->getAModel();
    this->initializedBModel_ = this->getBModel();
    this->initializedAJointId_ = this->getAJointId();
    this->initializedBJointId_ = this->getBJointId();
    this->distanceQueriesInitialized_ = true;
  }

  void CoalCollisionConstraint::updateBounds(
    const std::vector<const pinocchio::Data*>& data) {
    const std::vector<std::shared_ptr<const pinocchio::Model> >& models = this->getModels();
    if(data.size() != models.size()) {
      throw std::invalid_argument("CoalCollisionConstraint received an invalid data vector");
    }
    this->AData_ = nullptr;
    this->BData_ = nullptr;
    for(size_t i=0; i<models.size(); i++) {
      if(models[i].get() == this->getAModel().get()) this->AData_ = data[i];
      if(models[i].get() == this->getBModel().get()) this->BData_ = data[i];
    }
    if(!this->AData_ || !this->BData_) {
      throw std::invalid_argument("CoalCollisionConstraint model data is not found");
    }
    if(!this->distanceQueriesInitialized_ ||
       this->initializedAModel_.get() != this->getAModel().get() ||
       this->initializedBModel_.get() != this->getBModel().get() ||
       this->initializedAJointId_ != this->getAJointId() ||
       this->initializedBJointId_ != this->getBJointId()) {
      this->initializeDistanceQueries();
    }
    CollisionConstraint::updateBounds(data);
  }

  const pinocchio::Data& CoalCollisionConstraint::getData(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    if(model.get() == this->getAModel().get() && this->AData_) return *this->AData_;
    if(model.get() == this->getBModel().get() && this->BData_) return *this->BData_;
    throw std::invalid_argument("CoalCollisionConstraint model data is not available");
  }

  pinocchio::SE3 CoalCollisionConstraint::getGeometryWorldPlacement(
    const pinocchio::Data& data,
    const pinocchio::GeometryObject& geometryObject) const {
    if(geometryObject.parentJoint == 0) return geometryObject.placement;
    return data.oMi[geometryObject.parentJoint] * geometryObject.placement;
  }

  coal::Transform3s CoalCollisionConstraint::convertTransform(
    const pinocchio::SE3& transform) const {
    return coal::Transform3s(transform.rotation(), transform.translation());
  }

  bool CoalCollisionConstraint::computeDistance(
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
    if(!this->distanceQueriesInitialized_) return false;
    const pinocchio::Data& AData = this->getData(AModel);
    const pinocchio::Data& BData = this->getData(BModel);
    distance = std::numeric_limits<double>::max();
    Eigen::Vector3d bestAWorldPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d bestBWorldPosition = Eigen::Vector3d::Zero();
    Eigen::Vector3d bestDirection = Eigen::Vector3d::UnitX();
    const coal::DistanceRequest request(true, true);

    for(size_t i=0; i<this->distanceQueries_.size(); i++) {
      const pinocchio::GeometryObject& AGeometryObject =
        *this->AGeometryObjects_[this->queryAGeometryIndices_[i]];
      const pinocchio::GeometryObject& BGeometryObject =
        *this->BGeometryObjects_[this->queryBGeometryIndices_[i]];
      const pinocchio::SE3 APlacement =
        this->getGeometryWorldPlacement(AData, AGeometryObject);
      const pinocchio::SE3 BPlacement =
        this->getGeometryWorldPlacement(BData, BGeometryObject);
      coal::DistanceResult result;
      (*this->distanceQueries_[i])(
        this->convertTransform(APlacement), this->convertTransform(BPlacement),
        request, result);
      if(result.min_distance < distance &&
         result.nearest_points[0].allFinite() &&
         result.nearest_points[1].allFinite()) {
        distance = result.min_distance;
        bestAWorldPosition = result.nearest_points[0];
        bestBWorldPosition = result.nearest_points[1];
        if(result.normal.allFinite() && result.normal.norm() > 1e-12) {
          bestDirection = -result.normal.normalized();
        } else if((bestAWorldPosition - bestBWorldPosition).norm() > 1e-12) {
          bestDirection = (bestAWorldPosition - bestBWorldPosition).normalized();
        }
      }
    }
    if(distance == std::numeric_limits<double>::max()) return false;
    direction = bestDirection;
    if(this->getAJointId() == 0) {
      ALocalPosition = bestAWorldPosition;
    } else {
      ALocalPosition = AData.oMi[this->getAJointId()].actInv(bestAWorldPosition);
    }
    if(this->getBJointId() == 0) {
      BLocalPosition = bestBWorldPosition;
    } else {
      BLocalPosition = BData.oMi[this->getBJointId()].actInv(bestBWorldPosition);
    }
    return true;
  }
}
