#include <ik_constraint/CollisionConstraint.h>

#include <mujoco_viewer/mujoco_viewer.h>

#include <pinocchio/algorithm/frames.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ik_constraint{
  CollisionConstraint::CollisionConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    pinocchio::JointIndex AJointId,
    const std::shared_ptr<const pinocchio::Model>& BModel,
    pinocchio::JointIndex BJointId)
    : IKConstraint(
        AModel.get() == BModel.get()
        ? std::vector<std::shared_ptr<const pinocchio::Model> >{AModel}
        : std::vector<std::shared_ptr<const pinocchio::Model> >{AModel, BModel}),
      AModel_(AModel),
      AJointId_(AJointId),
      BModel_(BModel),
      BJointId_(BJointId) {
  }

  void CollisionConstraint::setAModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("CollisionConstraint A model is null");
    this->AModel_ = model;
    this->updateModels();
  }

  void CollisionConstraint::setBModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("CollisionConstraint B model is null");
    this->BModel_ = model;
    this->updateModels();
  }

  void CollisionConstraint::updateModels() {
    if(!this->AModel_ || !this->BModel_) {
      throw std::invalid_argument("CollisionConstraint model is null");
    }
    if(this->AModel_.get() == this->BModel_.get()) {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{this->AModel_});
    } else {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{
          this->AModel_, this->BModel_});
    }
  }

  void CollisionConstraint::checkJoints() const {
    if(!this->AModel_ || !this->BModel_) {
      throw std::invalid_argument("CollisionConstraint model is null");
    }
    if(this->AJointId_ >= this->AModel_->njoints ||
       this->BJointId_ >= this->BModel_->njoints) {
      throw std::out_of_range("CollisionConstraint joint ID is out of range");
    }
    if(this->velocityDamper_ == 0.0) {
      throw std::invalid_argument("CollisionConstraint velocity damper is zero");
    }
  }

  size_t CollisionConstraint::getConstraintModelIndex(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    for(size_t i=0; i<this->models_.size(); i++) {
      if(this->models_[i].get() == model.get()) return i;
    }
    throw std::invalid_argument("CollisionConstraint model is not registered in IKConstraint");
  }

  Eigen::Index CollisionConstraint::getConstraintVariableOffset(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    const size_t modelIndex = this->getConstraintModelIndex(model);
    Eigen::Index offset = 0;
    for(size_t i=0; i<modelIndex; i++) offset += this->models_[i]->nv;
    return offset;
  }

  Eigen::Vector3d CollisionConstraint::getWorldPosition(
    const pinocchio::Data& data,
    pinocchio::JointIndex jointId,
    const Eigen::Vector3d& localPosition) const {
    if(jointId == 0) return localPosition;
    return data.oMi[jointId].act(localPosition);
  }

  Eigen::Matrix<double,3,Eigen::Dynamic> CollisionConstraint::getPointJacobian(
    const std::shared_ptr<const pinocchio::Model>& model,
    pinocchio::Data& data,
    pinocchio::JointIndex jointId,
    const Eigen::Vector3d& localPosition) const {
    Eigen::Matrix<double,6,Eigen::Dynamic> jacobian6 =
      Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6, model->nv);
    if(jointId != 0) {
      pinocchio::getFrameJacobian(
        *model, data, jointId,
        pinocchio::SE3(Eigen::Matrix3d::Identity(), localPosition),
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian6);
    }
    return jacobian6.topRows<3>();
  }

  void CollisionConstraint::updateBounds(
    const std::vector<const pinocchio::Data*>& data) {
    this->checkJoints();
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("CollisionConstraint received an invalid data vector");
    }
    const bool distanceComputed = this->computeDistance(
      this->AModel_, this->AJointId_, this->BModel_, this->BJointId_,
      this->currentDistance_, this->currentDirection_,
      this->ACurrentLocalPosition_, this->BCurrentLocalPosition_);
    if(!distanceComputed) this->currentDistance_ = this->ignoreDistance_;

    const pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
    const pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
    this->ACurrentWorldPosition_ = this->getWorldPosition(
      AData, this->AJointId_, this->ACurrentLocalPosition_);
    this->BCurrentWorldPosition_ = this->getWorldPosition(
      BData, this->BJointId_, this->BCurrentLocalPosition_);

    this->eq_.resize(0);
    if(distanceComputed && this->currentDistance_ < this->ignoreDistance_) {
      this->minIneq_.resize(1);
      this->maxIneq_.resize(1);
      const double error = (this->tolerance_ - this->currentDistance_) /
        this->velocityDamper_;
      if(!this->invert_) {
        this->minIneq_[0] = std::min(error, this->maxError_) * this->weight_;
        this->maxIneq_[0] = 1e10;
      } else {
        this->minIneq_[0] = -1e10;
        this->maxIneq_[0] = std::max(error, -this->maxError_) * this->weight_;
      }
    } else {
      this->minIneq_.resize(0);
      this->maxIneq_.resize(0);
    }

    if(this->debugLevel_ >= 2) {
      std::cerr << "CollisionConstraint distance: " << this->currentDistance_ << std::endl;
      std::cerr << "CollisionConstraint direction: "
                << this->currentDirection_.transpose() << std::endl;
      std::cerr << "CollisionConstraint minIneq: "
                << this->minIneq_.transpose() << std::endl;
      std::cerr << "CollisionConstraint maxIneq: "
                << this->maxIneq_.transpose() << std::endl;
    }
  }

  void CollisionConstraint::updateJacobian(
    const std::vector<pinocchio::Data*>& data) {
    this->checkJoints();
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("CollisionConstraint received an invalid data vector");
    }
    Eigen::Index localDimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) localDimension += this->models_[i]->nv;
    this->jacobian_.resize(0, localDimension);
    if(this->currentDistance_ < this->ignoreDistance_) {
      pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
      pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
      const Eigen::Matrix<double,3,Eigen::Dynamic> AJacobian = this->getPointJacobian(
        this->AModel_, AData, this->AJointId_, this->ACurrentLocalPosition_);
      const Eigen::Matrix<double,3,Eigen::Dynamic> BJacobian = this->getPointJacobian(
        this->BModel_, BData, this->BJointId_, this->BCurrentLocalPosition_);
      Eigen::Matrix<double,3,Eigen::Dynamic> fullJacobian =
        Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3, localDimension);
      fullJacobian.middleCols(
        this->getConstraintVariableOffset(this->AModel_), this->AModel_->nv) += AJacobian;
      fullJacobian.middleCols(
        this->getConstraintVariableOffset(this->BModel_), this->BModel_->nv) -= BJacobian;
      const Eigen::Matrix<double,1,Eigen::Dynamic> distanceJacobian =
        this->weight_ * this->currentDirection_.transpose() * fullJacobian;
      this->jacobianIneq_ = distanceJacobian.sparseView();
    } else {
      this->jacobianIneq_.resize(0, localDimension);
    }
    this->jacobianExt_.resize(0, 0);
    this->jacobianIneqExt_.resize(this->jacobianIneq_.rows(), 0);
  }

  bool CollisionConstraint::isSatisfied() const {
    if(!this->invert_) {
      return this->currentDistance_ - this->tolerance_ >= -this->precision_;
    }
    return this->currentDistance_ - this->tolerance_ <= this->precision_;
  }

  double CollisionConstraint::distance() const {
    if(!this->invert_) {
      return std::abs(std::min(this->currentDistance_ - this->tolerance_, 0.0)) *
        this->weight_;
    }
    return std::abs(std::max(this->currentDistance_ - this->tolerance_, 0.0)) *
      this->weight_;
  }

  double CollisionConstraint::margin() const {
    if(!this->invert_) {
      return (this->currentDistance_ - this->tolerance_) * this->weight_;
    }
    return -(this->currentDistance_ - this->tolerance_) * this->weight_;
  }

  void CollisionConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    if(this->currentDistance_ >= this->ignoreDistance_) return;
    const std::array<mjtNum,3> APosition{
      this->ACurrentWorldPosition_.x(),
      this->ACurrentWorldPosition_.y(),
      this->ACurrentWorldPosition_.z()};
    const std::array<mjtNum,3> BPosition{
      this->BCurrentWorldPosition_.x(),
      this->BCurrentWorldPosition_.y(),
      this->BCurrentWorldPosition_.z()};
    const std::array<float,4> color{0.5f, 0.0f, 0.0f, 1.0f};
    mujoco_viewer::appendLine(viewer, APosition, BPosition, 1.0, color);
  }
}
