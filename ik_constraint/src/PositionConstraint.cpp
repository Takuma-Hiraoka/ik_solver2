#include <ik_constraint/PositionConstraint.h>

#include <mujoco_viewer/mujoco_viewer.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>

namespace ik_constraint{
  PositionConstraint::PositionConstraint(
    const std::shared_ptr<const pinocchio::Model>& model)
    : PositionConstraint(model, model) {
  }

  PositionConstraint::PositionConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    const std::shared_ptr<const pinocchio::Model>& BModel)
    : IKConstraint(
        AModel.get() == BModel.get()
        ? std::vector<std::shared_ptr<const pinocchio::Model> >{AModel}
        : std::vector<std::shared_ptr<const pinocchio::Model> >{AModel, BModel}),
      AModel_(AModel),
      BModel_(BModel) {
  }

  void PositionConstraint::setAModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("PositionConstraint A model is null");
    this->AModel_ = model;
    this->updateModels();
  }

  void PositionConstraint::setBModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("PositionConstraint B model is null");
    this->BModel_ = model;
    this->updateModels();
  }

  void PositionConstraint::updateModels() {
    if(this->AModel_.get() == this->BModel_.get()) {
      this->setModels(std::vector<std::shared_ptr<const pinocchio::Model> >{this->AModel_});
    } else {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{this->AModel_, this->BModel_});
    }
  }

  pinocchio::SE3 PositionConstraint::getPlacement(
    const std::shared_ptr<const pinocchio::Model>& model,
    const pinocchio::Data& data,
    pinocchio::JointIndex jointId,
    const pinocchio::SE3& localPose) const {
    if(jointId == 0) return localPose;
    if(jointId >= model->njoints) {
      throw std::out_of_range("PositionConstraint joint ID is out of range");
    }
    return data.oMi[jointId] * localPose;
  }

  size_t PositionConstraint::getConstraintModelIndex(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    for(size_t i=0; i<this->models_.size(); i++) {
      if(this->models_[i].get() == model.get()) return i;
    }
    throw std::invalid_argument("PositionConstraint model is not registered in IKConstraint");
  }

  Eigen::Index PositionConstraint::getConstraintVariableOffset(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    const size_t modelIndex = this->getConstraintModelIndex(model);
    Eigen::Index offset = 0;
    for(size_t i=0; i<modelIndex; i++) offset += this->models_[i]->nv;
    return offset;
  }

  void PositionConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("PositionConstraint received an invalid data vector");
    }
    const pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
    const pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
    this->APosition_ =
      this->getPlacement(this->AModel_, AData, this->AJointId_, this->ALocalPose_);
    this->BPosition_ =
      this->getPlacement(this->BModel_, BData, this->BJointId_, this->BLocalPose_);

    this->currentError_.head<3>() =
      this->APosition_.translation() - this->BPosition_.translation();
    this->currentError_.tail<3>() =
      pinocchio::log3(this->APosition_.rotation() * this->BPosition_.rotation().transpose());

    Eigen::Index activeDimension = 0;
    for(Eigen::Index i=0; i<6; i++) {
      if(this->weight_[i] > 0.0) activeDimension++;
    }
    this->eq_.resize(activeDimension);
    Eigen::Index index = 0;
    for(Eigen::Index i=0; i<6; i++) {
      if(this->weight_[i] > 0.0) {
        this->eq_[index] = IKConstraint::clamp(
          -this->currentError_[i], this->maxError_[i]) * this->weight_[i];
        index++;
      }
    }
    this->minIneq_.resize(0);
    this->maxIneq_.resize(0);

    if(this->debugLevel_ >= 1) {
      std::cerr << "PositionConstraint error: " << this->currentError_.transpose() << std::endl;
      std::cerr << "PositionConstraint eq: " << this->eq_.transpose() << std::endl;
    }
  }

  Eigen::Matrix<double,6,Eigen::Dynamic> PositionConstraint::getJointJacobian(
    const std::shared_ptr<const pinocchio::Model>& model,
    pinocchio::Data& data,
    pinocchio::JointIndex jointId,
    const pinocchio::SE3& localPose) const {
    Eigen::Matrix<double,6,Eigen::Dynamic> jacobian =
      Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6, model->nv);
    if(jointId == 0) return jacobian;
    if(jointId >= model->njoints) {
      throw std::out_of_range("PositionConstraint joint ID is out of range");
    }

    pinocchio::getFrameJacobian(
      *model, data, jointId, localPose,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    return jacobian;
  }

  void PositionConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("PositionConstraint received an invalid data vector");
    }
    pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
    pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
    const Eigen::Matrix<double,6,Eigen::Dynamic> AJacobian =
      this->getJointJacobian(this->AModel_, AData, this->AJointId_, this->ALocalPose_);
    const Eigen::Matrix<double,6,Eigen::Dynamic> BJacobian =
      this->getJointJacobian(this->BModel_, BData, this->BJointId_, this->BLocalPose_);
    Eigen::Index localDimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) localDimension += this->models_[i]->nv;
    Eigen::Matrix<double,6,Eigen::Dynamic> fullJacobian =
      Eigen::Matrix<double,6,Eigen::Dynamic>::Zero(6, localDimension);
    fullJacobian.middleCols(
      this->getConstraintVariableOffset(this->AModel_), this->AModel_->nv) += AJacobian;
    fullJacobian.middleCols(
      this->getConstraintVariableOffset(this->BModel_), this->BModel_->nv) -= BJacobian;

    Eigen::Index activeDimension = 0;
    for(Eigen::Index i=0; i<6; i++) {
      if(this->weight_[i] > 0.0) activeDimension++;
    }
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> activeJacobian(
      activeDimension, localDimension);
    Eigen::Index index = 0;
    for(Eigen::Index i=0; i<6; i++) {
      if(this->weight_[i] > 0.0) {
        activeJacobian.row(index) = this->weight_[i] * fullJacobian.row(i);
        index++;
      }
    }
    this->jacobian_ = activeJacobian.sparseView();
    this->jacobianIneq_.resize(0, localDimension);
    this->jacobianExt_.resize(activeDimension, 0);
    this->jacobianIneqExt_.resize(0, 0);

    if(this->debugLevel_ >= 1) {
      std::cerr << "PositionConstraint Jacobian:" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }
  }

  bool PositionConstraint::isSatisfied() const {
    return this->distance() <= this->precision_;
  }

  double PositionConstraint::distance() const {
    return this->currentError_.cwiseProduct(this->weight_).norm();
  }

  void PositionConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    const std::array<float,4> white{1.0f, 1.0f, 1.0f, 1.0f};
    const std::array<float,4> red{1.0f, 0.0f, 0.0f, 1.0f};
    const std::array<float,4> green{0.0f, 1.0f, 0.0f, 1.0f};
    const std::array<float,4> blue{0.0f, 0.0f, 1.0f, 1.0f};
    const std::array<std::array<float,4>,3> axisColors{red, green, blue};
    const double axisLength = 0.05;
    const double lineWidth = 1.0;

    const std::array<mjtNum,3> AOrigin{
      this->APosition_.translation().x(),
      this->APosition_.translation().y(),
      this->APosition_.translation().z()};
    const std::array<mjtNum,3> BOrigin{
      this->BPosition_.translation().x(),
      this->BPosition_.translation().y(),
      this->BPosition_.translation().z()};

    for(Eigen::Index i=0; i<3; i++) {
      const Eigen::Vector3d AAxisEnd =
        this->APosition_.translation() +
        axisLength * this->APosition_.rotation().col(i);
      const Eigen::Vector3d BAxisEnd =
        this->BPosition_.translation() +
        axisLength * this->BPosition_.rotation().col(i);
      const std::array<mjtNum,3> AAxisEndArray{
        AAxisEnd.x(), AAxisEnd.y(), AAxisEnd.z()};
      const std::array<mjtNum,3> BAxisEndArray{
        BAxisEnd.x(), BAxisEnd.y(), BAxisEnd.z()};
      mujoco_viewer::appendLine(
        viewer, AOrigin, AAxisEndArray, lineWidth, axisColors[i]);
      mujoco_viewer::appendLine(
        viewer, BOrigin, BAxisEndArray, lineWidth, axisColors[i]);
    }
    mujoco_viewer::appendLine(viewer, AOrigin, BOrigin, lineWidth, white);
  }
}
