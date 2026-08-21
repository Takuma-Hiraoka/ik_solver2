#include <ik_constraint/COMConstraint.h>

#include <mujoco_viewer/mujoco_viewer.h>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ik_constraint{
  COMConstraint::COMConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel)
    : IKConstraint(std::vector<std::shared_ptr<const pinocchio::Model> >{AModel}),
      AModel_(AModel) {
  }

  COMConstraint::COMConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    const std::shared_ptr<const pinocchio::Model>& BModel)
    : IKConstraint(
        AModel.get() == BModel.get()
        ? std::vector<std::shared_ptr<const pinocchio::Model> >{AModel}
        : std::vector<std::shared_ptr<const pinocchio::Model> >{AModel, BModel}),
      AModel_(AModel),
      BModel_(BModel) {
    if(!BModel) throw std::invalid_argument("COMConstraint B model is null");
  }

  void COMConstraint::setAModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("COMConstraint A model is null");
    this->AModel_ = model;
    this->updateModels();
  }

  void COMConstraint::setBModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    this->BModel_ = model;
    this->updateModels();
  }

  void COMConstraint::updateModels() {
    if(!this->AModel_) throw std::invalid_argument("COMConstraint A model is null");
    if(!this->BModel_ || this->AModel_.get() == this->BModel_.get()) {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{this->AModel_});
    } else {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{
          this->AModel_, this->BModel_});
    }
  }

  size_t COMConstraint::getConstraintModelIndex(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    for(size_t i=0; i<this->models_.size(); i++) {
      if(this->models_[i].get() == model.get()) return i;
    }
    throw std::invalid_argument("COMConstraint model is not registered in IKConstraint");
  }

  Eigen::Index COMConstraint::getConstraintVariableOffset(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    const size_t modelIndex = this->getConstraintModelIndex(model);
    Eigen::Index offset = 0;
    for(size_t i=0; i<modelIndex; i++) offset += this->models_[i]->nv;
    return offset;
  }

  void COMConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("COMConstraint received an invalid data vector");
    }
    const pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
    this->APosition_ = AData.com[0] + this->ALocalPosition_;
    if(this->BModel_) {
      const pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
      this->BPosition_ = BData.com[0] + this->BLocalPosition_;
    } else {
      this->BPosition_ = this->BLocalPosition_;
    }
    this->currentErrorEval_ =
      this->evalRotation_.transpose() * (this->APosition_ - this->BPosition_);

    Eigen::Index activeDimension = 0;
    for(Eigen::Index i=0; i<3; i++) {
      if(this->weight_[i] > 0.0) activeDimension++;
    }
    this->eq_.resize(activeDimension);
    Eigen::Index index = 0;
    for(Eigen::Index i=0; i<3; i++) {
      if(this->weight_[i] > 0.0) {
        this->eq_[index] = IKConstraint::clamp(
          -this->currentErrorEval_[i], this->maxError_[i]) * this->weight_[i];
        index++;
      }
    }

    if(this->C_.cols() != 3 || this->C_.rows() != this->dl_.size() ||
       this->du_.size() != this->dl_.size()) {
      std::cerr << "COMConstraint inequality dimension mismatch" << std::endl;
      this->C_.resize(0,3);
      this->dl_.resize(0);
      this->du_.resize(0);
    }
    if(this->C_.rows() != this->maxCErrorVector_.size()) {
      this->maxCErrorVector_ = Eigen::VectorXd::Constant(this->C_.rows(), this->maxCError_);
    }
    const Eigen::VectorXd CError = this->C_ * this->currentErrorEval_;
    this->currentUpper_ = this->du_ - CError -
      this->CTolerance_ * Eigen::VectorXd::Ones(this->du_.size());
    this->currentLower_ = this->dl_ - CError +
      this->CTolerance_ * Eigen::VectorXd::Ones(this->dl_.size());
    this->maxIneq_.resize(this->C_.rows());
    this->minIneq_.resize(this->C_.rows());
    for(Eigen::Index i=0; i<this->C_.rows(); i++) {
      this->maxIneq_[i] = std::max(this->currentUpper_[i], -this->maxCErrorVector_[i]);
      this->minIneq_[i] = std::min(this->currentLower_[i], this->maxCErrorVector_[i]);
    }

    if(this->debugLevel_ >= 1) {
      std::cerr << "COMConstraint A COM: " << this->APosition_.transpose() << std::endl;
      std::cerr << "COMConstraint B COM: " << this->BPosition_.transpose() << std::endl;
      std::cerr << "COMConstraint error: " << this->currentErrorEval_.transpose() << std::endl;
      std::cerr << "COMConstraint eq: " << this->eq_.transpose() << std::endl;
      std::cerr << "COMConstraint minIneq: " << this->minIneq_.transpose() << std::endl;
      std::cerr << "COMConstraint maxIneq: " << this->maxIneq_.transpose() << std::endl;
    }
  }

  void COMConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("COMConstraint received an invalid data vector");
    }
    Eigen::Index localDimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) localDimension += this->models_[i]->nv;
    Eigen::Matrix<double,3,Eigen::Dynamic> fullJacobian =
      Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3, localDimension);

    pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
    const Eigen::Matrix<double,3,Eigen::Dynamic>& AJacobian =
      pinocchio::jacobianCenterOfMass(*this->AModel_, AData, false);
    fullJacobian.middleCols(
      this->getConstraintVariableOffset(this->AModel_), this->AModel_->nv) += AJacobian;
    if(this->BModel_) {
      pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
      const Eigen::Matrix<double,3,Eigen::Dynamic>& BJacobian =
        pinocchio::jacobianCenterOfMass(*this->BModel_, BData, false);
      fullJacobian.middleCols(
        this->getConstraintVariableOffset(this->BModel_), this->BModel_->nv) -= BJacobian;
    }
    const Eigen::Matrix<double,3,Eigen::Dynamic> localJacobian =
      this->evalRotation_.transpose() * fullJacobian;

    Eigen::Index activeDimension = 0;
    for(Eigen::Index i=0; i<3; i++) {
      if(this->weight_[i] > 0.0) activeDimension++;
    }
    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> activeJacobian(
      activeDimension, localDimension);
    Eigen::Index index = 0;
    for(Eigen::Index i=0; i<3; i++) {
      if(this->weight_[i] > 0.0) {
        activeJacobian.row(index) = this->weight_[i] * localJacobian.row(i);
        index++;
      }
    }
    this->jacobian_ = activeJacobian.sparseView();
    const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> inequalityJacobian =
      this->C_ * localJacobian;
    this->jacobianIneq_ = inequalityJacobian.sparseView();
    this->jacobianExt_.resize(activeDimension, 0);
    this->jacobianIneqExt_.resize(this->C_.rows(), 0);

    if(this->debugLevel_ >= 1) {
      std::cerr << "COMConstraint Jacobian:" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
      std::cerr << "COMConstraint inequality Jacobian:" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }
  }

  bool COMConstraint::isSatisfied() const {
    if(this->currentErrorEval_.cwiseProduct(this->weight_).norm() > this->precision_) {
      return false;
    }
    for(Eigen::Index i=0; i<this->currentLower_.size(); i++) {
      if(this->currentLower_[i] > this->CPrecision_) return false;
    }
    for(Eigen::Index i=0; i<this->currentUpper_.size(); i++) {
      if(this->currentUpper_[i] < -this->CPrecision_) return false;
    }
    return true;
  }

  double COMConstraint::distance() const {
    return std::sqrt(
      this->currentErrorEval_.cwiseProduct(this->weight_).squaredNorm() +
      this->currentLower_.cwiseMax(
        Eigen::VectorXd::Zero(this->currentLower_.size())).squaredNorm() +
      this->currentUpper_.cwiseMin(
        Eigen::VectorXd::Zero(this->currentUpper_.size())).squaredNorm());
  }

  void COMConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    const std::array<float,4> white{1.0f, 1.0f, 1.0f, 1.0f};
    const std::array<float,4> red{1.0f, 0.0f, 0.0f, 1.0f};
    const std::array<float,4> green{0.0f, 1.0f, 0.0f, 1.0f};
    const std::array<float,4> blue{0.0f, 0.0f, 1.0f, 1.0f};
    const std::array<std::array<float,4>,3> axisColors{red, green, blue};
    const double axisLength = 0.05;
    const double lineWidth = 1.0;
    const std::array<mjtNum,3> AOrigin{
      this->APosition_.x(), this->APosition_.y(), this->APosition_.z()};
    const std::array<mjtNum,3> BOrigin{
      this->BPosition_.x(), this->BPosition_.y(), this->BPosition_.z()};
    for(Eigen::Index i=0; i<3; i++) {
      const Eigen::Vector3d AAxisEnd =
        this->APosition_ + axisLength * Eigen::Vector3d::Unit(i);
      const Eigen::Vector3d BAxisEnd =
        this->BPosition_ + axisLength * Eigen::Vector3d::Unit(i);
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
