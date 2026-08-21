#include <ik_constraint/JointAngleConstraint.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ik_constraint{
  JointAngleConstraint::JointAngleConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel)
    : IKConstraint(std::vector<std::shared_ptr<const pinocchio::Model> >{AModel}),
      AModel_(AModel) {
  }

  JointAngleConstraint::JointAngleConstraint(
    const std::shared_ptr<const pinocchio::Model>& AModel,
    const std::shared_ptr<const pinocchio::Model>& BModel)
    : IKConstraint(
        AModel.get() == BModel.get()
        ? std::vector<std::shared_ptr<const pinocchio::Model> >{AModel}
        : std::vector<std::shared_ptr<const pinocchio::Model> >{AModel, BModel}),
      AModel_(AModel),
      BModel_(BModel) {
    if(!BModel) throw std::invalid_argument("JointAngleConstraint B model is null");
  }

  void JointAngleConstraint::setAModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("JointAngleConstraint A model is null");
    this->AModel_ = model;
    this->updateModels();
  }

  void JointAngleConstraint::setBModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    this->BModel_ = model;
    this->updateModels();
  }

  void JointAngleConstraint::updateModels() {
    if(!this->AModel_) throw std::invalid_argument("JointAngleConstraint A model is null");
    if(!this->BModel_ || this->AModel_.get() == this->BModel_.get()) {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{this->AModel_});
    } else {
      this->setModels(
        std::vector<std::shared_ptr<const pinocchio::Model> >{
          this->AModel_, this->BModel_});
    }
  }

  size_t JointAngleConstraint::getConstraintModelIndex(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    for(size_t i=0; i<this->models_.size(); i++) {
      if(this->models_[i].get() == model.get()) return i;
    }
    throw std::invalid_argument(
      "JointAngleConstraint model is not registered in IKConstraint");
  }

  Eigen::Index JointAngleConstraint::getConstraintVariableOffset(
    const std::shared_ptr<const pinocchio::Model>& model) const {
    const size_t modelIndex = this->getConstraintModelIndex(model);
    Eigen::Index offset = 0;
    for(size_t i=0; i<modelIndex; i++) offset += this->models_[i]->nv;
    return offset;
  }

  void JointAngleConstraint::checkJoint(
    const std::shared_ptr<const pinocchio::Model>& model,
    pinocchio::JointIndex jointId) const {
    if(!model) throw std::invalid_argument("JointAngleConstraint model is null");
    if(jointId == 0 || jointId >= model->njoints) {
      throw std::out_of_range("JointAngleConstraint joint ID is out of range");
    }
    if(model->joints[jointId].nq() != 1 || model->joints[jointId].nv() != 1) {
      throw std::invalid_argument("JointAngleConstraint requires a one-DOF joint with nq equal to one");
    }
  }

  double JointAngleConstraint::getJointValue(
    const std::shared_ptr<const pinocchio::Model>& model,
    const pinocchio::Data& data,
    pinocchio::JointIndex jointId) const {
    this->checkJoint(model, jointId);
    return data.joints[jointId].joint_q()[0];
  }

  void JointAngleConstraint::updateBounds(
    const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("JointAngleConstraint received an invalid data vector");
    }
    const pinocchio::Data& AData = *data[this->getConstraintModelIndex(this->AModel_)];
    const double AValue =
      this->getJointValue(this->AModel_, AData, this->AJointId_) + this->AOffset_;
    double BValue = this->BOffset_;
    if(this->BModel_) {
      const pinocchio::Data& BData = *data[this->getConstraintModelIndex(this->BModel_)];
      BValue += this->getJointValue(this->BModel_, BData, this->BJointId_);
    }
    this->currentError_ = BValue - AValue;
    this->eq_.resize(1);
    this->eq_[0] = this->weight_ *
      IKConstraint::clamp(this->currentError_, this->maxError_);
    this->minIneq_.resize(0);
    this->maxIneq_.resize(0);

    if(this->debugLevel_ >= 1) {
      std::cerr << "JointAngleConstraint A: " << AValue << std::endl;
      std::cerr << "JointAngleConstraint B: " << BValue << std::endl;
      std::cerr << "JointAngleConstraint eq: " << this->eq_[0] << std::endl;
    }
  }

  void JointAngleConstraint::updateJacobian(
    const std::vector<pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("JointAngleConstraint received an invalid data vector");
    }
    this->checkJoint(this->AModel_, this->AJointId_);
    if(this->BModel_) this->checkJoint(this->BModel_, this->BJointId_);
    Eigen::Index localDimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) localDimension += this->models_[i]->nv;
    Eigen::Matrix<double,1,Eigen::Dynamic> jacobian =
      Eigen::Matrix<double,1,Eigen::Dynamic>::Zero(1, localDimension);
    const Eigen::Index AColumn = this->getConstraintVariableOffset(this->AModel_) +
      this->AModel_->idx_vs[this->AJointId_];
    jacobian[AColumn] += this->weight_;
    if(this->BModel_) {
      const Eigen::Index BColumn = this->getConstraintVariableOffset(this->BModel_) +
        this->BModel_->idx_vs[this->BJointId_];
      jacobian[BColumn] -= this->weight_;
    }
    this->jacobian_ = jacobian.sparseView();
    this->jacobianIneq_.resize(0, localDimension);
    this->jacobianExt_.resize(1, 0);
    this->jacobianIneqExt_.resize(0, 0);

    if(this->debugLevel_ >= 1) {
      std::cerr << "JointAngleConstraint Jacobian:" << std::endl;
      std::cerr << this->jacobian_ << std::endl;
    }
  }

  bool JointAngleConstraint::isSatisfied() const {
    return this->distance() <= this->precision_;
  }

  double JointAngleConstraint::distance() const {
    return std::abs(this->currentError_ * this->weight_);
  }
}
