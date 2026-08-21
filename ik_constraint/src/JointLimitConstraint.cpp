#include <ik_constraint/JointLimitConstraint.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ik_constraint{
  JointLimitConstraint::JointLimitConstraint(
    const std::shared_ptr<const pinocchio::Model>& model)
    : IKConstraint(std::vector<std::shared_ptr<const pinocchio::Model> >{model}),
      model_(model) {
  }

  void JointLimitConstraint::setModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    if(!model) throw std::invalid_argument("JointLimitConstraint model is null");
    this->model_ = model;
    this->setModels(std::vector<std::shared_ptr<const pinocchio::Model> >{model});
  }

  void JointLimitConstraint::checkJoint() const {
    if(!this->model_) throw std::invalid_argument("JointLimitConstraint model is null");
    if(this->jointId_ == 0 || this->jointId_ >= this->model_->njoints) {
      throw std::out_of_range("JointLimitConstraint joint ID is out of range");
    }
    if(this->model_->joints[this->jointId_].nq() != 1 ||
       this->model_->joints[this->jointId_].nv() != 1) {
      throw std::invalid_argument("JointLimitConstraint requires a one-DOF joint with nq equal to one");
    }
  }

  void JointLimitConstraint::updateBounds(
    const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != 1 || !data[0]) {
      throw std::invalid_argument("JointLimitConstraint received an invalid data vector");
    }
    this->checkJoint();
    const Eigen::Index configurationIndex = this->model_->idx_qs[this->jointId_];
    double lower = this->model_->lowerPositionLimit[configurationIndex];
    double upper = this->model_->upperPositionLimit[configurationIndex];
    const double effectiveTolerance =
      std::min(this->tolerance_, 0.5 * std::max(upper - lower, 0.0));
    lower += effectiveTolerance;
    upper -= effectiveTolerance;
    const double jointValue = data[0]->joints[this->jointId_].joint_q()[0];
    this->currentLower_ = lower - jointValue;
    this->currentUpper_ = upper - jointValue;

    this->eq_.resize(0);
    this->minIneq_.resize(1);
    this->maxIneq_.resize(1);
    this->minIneq_[0] =
      std::min(this->currentLower_, this->maxError_) * this->weight_;
    this->maxIneq_[0] =
      std::max(this->currentUpper_, -this->maxError_) * this->weight_;

    if(this->debugLevel_ >= 1) {
      std::cerr << "JointLimitConstraint q: " << jointValue << std::endl;
      std::cerr << "JointLimitConstraint lower: " << lower << std::endl;
      std::cerr << "JointLimitConstraint upper: " << upper << std::endl;
      std::cerr << "JointLimitConstraint minIneq: " << this->minIneq_[0] << std::endl;
      std::cerr << "JointLimitConstraint maxIneq: " << this->maxIneq_[0] << std::endl;
    }
  }

  void JointLimitConstraint::updateJacobian(
    const std::vector<pinocchio::Data*>& data) {
    if(data.size() != 1 || !data[0]) {
      throw std::invalid_argument("JointLimitConstraint received an invalid data vector");
    }
    this->checkJoint();
    this->jacobian_.resize(0, this->model_->nv);
    this->jacobianIneq_.resize(1, this->model_->nv);
    this->jacobianIneq_.setZero();
    this->jacobianIneq_.insert(
      0, this->model_->idx_vs[this->jointId_]) = this->weight_;
    this->jacobianExt_.resize(0, 0);
    this->jacobianIneqExt_.resize(1, 0);

    if(this->debugLevel_ >= 1) {
      std::cerr << "JointLimitConstraint inequality Jacobian:" << std::endl;
      std::cerr << this->jacobianIneq_ << std::endl;
    }
  }

  bool JointLimitConstraint::isSatisfied() const {
    const double lowerViolation = std::max(this->minIneq_[0], 0.0);
    const double upperViolation = std::min(this->maxIneq_[0], 0.0);
    return std::sqrt(
      lowerViolation * lowerViolation + upperViolation * upperViolation) <= this->precision_;
  }

  double JointLimitConstraint::distance() const {
    const double lowerViolation = std::max(this->currentLower_, 0.0);
    const double upperViolation = std::min(this->currentUpper_, 0.0);
    return std::sqrt(
      lowerViolation * lowerViolation + upperViolation * upperViolation) * this->weight_;
  }
}
