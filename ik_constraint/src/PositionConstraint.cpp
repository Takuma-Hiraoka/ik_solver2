#include <ik_constraint/PositionConstraint.h>

#include <mujoco_viewer/mujoco_viewer.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace ik_constraint{
  namespace {
    // Logarithm of the rotation taking axis to normal on the unit sphere.
    // Unlike a cross product alone, this also rejects antiparallel axes.
    void axisError(const Eigen::Vector3d& normal, const Eigen::Vector3d& axis,
      Eigen::Vector3d& error, Eigen::Matrix3d& angularJacobian) {
      const Eigen::Vector3d cross = axis.cross(normal);
      const double sine = cross.norm();
      const double cosine = std::clamp(axis.dot(normal),-1.0,1.0);
      if(sine > 1e-8) {
        const double angle = std::atan2(sine,cosine);
        error = (angle / sine) * cross;
        const Eigen::Vector3d gradient =
          ((cosine * sine - angle) / (sine * sine * sine)) *
            (normal - cosine * axis) - axis;
        const Eigen::Matrix3d derivative =
          (angle / sine) * pinocchio::skew(axis) + cross * gradient.transpose();
        angularJacobian = -derivative * pinocchio::skew(normal);
      } else {
        error.setZero();
        if(cosine < 0.0) {
          const Eigen::Index freeAxis = axis.x() > 0.5 ? 0 : (axis.y() > 0.5 ? 1 : 2);
          error[(freeAxis + 1) % 3] = std::acos(-1.0);
        }
        angularJacobian = Eigen::Matrix3d::Identity() - axis * axis.transpose();
      }
    }
  }

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
    if(this->evalModel_ && this->evalModel_.get() != this->AModel_.get() &&
       this->evalModel_.get() != this->BModel_.get()) {
      this->models_.push_back(this->evalModel_);
    }
  }

  void PositionConstraint::setEvalModel(
    const std::shared_ptr<const pinocchio::Model>& model) {
    this->evalModel_ = model;
    this->updateModels();
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

    this->evalRotation_ = this->evalLocalR_;
    if(this->evalJointId_ != 0) {
      if(!this->evalModel_) throw std::invalid_argument("PositionConstraint evaluation model is null");
      const pinocchio::Data& evalData = *data[this->getConstraintModelIndex(this->evalModel_)];
      this->evalRotation_ = this->getPlacement(this->evalModel_,evalData,
        this->evalJointId_,pinocchio::SE3(this->evalLocalR_,Eigen::Vector3d::Zero())).rotation();
    }
    const Eigen::Matrix3d worldToEval = this->evalRotation_.transpose();
    this->currentError_.head<3>() = worldToEval *
      (this->APosition_.translation() - this->BPosition_.translation());
    this->rotationErrorJacobian_ = worldToEval;
    this->axisAlignment_ = false;
    const bool evalIsA = this->evalJointId_ == this->AJointId_ &&
      (this->evalJointId_ == 0 || this->evalModel_.get() == this->AModel_.get()) &&
      this->evalLocalR_.isApprox(this->ALocalPose_.rotation(),1e-12);
    const bool evalIsB = this->evalJointId_ == this->BJointId_ &&
      (this->evalJointId_ == 0 || this->evalModel_.get() == this->BModel_.get()) &&
      this->evalLocalR_.isApprox(this->BLocalPose_.rotation(),1e-12);
    const Eigen::Index rotationDimension = (this->weight_.tail<3>().array() > 0.0).count();
    if(rotationDimension == 2 && (evalIsA || evalIsB)) {
      Eigen::Vector3d axis = Eigen::Vector3d::Zero();
      for(Eigen::Index i = 0; i < 3; ++i)
        if(this->weight_[3+i] <= 0.0) axis[i] = 1.0;
      // Use B when both frames match. Otherwise account for A's moving frame.
      const bool useB = evalIsB;
      const Eigen::Vector3d normal = worldToEval *
        ((useB ? this->APosition_.rotation() : this->BPosition_.rotation()) * axis);
      Eigen::Vector3d error;
      Eigen::Matrix3d angularJacobian;
      axisError(normal,axis,error,angularJacobian);
      this->currentError_.tail<3>() = useB ? error : -error;
      this->rotationErrorJacobian_ = angularJacobian * worldToEval;
      this->axisAlignment_ = true;
    } else {
      Eigen::Matrix3d ARotation = this->APosition_.rotation();
      Eigen::Matrix3d BRotation = this->BPosition_.rotation();
      if(rotationDimension == 1 && (evalIsA || evalIsB)) {
        // With two free rotational axes, remove the swing and measure twist.
        Eigen::Vector3d axis = Eigen::Vector3d::Zero();
        for(Eigen::Index i = 0; i < 3; ++i)
          if(this->weight_[3+i] > 0.0) axis[i] = 1.0;
        if(evalIsA) {
          BRotation = Eigen::Quaterniond::FromTwoVectors(
            BRotation * axis,ARotation * axis).toRotationMatrix() * BRotation;
        } else {
          ARotation = Eigen::Quaterniond::FromTwoVectors(
            ARotation * axis,BRotation * axis).toRotationMatrix() * ARotation;
        }
      }
      this->currentError_.tail<3>() = worldToEval *
        pinocchio::log3(ARotation * BRotation.transpose());
    }

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

    Eigen::Matrix<double,3,Eigen::Dynamic> evalAngularJacobian =
      Eigen::Matrix<double,3,Eigen::Dynamic>::Zero(3,localDimension);
    if(this->evalJointId_ != 0) {
      if(!this->evalModel_) throw std::invalid_argument("PositionConstraint evaluation model is null");
      pinocchio::Data& evalData = *data[this->getConstraintModelIndex(this->evalModel_)];
      const Eigen::Matrix<double,6,Eigen::Dynamic> evalJacobian = this->getJointJacobian(
        this->evalModel_,evalData,this->evalJointId_,pinocchio::SE3::Identity());
      evalAngularJacobian.middleCols(
        this->getConstraintVariableOffset(this->evalModel_),this->evalModel_->nv) = evalJacobian.bottomRows<3>();
    }
    const Eigen::Matrix3d worldToEval = this->evalRotation_.transpose();
    fullJacobian.topRows<3>() = (worldToEval * fullJacobian.topRows<3>() +
      pinocchio::skew(this->currentError_.head<3>()) * worldToEval * evalAngularJacobian).eval();
    fullJacobian.bottomRows<3>() =
      (this->rotationErrorJacobian_ * fullJacobian.bottomRows<3>()).eval();
    if(!this->axisAlignment_) {
      fullJacobian.bottomRows<3>() += pinocchio::skew(this->currentError_.tail<3>()) *
        worldToEval * evalAngularJacobian;
    }

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
