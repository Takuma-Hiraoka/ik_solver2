#include <ik_constraint/IKConstraint.h>

#include <mujoco_viewer/mujoco_viewer.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ik_constraint{
  IKConstraint::IKConstraint(
    const std::vector<std::shared_ptr<const pinocchio::Model> >& models)
    : models_() {
    this->setModels(models);
  }

  void IKConstraint::setModels(
    const std::vector<std::shared_ptr<const pinocchio::Model> >& models) {
    if(models.empty()) {
      throw std::invalid_argument("IKConstraint requires at least one Pinocchio model");
    }
    for(size_t i=0; i<models.size(); i++) {
      if(!models[i]) {
        throw std::invalid_argument("IKConstraint model vector contains a null model");
      }
      for(size_t j=0; j<i; j++) {
        if(models[i].get() == models[j].get()) {
          throw std::invalid_argument("IKConstraint model vector contains a duplicate model");
        }
      }
    }
    this->models_ = models;
  }

  double IKConstraint::distance() const {
    return std::sqrt(
      this->eq_.squaredNorm() +
      this->minIneq_.cwiseMax(Eigen::VectorXd::Zero(this->minIneq_.size())).squaredNorm() +
      this->maxIneq_.cwiseMin(Eigen::VectorXd::Zero(this->maxIneq_.size())).squaredNorm());
  }

  double IKConstraint::margin() const {
    if(!this->isSatisfied()) return -this->distance();
    if(this->eq_.size() > 0) return 0.0;
    if(this->minIneq_.size() == 0 || this->maxIneq_.size() == 0) return 0.0;
    return std::min(std::abs(this->minIneq_.maxCoeff()), std::abs(this->maxIneq_.minCoeff()));
  }

  void IKConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    (void)viewer;
  }

  size_t IKConstraint::getModelIndex(
    const std::shared_ptr<const pinocchio::Model>& model,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) const {
    for(size_t i=0; i<models.size(); i++) {
      if(models[i].get() == model.get()) return i;
    }
    throw std::invalid_argument("IKConstraint model is not contained in the solver model vector");
  }

  size_t IKConstraint::getVariableOffset(
    const std::shared_ptr<const pinocchio::Model>& model,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) const {
    const size_t modelIndex = this->getModelIndex(model, models);
    size_t offset = 0;
    for(size_t i=0; i<modelIndex; i++) offset += static_cast<size_t>(models[i]->nv);
    return offset;
  }

  size_t IKConstraint::getVariableDimension(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) {
    size_t dimension = 0;
    for(size_t i=0; i<models.size(); i++) {
      if(!models[i]) throw std::invalid_argument("solver model vector contains a null model");
      dimension += static_cast<size_t>(models[i]->nv);
    }
    return dimension;
  }

  std::vector<std::shared_ptr<pinocchio::Data> > IKConstraint::createDataVector(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) {
    std::vector<std::shared_ptr<pinocchio::Data> > data;
    data.reserve(models.size());
    for(size_t i=0; i<models.size(); i++) {
      if(!models[i]) throw std::invalid_argument("solver model vector contains a null model");
      data.push_back(std::make_shared<pinocchio::Data>(*models[i]));
    }
    return data;
  }

  pinocchio::Data& IKConstraint::getData(
    const std::shared_ptr<const pinocchio::Model>& model,
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data) {
    if(models.size() != data.size()) {
      throw std::invalid_argument("model and data vector sizes do not match");
    }
    for(size_t i=0; i<models.size(); i++) {
      if(models[i].get() == model.get()) {
        if(!data[i]) throw std::invalid_argument("data vector contains null data");
        return *data[i];
      }
    }
    throw std::invalid_argument("constraint model is not contained in the solver model vector");
  }

  std::vector<const pinocchio::Data*> IKConstraint::getConstDataVector(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data) const {
    std::vector<const pinocchio::Data*> constraintData;
    constraintData.reserve(this->models_.size());
    for(size_t i=0; i<this->models_.size(); i++) {
      constraintData.push_back(&IKConstraint::getData(this->models_[i], models, data));
    }
    return constraintData;
  }

  std::vector<pinocchio::Data*> IKConstraint::getDataVector(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data) const {
    std::vector<pinocchio::Data*> constraintData;
    constraintData.reserve(this->models_.size());
    for(size_t i=0; i<this->models_.size(); i++) {
      constraintData.push_back(&IKConstraint::getData(this->models_[i], models, data));
    }
    return constraintData;
  }

  void IKConstraint::convertJacobian(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models) {
    Eigen::Index localDimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) localDimension += this->models_[i]->nv;
    const Eigen::Index variableDimension =
      static_cast<Eigen::Index>(IKConstraint::getVariableDimension(models));
    if(this->jacobian_.cols() != localDimension) {
      throw std::invalid_argument("equality Jacobian column count does not match constraint models");
    }
    if(this->jacobianIneq_.cols() != localDimension) {
      throw std::invalid_argument("inequality Jacobian column count does not match constraint models");
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian(
      this->jacobian_.rows(), variableDimension);
    Eigen::SparseMatrix<double,Eigen::ColMajor> jacobianColMajor(
      this->jacobian_.rows(), variableDimension);
    Eigen::Index localOffset = 0;
    for(size_t i=0; i<this->models_.size(); i++) {
      const Eigen::Index variableOffset = static_cast<Eigen::Index>(
        this->getVariableOffset(this->models_[i], models));
      jacobianColMajor.middleCols(variableOffset, this->models_[i]->nv) =
        this->jacobian_.middleCols(localOffset, this->models_[i]->nv);
      localOffset += this->models_[i]->nv;
    }
    jacobian = jacobianColMajor;
    this->jacobian_ = jacobian;

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneq(
      this->jacobianIneq_.rows(), variableDimension);
    Eigen::SparseMatrix<double,Eigen::ColMajor> jacobianIneqColMajor(
      this->jacobianIneq_.rows(), variableDimension);
    localOffset = 0;
    for(size_t i=0; i<this->models_.size(); i++) {
      const Eigen::Index variableOffset = static_cast<Eigen::Index>(
        this->getVariableOffset(this->models_[i], models));
      jacobianIneqColMajor.middleCols(variableOffset, this->models_[i]->nv) =
        this->jacobianIneq_.middleCols(localOffset, this->models_[i]->nv);
      localOffset += this->models_[i]->nv;
    }
    jacobianIneq = jacobianIneqColMajor;
    this->jacobianIneq_ = jacobianIneq;
  }

  double IKConstraint::clamp(const double& value, const double& limitValue) {
    return std::min(std::max(value, -limitValue), limitValue);
  }
}
