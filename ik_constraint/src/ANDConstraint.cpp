#include <ik_constraint/ANDConstraint.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ik_constraint{
  std::vector<std::shared_ptr<const pinocchio::Model> > ANDConstraint::collectModels(
    const std::vector<std::shared_ptr<IKConstraint> >& children) {
    if(children.empty()) throw std::invalid_argument("ANDConstraint requires at least one child");
    std::vector<std::shared_ptr<const pinocchio::Model> > models;
    for(size_t i=0; i<children.size(); i++) {
      if(!children[i]) throw std::invalid_argument("ANDConstraint contains a null child");
      const std::vector<std::shared_ptr<const pinocchio::Model> >& childModels =
        children[i]->getModels();
      for(size_t j=0; j<childModels.size(); j++) {
        bool found = false;
        for(size_t k=0; k<models.size(); k++) {
          if(models[k].get() == childModels[j].get()) found = true;
        }
        if(!found) models.push_back(childModels[j]);
      }
    }
    return models;
  }

  ANDConstraint::ANDConstraint(
    const std::vector<std::shared_ptr<IKConstraint> >& children)
    : IKConstraint(ANDConstraint::collectModels(children)),
      children_(children) {
  }

  void ANDConstraint::setChildren(
    const std::vector<std::shared_ptr<IKConstraint> >& children) {
    const std::vector<std::shared_ptr<const pinocchio::Model> > models =
      ANDConstraint::collectModels(children);
    this->children_ = children;
    this->setModels(models);
  }

  std::vector<const pinocchio::Data*> ANDConstraint::getChildConstData(
    const std::shared_ptr<IKConstraint>& child,
    const std::vector<const pinocchio::Data*>& data) const {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("ANDConstraint received an invalid data vector");
    }
    std::vector<const pinocchio::Data*> childData;
    const std::vector<std::shared_ptr<const pinocchio::Model> >& childModels = child->getModels();
    childData.reserve(childModels.size());
    for(size_t i=0; i<childModels.size(); i++) {
      bool found = false;
      for(size_t j=0; j<this->models_.size(); j++) {
        if(childModels[i].get() == this->models_[j].get()) {
          childData.push_back(data[j]);
          found = true;
          break;
        }
      }
      if(!found) throw std::invalid_argument("ANDConstraint child model is not registered");
    }
    return childData;
  }

  std::vector<pinocchio::Data*> ANDConstraint::getChildData(
    const std::shared_ptr<IKConstraint>& child,
    const std::vector<pinocchio::Data*>& data) const {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("ANDConstraint received an invalid data vector");
    }
    std::vector<pinocchio::Data*> childData;
    const std::vector<std::shared_ptr<const pinocchio::Model> >& childModels = child->getModels();
    childData.reserve(childModels.size());
    for(size_t i=0; i<childModels.size(); i++) {
      bool found = false;
      for(size_t j=0; j<this->models_.size(); j++) {
        if(childModels[i].get() == this->models_[j].get()) {
          childData.push_back(data[j]);
          found = true;
          break;
        }
      }
      if(!found) throw std::invalid_argument("ANDConstraint child model is not registered");
    }
    return childData;
  }

  Eigen::SparseMatrix<double,Eigen::RowMajor> ANDConstraint::convertChildJacobian(
    const std::shared_ptr<IKConstraint>& child,
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian) const {
    const std::vector<std::shared_ptr<const pinocchio::Model> >& childModels = child->getModels();
    Eigen::Index childDimension = 0;
    for(size_t i=0; i<childModels.size(); i++) childDimension += childModels[i]->nv;
    if(jacobian.cols() != childDimension) {
      throw std::invalid_argument("ANDConstraint child Jacobian has an invalid column count");
    }
    Eigen::Index dimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) dimension += this->models_[i]->nv;
    std::vector<Eigen::Index> columnMap(static_cast<size_t>(childDimension));
    Eigen::Index childOffset = 0;
    for(size_t i=0; i<childModels.size(); i++) {
      Eigen::Index offset = 0;
      bool found = false;
      for(size_t j=0; j<this->models_.size(); j++) {
        if(childModels[i].get() == this->models_[j].get()) {
          found = true;
          break;
        }
        offset += this->models_[j]->nv;
      }
      if(!found) throw std::invalid_argument("ANDConstraint child model is not registered");
      for(Eigen::Index j=0; j<childModels[i]->nv; j++) {
        columnMap[static_cast<size_t>(childOffset + j)] = offset + j;
      }
      childOffset += childModels[i]->nv;
    }
    std::vector<Eigen::Triplet<double> > triplets;
    triplets.reserve(static_cast<size_t>(jacobian.nonZeros()));
    for(Eigen::Index row=0; row<jacobian.outerSize(); row++) {
      for(Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator entry(jacobian, row);
          entry; ++entry) {
        triplets.push_back(Eigen::Triplet<double>(
          entry.row(), columnMap[static_cast<size_t>(entry.col())], entry.value()));
      }
    }
    Eigen::SparseMatrix<double,Eigen::RowMajor> converted(jacobian.rows(), dimension);
    converted.setFromTriplets(triplets.begin(), triplets.end());
    return converted;
  }

  void ANDConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    Eigen::Index equalityRows = 0;
    Eigen::Index inequalityRows = 0;
    for(size_t i=0; i<this->children_.size(); i++) {
      const std::vector<const pinocchio::Data*> childData =
        this->getChildConstData(this->children_[i], data);
      this->children_[i]->updateBounds(childData);
      equalityRows += this->children_[i]->getEq().rows();
      inequalityRows += this->children_[i]->getMinIneq().rows();
      if(this->children_[i]->getMinIneq().rows() !=
         this->children_[i]->getMaxIneq().rows()) {
        throw std::invalid_argument("ANDConstraint child inequality dimensions do not match");
      }
    }
    this->eq_.resize(equalityRows);
    this->minIneq_.resize(inequalityRows);
    this->maxIneq_.resize(inequalityRows);
    Eigen::Index equalityOffset = 0;
    Eigen::Index inequalityOffset = 0;
    for(size_t i=0; i<this->children_.size(); i++) {
      const Eigen::Index childEqualityRows = this->children_[i]->getEq().rows();
      const Eigen::Index childInequalityRows = this->children_[i]->getMinIneq().rows();
      this->eq_.segment(equalityOffset, childEqualityRows) = this->children_[i]->getEq();
      this->minIneq_.segment(inequalityOffset, childInequalityRows) =
        this->children_[i]->getMinIneq();
      this->maxIneq_.segment(inequalityOffset, childInequalityRows) =
        this->children_[i]->getMaxIneq();
      equalityOffset += childEqualityRows;
      inequalityOffset += childInequalityRows;
    }
  }

  void ANDConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    Eigen::Index dimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) dimension += this->models_[i]->nv;
    Eigen::Index equalityRows = 0;
    Eigen::Index inequalityRows = 0;
    Eigen::Index extensionColumns = 0;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > equalityJacobians;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > inequalityJacobians;
    equalityJacobians.reserve(this->children_.size());
    inequalityJacobians.reserve(this->children_.size());
    for(size_t i=0; i<this->children_.size(); i++) {
      const std::vector<pinocchio::Data*> childData =
        this->getChildData(this->children_[i], data);
      this->children_[i]->updateJacobian(childData);
      equalityJacobians.push_back(this->convertChildJacobian(
        this->children_[i], this->children_[i]->getJacobian()));
      inequalityJacobians.push_back(this->convertChildJacobian(
        this->children_[i], this->children_[i]->getJacobianIneq()));
      equalityRows += equalityJacobians[i].rows();
      inequalityRows += inequalityJacobians[i].rows();
      extensionColumns += std::max(
        this->children_[i]->getJacobianExt().cols(),
        this->children_[i]->getJacobianIneqExt().cols());
    }
    std::vector<Eigen::Triplet<double> > equalityTriplets;
    std::vector<Eigen::Triplet<double> > inequalityTriplets;
    std::vector<Eigen::Triplet<double> > equalityExtensionTriplets;
    std::vector<Eigen::Triplet<double> > inequalityExtensionTriplets;
    Eigen::Index equalityOffset = 0;
    Eigen::Index inequalityOffset = 0;
    Eigen::Index extensionOffset = 0;
    for(size_t i=0; i<this->children_.size(); i++) {
      for(Eigen::Index row=0; row<equalityJacobians[i].outerSize(); row++) {
        for(Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator entry(
              equalityJacobians[i], row); entry; ++entry) {
          equalityTriplets.push_back(Eigen::Triplet<double>(
            equalityOffset + entry.row(), entry.col(), entry.value()));
        }
      }
      for(Eigen::Index row=0; row<inequalityJacobians[i].outerSize(); row++) {
        for(Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator entry(
              inequalityJacobians[i], row); entry; ++entry) {
          inequalityTriplets.push_back(Eigen::Triplet<double>(
            inequalityOffset + entry.row(), entry.col(), entry.value()));
        }
      }
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& childEqualityExtension =
        this->children_[i]->getJacobianExt();
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& childInequalityExtension =
        this->children_[i]->getJacobianIneqExt();
      for(Eigen::Index row=0; row<childEqualityExtension.outerSize(); row++) {
        for(Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator entry(
              childEqualityExtension, row); entry; ++entry) {
          equalityExtensionTriplets.push_back(Eigen::Triplet<double>(
            equalityOffset + entry.row(), extensionOffset + entry.col(), entry.value()));
        }
      }
      for(Eigen::Index row=0; row<childInequalityExtension.outerSize(); row++) {
        for(Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator entry(
              childInequalityExtension, row); entry; ++entry) {
          inequalityExtensionTriplets.push_back(Eigen::Triplet<double>(
            inequalityOffset + entry.row(), extensionOffset + entry.col(), entry.value()));
        }
      }
      equalityOffset += equalityJacobians[i].rows();
      inequalityOffset += inequalityJacobians[i].rows();
      extensionOffset += std::max(
        childEqualityExtension.cols(), childInequalityExtension.cols());
    }
    this->jacobian_.resize(equalityRows, dimension);
    this->jacobian_.setFromTriplets(equalityTriplets.begin(), equalityTriplets.end());
    this->jacobianIneq_.resize(inequalityRows, dimension);
    this->jacobianIneq_.setFromTriplets(
      inequalityTriplets.begin(), inequalityTriplets.end());
    this->jacobianExt_.resize(equalityRows, extensionColumns);
    this->jacobianExt_.setFromTriplets(
      equalityExtensionTriplets.begin(), equalityExtensionTriplets.end());
    this->jacobianIneqExt_.resize(inequalityRows, extensionColumns);
    this->jacobianIneqExt_.setFromTriplets(
      inequalityExtensionTriplets.begin(), inequalityExtensionTriplets.end());
  }

  bool ANDConstraint::isSatisfied() const {
    for(size_t i=0; i<this->children_.size(); i++) {
      if(!this->children_[i]->isSatisfied()) return false;
    }
    return true;
  }

  double ANDConstraint::distance() const {
    double squaredDistance = 0.0;
    for(size_t i=0; i<this->children_.size(); i++) {
      squaredDistance += std::pow(this->children_[i]->distance(), 2);
    }
    return std::sqrt(squaredDistance);
  }

  double ANDConstraint::margin() const {
    if(!this->isSatisfied()) return -this->distance();
    double minimumMargin = std::numeric_limits<double>::max();
    for(size_t i=0; i<this->children_.size(); i++) {
      minimumMargin = std::min(minimumMargin, this->children_[i]->margin());
    }
    return minimumMargin;
  }

  void ANDConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    for(size_t i=0; i<this->children_.size(); i++) this->children_[i]->drawOn(viewer);
  }
}
