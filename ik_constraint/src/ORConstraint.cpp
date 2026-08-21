#include <ik_constraint/ORConstraint.h>

#include <algorithm>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace ik_constraint{
  std::vector<std::shared_ptr<const pinocchio::Model> > ORConstraint::collectModels(
    const std::vector<std::shared_ptr<IKConstraint> >& children) {
    if(children.empty()) throw std::invalid_argument("ORConstraint requires at least one child");
    std::vector<std::shared_ptr<const pinocchio::Model> > models;
    for(size_t i=0; i<children.size(); i++) {
      if(!children[i]) throw std::invalid_argument("ORConstraint contains a null child");
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

  ORConstraint::ORConstraint(
    const std::vector<std::shared_ptr<IKConstraint> >& children)
    : IKConstraint(ORConstraint::collectModels(children)),
      children_(children) {
  }

  void ORConstraint::setChildren(
    const std::vector<std::shared_ptr<IKConstraint> >& children) {
    const std::vector<std::shared_ptr<const pinocchio::Model> > models =
      ORConstraint::collectModels(children);
    this->children_ = children;
    this->setModels(models);
    this->activeIndex_ = -1;
  }

  std::vector<const pinocchio::Data*> ORConstraint::getChildConstData(
    const std::shared_ptr<IKConstraint>& child,
    const std::vector<const pinocchio::Data*>& data) const {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("ORConstraint received an invalid data vector");
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
      if(!found) throw std::invalid_argument("ORConstraint child model is not registered");
    }
    return childData;
  }

  std::vector<pinocchio::Data*> ORConstraint::getChildData(
    const std::shared_ptr<IKConstraint>& child,
    const std::vector<pinocchio::Data*>& data) const {
    if(data.size() != this->models_.size()) {
      throw std::invalid_argument("ORConstraint received an invalid data vector");
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
      if(!found) throw std::invalid_argument("ORConstraint child model is not registered");
    }
    return childData;
  }

  Eigen::SparseMatrix<double,Eigen::RowMajor> ORConstraint::convertChildJacobian(
    const std::shared_ptr<IKConstraint>& child,
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian) const {
    const std::vector<std::shared_ptr<const pinocchio::Model> >& childModels = child->getModels();
    Eigen::Index childDimension = 0;
    for(size_t i=0; i<childModels.size(); i++) childDimension += childModels[i]->nv;
    if(jacobian.cols() != childDimension) {
      throw std::invalid_argument("ORConstraint child Jacobian has an invalid column count");
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
      if(!found) throw std::invalid_argument("ORConstraint child model is not registered");
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

  void ORConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    this->activeIndex_ = -1;
    double maximumMargin = -std::numeric_limits<double>::max();
    for(size_t i=0; i<this->children_.size(); i++) {
      const std::vector<const pinocchio::Data*> childData =
        this->getChildConstData(this->children_[i], data);
      this->children_[i]->updateBounds(childData);
      const double childMargin = this->children_[i]->margin();
      if(childMargin > maximumMargin) {
        maximumMargin = childMargin;
        this->activeIndex_ = static_cast<int>(i);
      }
    }
    const std::shared_ptr<IKConstraint>& activeChild =
      this->children_[static_cast<size_t>(this->activeIndex_)];
    this->eq_ = activeChild->getEq();
    this->minIneq_ = activeChild->getMinIneq();
    this->maxIneq_ = activeChild->getMaxIneq();

    if(this->debugLevel_ >= 2) {
      std::cerr << "ORConstraint active child: " << this->activeIndex_ << std::endl;
    }
  }

  void ORConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    if(this->activeIndex_ < 0) throw std::logic_error("ORConstraint bounds are not updated");
    const std::shared_ptr<IKConstraint>& activeChild =
      this->children_[static_cast<size_t>(this->activeIndex_)];
    const std::vector<pinocchio::Data*> childData = this->getChildData(activeChild, data);
    activeChild->updateJacobian(childData);
    this->jacobian_ = this->convertChildJacobian(activeChild, activeChild->getJacobian());
    this->jacobianIneq_ =
      this->convertChildJacobian(activeChild, activeChild->getJacobianIneq());
    this->jacobianExt_ = activeChild->getJacobianExt();
    this->jacobianIneqExt_ = activeChild->getJacobianIneqExt();
  }

  bool ORConstraint::isSatisfied() const {
    for(size_t i=0; i<this->children_.size(); i++) {
      if(this->children_[i]->isSatisfied()) return true;
    }
    return false;
  }

  double ORConstraint::distance() const {
    if(this->activeIndex_ < 0) return 0.0;
    return this->children_[static_cast<size_t>(this->activeIndex_)]->distance();
  }

  double ORConstraint::margin() const {
    if(this->activeIndex_ < 0) return 0.0;
    return this->children_[static_cast<size_t>(this->activeIndex_)]->margin();
  }

  void ORConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    for(size_t i=0; i<this->children_.size(); i++) this->children_[i]->drawOn(viewer);
  }
}
