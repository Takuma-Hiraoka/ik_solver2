#include <ik_constraint_sp/SPConstraint.h>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
  double cross(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
    return (b.x()-a.x())*(c.y()-a.y())-(b.y()-a.y())*(c.x()-a.x());
  }
  std::array<mjtNum,3> point(const Eigen::Vector3d& p) {
    return std::array<mjtNum,3>{p.x(),p.y(),p.z()};
  }
}

namespace ik_constraint_sp {
  SPConstraint::SPConstraint(const std::vector<std::shared_ptr<const pinocchio::Model> >& models) {
    this->setModels(models);
  }

  std::shared_ptr<ik_constraint::IKConstraint> SPConstraint::clone() const {
    return std::make_shared<SPConstraint>(*this);
  }

  void SPConstraint::invalidate() {
    this->valid_ = false;
    this->hull_.clear();
    this->worldPolygons_.clear();
    this->margins_.resize(0);
  }

  size_t SPConstraint::modelIndex(const std::shared_ptr<const pinocchio::Model>& model) const {
    for(size_t i=0; i<this->models_.size(); i++) {
      if(this->models_[i] == model) return i;
    }
    throw std::invalid_argument("SPConstraint model is not registered");
  }

  void SPConstraint::updateModels() {
    std::vector<std::shared_ptr<const pinocchio::Model> > models = this->COMModels_;
    for(size_t i=0; i<this->polygons_.size(); i++) {
      const std::shared_ptr<const pinocchio::Model>& model = this->polygons_[i].model;
      if(model && std::find(models.begin(), models.end(), model) == models.end()) models.push_back(model);
    }
    if(!models.empty()) this->IKConstraint::setModels(models);
    this->invalidate();
  }

  void SPConstraint::setModels(const std::vector<std::shared_ptr<const pinocchio::Model> >& models) {
    if(models.empty()) throw std::invalid_argument("SPConstraint requires COM models");
    std::vector<double> masses;
    double mass = 0.0;
    for(size_t i=0; i<models.size(); i++) {
      if(!models[i] || std::find(models.begin(), models.begin()+i, models[i]) != models.begin()+i) {
        throw std::invalid_argument("SPConstraint COM models must be non-null and unique");
      }
      double modelMass = 0.0;
      for(size_t j=1; j<models[i]->inertias.size(); j++) modelMass += models[i]->inertias[j].mass();
      if(!std::isfinite(modelMass) || modelMass < 0.0) throw std::invalid_argument("Invalid model mass");
      masses.push_back(modelMass);
      mass += modelMass;
    }
    if(!std::isfinite(mass) || mass <= 0.0) throw std::invalid_argument("SPConstraint total mass must be finite and positive");
    this->COMModels_ = models;
    this->masses_ = masses;
    this->mass_ = mass;
    this->updateModels();
  }

  void SPConstraint::setSupportPolygons(const std::vector<SupportPolygon>& polygons) {
    for(size_t i=0; i<polygons.size(); i++) {
      const SupportPolygon& polygon = polygons[i];
      if((!polygon.model && polygon.jointId != 0) ||
         (polygon.model && polygon.jointId >= polygon.model->njoints)) {
        throw std::invalid_argument("SPConstraint invalid support joint ID");
      }
      if(polygon.vertices.size() < 3 || !polygon.localPose.toHomogeneousMatrix().allFinite()) {
        throw std::invalid_argument("SPConstraint invalid support polygon");
      }
      double area = 0.0;
      for(size_t j=0; j<polygon.vertices.size(); j++) {
        const Eigen::Vector2d& a = polygon.vertices[j];
        const Eigen::Vector2d& b = polygon.vertices[(j+1)%polygon.vertices.size()];
        if(!a.allFinite()) throw std::invalid_argument("SPConstraint non-finite support vertex");
        area += a.x()*b.y()-a.y()*b.x();
      }
      if(area >= -1e-12) throw std::invalid_argument("Support vertices must be clockwise and non-degenerate");
    }
    this->polygons_ = polygons;
    this->updateModels();
  }

  void SPConstraint::setTolerance(double value) {
    if(!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Invalid SP tolerance");
    this->tolerance_ = value;
    this->invalidate();
  }
  void SPConstraint::setPrecision(double value) {
    if(!std::isfinite(value) || value < 0.0) throw std::invalid_argument("Invalid SP precision");
    this->precision_ = value;
  }
  void SPConstraint::setMaxError(double value) {
    if(!std::isfinite(value) || value <= 0.0) throw std::invalid_argument("Invalid SP max error");
    this->maxError_ = value;
  }

  void SPConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    this->invalidate();
    if(this->COMModels_.empty() || this->polygons_.empty()) {
      throw std::logic_error("Register COM models and support polygons before using SPConstraint");
    }
    if(data.size() != this->models_.size()) throw std::invalid_argument("Invalid SPConstraint data vector");
    for(size_t i=0; i<data.size(); i++) {
      if(!data[i]) throw std::invalid_argument("Null SPConstraint data");
    }
    this->COM_.setZero();
    for(size_t i=0; i<this->COMModels_.size(); i++) {
      if(this->masses_[i] > 0.0) this->COM_ += this->masses_[i]/this->mass_*data[this->modelIndex(this->COMModels_[i])]->com[0];
    }
    if(!this->COM_.allFinite()) throw std::runtime_error("SPConstraint non-finite COM");
    std::vector<Eigen::Vector3d> points;
    for(size_t i=0; i<this->polygons_.size(); i++) {
      const SupportPolygon& polygon = this->polygons_[i];
      const pinocchio::SE3 pose = polygon.model && polygon.jointId != 0 ?
        data[this->modelIndex(polygon.model)]->oMi[polygon.jointId]*polygon.localPose : polygon.localPose;
      std::vector<Eigen::Vector3d> world;
      for(size_t j=0; j<polygon.vertices.size(); j++) {
        const Eigen::Vector3d p = pose.act(Eigen::Vector3d(polygon.vertices[j].x(),polygon.vertices[j].y(),0.0));
        if(!p.allFinite()) throw std::runtime_error("SPConstraint non-finite support position");
        world.push_back(p);
        points.push_back(p);
      }
      this->worldPolygons_.push_back(world);
    }
    std::sort(points.begin(),points.end(),[](const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
      return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    points.erase(std::unique(points.begin(),points.end(),[](const Eigen::Vector3d& a,const Eigen::Vector3d& b) {
      return (a.head<2>()-b.head<2>()).squaredNorm() < 1e-24;
    }),points.end());
    // Monotone chain: counter-clockwise hull of the union projected onto world XY.
    for(size_t i=0; i<points.size(); i++) {
      while(this->hull_.size() >= 2 && cross(this->hull_[this->hull_.size()-2],this->hull_.back(),points[i]) <= 1e-12) this->hull_.pop_back();
      this->hull_.push_back(points[i]);
    }
    const size_t lowerSize = this->hull_.size();
    for(size_t i=points.size(); i>1; i--) {
      const Eigen::Vector3d& p = points[i-2];
      while(this->hull_.size() > lowerSize && cross(this->hull_[this->hull_.size()-2],this->hull_.back(),p) <= 1e-12) this->hull_.pop_back();
      this->hull_.push_back(p);
    }
    if(!this->hull_.empty()) this->hull_.pop_back();
    if(this->hull_.size() < 3) throw std::runtime_error("SPConstraint projected support hull has no area");
    this->eq_.resize(0);
    this->margins_.resize(this->hull_.size());
    this->minIneq_.resize(this->hull_.size());
    this->maxIneq_ = Eigen::VectorXd::Constant(this->hull_.size(),std::numeric_limits<double>::infinity());
    for(size_t i=0; i<this->hull_.size(); i++) {
      const Eigen::Vector2d edge = (this->hull_[(i+1)%this->hull_.size()]-this->hull_[i]).head<2>();
      const Eigen::Vector2d normal(-edge.y()/edge.norm(),edge.x()/edge.norm());
      this->margins_[i] = normal.dot((this->COM_-this->hull_[i]).head<2>())-this->tolerance_;
      // Limit only the restoring step; never tighten already satisfied edges.
      this->minIneq_[i] = std::min(-this->margins_[i],this->maxError_);
    }
    this->valid_ = true;
    if(this->debugLevel_ > 0) std::cerr << "SPConstraint COM: " << this->COM_.transpose() << " margin: " << this->margin() << std::endl;
  }

  void SPConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    if(!this->valid_) throw std::logic_error("Call SPConstraint::updateBounds before updateJacobian");
    if(data.size() != this->models_.size()) throw std::invalid_argument("Invalid SPConstraint data vector");
    Eigen::Index dimension = 0;
    for(size_t i=0; i<this->models_.size(); i++) dimension += this->models_[i]->nv;
    Eigen::MatrixXd COMJacobian = Eigen::MatrixXd::Zero(3,dimension);
    Eigen::Index offset = 0;
    for(size_t i=0; i<this->models_.size(); i++) {
      for(size_t j=0; j<this->COMModels_.size(); j++) {
        if(this->models_[i] == this->COMModels_[j] && this->masses_[j] > 0.0) {
          if(!data[i]) throw std::invalid_argument("Null SPConstraint data");
          COMJacobian.middleCols(offset,this->models_[i]->nv) =
            this->masses_[j]/this->mass_*pinocchio::jacobianCenterOfMass(*this->models_[i],*data[i],false);
        }
      }
      offset += this->models_[i]->nv;
    }
    Eigen::MatrixXd jacobian(this->hull_.size(),dimension);
    for(size_t i=0; i<this->hull_.size(); i++) {
      const Eigen::Vector2d edge = (this->hull_[(i+1)%this->hull_.size()]-this->hull_[i]).head<2>();
      const Eigen::Vector2d normal(-edge.y()/edge.norm(),edge.x()/edge.norm());
      // Supports are treated as fixed during each linearization.
      jacobian.row(i) = normal.transpose()*COMJacobian.topRows(2);
    }
    this->jacobian_.resize(0,dimension);
    this->jacobianIneq_ = jacobian.sparseView();
    this->jacobianExt_.resize(0,0);
    this->jacobianIneqExt_.resize(this->hull_.size(),0);
  }

  bool SPConstraint::isSatisfied() const { return this->valid_ && this->distance() <= this->precision_; }
  double SPConstraint::distance() const {
    if(!this->valid_) return std::numeric_limits<double>::infinity();
    return this->margins_.cwiseMin(0.0).norm();
  }
  double SPConstraint::margin() const {
    return this->valid_ ? this->margins_.minCoeff() : -std::numeric_limits<double>::infinity();
  }

  void SPConstraint::drawOn(mujoco_viewer::Viewer& viewer) const {
    if(!this->valid_) return;
    const std::array<float,4> green{0.0f,1.0f,0.0f,1.0f};
    const std::array<float,4> cyan{0.0f,1.0f,1.0f,1.0f};
    const std::array<float,4> red{1.0f,0.1f,0.1f,1.0f};
    for(size_t i=0; i<this->worldPolygons_.size(); i++) {
      const std::vector<Eigen::Vector3d>& polygon = this->worldPolygons_[i];
      for(size_t j=0; j<polygon.size(); j++) mujoco_viewer::appendLine(viewer,point(polygon[j]),point(polygon[(j+1)%polygon.size()]),2.0,green);
    }
    double height = 0.0;
    for(size_t i=0; i<this->hull_.size(); i++) height += this->hull_[i].z()/this->hull_.size();
    for(size_t i=0; i<this->hull_.size(); i++) {
      Eigen::Vector3d a = this->hull_[i];
      Eigen::Vector3d b = this->hull_[(i+1)%this->hull_.size()];
      a.z() = height;
      b.z() = height;
      mujoco_viewer::appendLine(viewer,point(a),point(b),2.0,cyan);
    }
    const Eigen::Vector3d projected(this->COM_.x(),this->COM_.y(),height);
    mujoco_viewer::appendSphere(viewer,point(this->COM_),0.015,red);
    mujoco_viewer::appendSphere(viewer,point(projected),0.012,red);
    mujoco_viewer::appendLine(viewer,point(this->COM_),point(projected),1.0,red);
  }
}
