#include <ik_constraint_scfr/ScfrConstraint.h>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <algorithm>

namespace ik_constraint_scfr {
  ScfrConstraint::ScfrConstraint(const std::shared_ptr<const pinocchio::Model>& model)
    : ik_constraint::COMConstraint(model) { this->weight_.setZero(); }

  std::shared_ptr<ik_constraint::IKConstraint> ScfrConstraint::clone() const {
    return std::make_shared<ScfrConstraint>(*this);
  }

  double ScfrConstraint::getMass() const {
    double mass = 0.0;
    // Include bodies merged into universe when the root or joints are fixed.
    for(pinocchio::JointIndex id = 0; id < this->AModel_->njoints; ++id)
      mass += this->AModel_->inertias[id].mass();
    return mass;
  }

  void ScfrConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != this->models_.size()) throw std::invalid_argument("ScfrConstraint: invalid data vector");
    if(this->BModel_ || !this->evalRotation_.isIdentity() || !this->BLocalPosition_.isZero())
      throw std::invalid_argument("ScfrConstraint requires world-frame COM evaluation");
    if(!this->jointIds_.empty() && this->jointIds_.size() != this->poses_.size())
      throw std::invalid_argument("ScfrConstraint: jointIds and poses differ in size");
    const pinocchio::Data* modelData = nullptr;
    for(std::size_t i = 0; i < this->models_.size(); ++i)
      if(this->models_[i] == this->AModel_) modelData = data[i];
    if(!modelData) throw std::invalid_argument("ScfrConstraint: missing model data");
    std::vector<Eigen::Isometry3d> poses = this->poses_;
    for(std::size_t i = 0; i < this->jointIds_.size(); ++i) {
      if(this->jointIds_[i] >= this->AModel_->njoints)
        throw std::out_of_range("ScfrConstraint: invalid joint ID");
      poses[i].matrix() = modelData->oMi[this->jointIds_[i]].toHomogeneousMatrix() * this->poses_[i].matrix();
    }
    if(poses.size() != this->As_.size() || poses.size() != this->bs_.size() ||
       poses.size() != this->Cs_.size() || poses.size() != this->dls_.size() || poses.size() != this->dus_.size())
      throw std::invalid_argument("ScfrConstraint: contact arrays differ in size");
    for(std::size_t i = 0; i < poses.size(); ++i)
      if(this->As_[i].cols() != 6 || this->Cs_[i].cols() != 6 ||
         this->As_[i].rows() != this->bs_[i].size() || this->Cs_[i].rows() != this->dls_[i].size() ||
         this->Cs_[i].rows() != this->dus_[i].size())
        throw std::invalid_argument("ScfrConstraint: invalid wrench dimensions");
    Eigen::SparseMatrix<double,Eigen::RowMajor> M(0,2);
    Eigen::VectorXd lower, upper;
    this->vertices_.clear();
    this->hasRegion_ = !poses.empty() && this->getMass() > 0.0 &&
      scfr_solver::calcSCFR(poses, this->As_, this->bs_, this->Cs_, this->dls_, this->dus_,
                            this->getMass(), M, lower, upper, this->vertices_, this->SCFRParam_) && M.rows() > 0;
    if(this->hasRegion_) {
      Eigen::SparseMatrix<double,Eigen::ColMajor> C(M.rows(),3);
      C.leftCols(2) = M;
      this->C_ = C;
      this->dl_ = lower;
      this->du_ = upper;
    } else {
      // An impossible inequality prevents empty or infeasible contacts being treated as satisfied.
      this->C_.resize(1,3); this->C_.setZero();
      this->dl_ = Eigen::VectorXd::Ones(1);
      this->du_ = Eigen::VectorXd::Ones(1);
      this->vertices_.clear();
    }
    const double fixedMass = this->AModel_->inertias[0].mass();
    if(fixedMass > 0.0) {
      pinocchio::Data comData = *modelData;
      const double movingMass = this->getMass() - fixedMass;
      const Eigen::Vector3d movingMoment = movingMass > 0.0 ?
        Eigen::Vector3d(movingMass * modelData->com[0]) : Eigen::Vector3d::Zero();
      comData.com[0] = (movingMoment + fixedMass * this->AModel_->inertias[0].lever()) / this->getMass();
      ik_constraint::COMConstraint::updateBounds(std::vector<const pinocchio::Data*>{&comData});
    } else {
      ik_constraint::COMConstraint::updateBounds(data);
    }
  }

  void ScfrConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    if(this->getMass() <= 0.0) throw std::invalid_argument("ScfrConstraint requires positive mass");
    ik_constraint::COMConstraint::updateJacobian(data);
    const double movingMassRatio = (this->getMass() - this->AModel_->inertias[0].mass()) / this->getMass();
    this->jacobian_ *= movingMassRatio;
    this->jacobianIneq_ *= movingMassRatio;
  }

  bool ScfrConstraint::isSatisfied() const {
    if(!this->hasRegion_) return false;
    return this->currentLower_.cwiseMax(0.0).squaredNorm() +
      this->currentUpper_.cwiseMin(0.0).squaredNorm() <= this->CPrecision_ * this->CPrecision_;
  }
  double ScfrConstraint::distance() const {
    return this->hasRegion_ ? ik_constraint::COMConstraint::distance() : std::numeric_limits<double>::infinity();
  }
  double ScfrConstraint::margin() const {
    if(!this->hasRegion_) return -std::numeric_limits<double>::infinity();
    return std::min((-this->currentLower_).minCoeff(), this->currentUpper_.minCoeff());
  }
}
