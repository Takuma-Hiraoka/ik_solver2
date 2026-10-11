#include <ik_constraint_scfr/SlideScfrConstraint.h>
#include <cmath>
#include <stdexcept>

namespace ik_constraint_scfr {
  void SlideScfrConstraint::setPositionConstraints(
    const std::vector<std::shared_ptr<ik_constraint::PositionConstraint> >& constraints) {
    for(std::size_t i = 0; i < constraints.size(); ++i) {
      if(!constraints[i] || constraints[i]->getAModel() != this->AModel_)
        throw std::invalid_argument("SlideScfrConstraint: contacts must belong to the COM model");
    }
    this->positionConstraints_ = constraints;
    this->initialPoses_.clear();
  }
  void SlideScfrConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    if(data.size() != 1 || !data[0]) throw std::invalid_argument("SlideScfrConstraint: invalid data");
    if(this->initialPoses_.empty()) {
      for(std::size_t i = 0; i < this->positionConstraints_.size(); ++i) {
        Eigen::Isometry3d pose;
        if(this->positionConstraints_[i]->getAJointId() >= this->AModel_->njoints)
          throw std::out_of_range("SlideScfrConstraint: invalid joint ID");
        pose.matrix() = data[0]->oMi[this->positionConstraints_[i]->getAJointId()].toHomogeneousMatrix() *
          this->positionConstraints_[i]->getALocalPose().toHomogeneousMatrix();
        this->initialPoses_.push_back(pose);
      }
    }
    std::vector<Eigen::Isometry3d> poses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
    std::vector<Eigen::VectorXd> bs;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
    std::vector<Eigen::VectorXd> dls;
    std::vector<Eigen::VectorXd> dus;
    for (int i=0;i<this->positionConstraints_.size();i++) {
      Eigen::Isometry3d currentPose;
      currentPose.matrix() = data[0]->oMi[this->positionConstraints_[i]->getAJointId()].toHomogeneousMatrix() *
        this->positionConstraints_[i]->getALocalPose().toHomogeneousMatrix();
      poses.push_back(currentPose);
      const Eigen::Vector3d transDiff = currentPose.translation() - this->initialPoses_[i].translation();
      const Eigen::Matrix3d rotDiff = this->initialPoses_[i].linear().transpose() * currentPose.linear();
      const double yaw = std::atan2(rotDiff(1,0), rotDiff(0,0));
      if (((std::abs(transDiff[0]) >= this->slideThreshold_[0]) || (std::abs(transDiff[1]) >= this->slideThreshold_[1])) &&
          (std::abs(yaw) >= this->slideThreshold_[5])) { // 並進回転滑り
        Eigen::SparseMatrix<double,Eigen::RowMajor> A(3,6);
        A.insert(0,0) = transDiff[0] > 0 ? -1.0 : 1.0; A.insert(0,2) = 0.2;
        A.insert(1,1) = transDiff[1] > 0 ? -1.0 : 1.0; A.insert(1,2) = 0.2;
        A.insert(2,5) = yaw > 0 ? -1.0 : 1.0; A.insert(2,2) = 0.005;
        As.push_back(A);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(3);
        bs.push_back(b);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(5,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,2) = 0.05; C.insert(1,3) = 1.0;
        C.insert(2,2) = 0.05; C.insert(2,3) = -1.0;
        C.insert(3,2) = 0.05; C.insert(3,4) = 1.0;
        C.insert(4,2) = 0.05; C.insert(4,4) = -1.0;
        Cs.push_back(C);
        Eigen::VectorXd dl = Eigen::VectorXd::Zero(5);
        dls.push_back(dl);
        Eigen::VectorXd du = 1e10 * Eigen::VectorXd::Ones(5);
        du[0] = 20000.0;
        dus.push_back(du);
      } else if ((std::abs(transDiff[0]) < this->slideThreshold_[0]) &&
          (std::abs(transDiff[1]) < this->slideThreshold_[1]) &&
          (std::abs(yaw) >= this->slideThreshold_[5])) { // 回転滑り
        Eigen::SparseMatrix<double,Eigen::RowMajor> A(1,6);
        A.insert(0,5) = yaw > 0 ? -1.0 : 1.0; A.insert(0,2) = 0.005;
        As.push_back(A);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(1);
        bs.push_back(b);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(9,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,2) = 0.2; C.insert(1,0) = 1.0;
        C.insert(2,2) = 0.2; C.insert(2,0) = -1.0;
        C.insert(3,2) = 0.2; C.insert(3,1) = 1.0;
        C.insert(4,2) = 0.2; C.insert(4,1) = -1.0;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        Cs.push_back(C);
        Eigen::VectorXd dl = Eigen::VectorXd::Zero(9);
        dls.push_back(dl);
        Eigen::VectorXd du = 1e10 * Eigen::VectorXd::Ones(9);
        du[0] = 20000.0;
        dus.push_back(du);
      } else { // 不動
        As.emplace_back(0,6);
        bs.emplace_back(0);
        Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6); // TODO 干渉形状から出す？
        C.insert(0,2) = 1.0;
        C.insert(1,2) = 0.2; C.insert(1,0) = 1.0;
        C.insert(2,2) = 0.2; C.insert(2,0) = -1.0;
        C.insert(3,2) = 0.2; C.insert(3,1) = 1.0;
        C.insert(4,2) = 0.2; C.insert(4,1) = -1.0;
        C.insert(5,2) = 0.05; C.insert(5,3) = 1.0;
        C.insert(6,2) = 0.05; C.insert(6,3) = -1.0;
        C.insert(7,2) = 0.05; C.insert(7,4) = 1.0;
        C.insert(8,2) = 0.05; C.insert(8,4) = -1.0;
        C.insert(9,2) = 0.05; C.insert(9,5) = 1.0;
        C.insert(10,2) = 0.005; C.insert(10,5) = -1.0;
        Cs.push_back(C);
        Eigen::VectorXd dl = Eigen::VectorXd::Zero(11);
        dls.push_back(dl);
        Eigen::VectorXd du = 1e10 * Eigen::VectorXd::Ones(11);
        du[0] = 20000.0;
        dus.push_back(du);
      }
    }
    // Always restore sticking conditions when the displacement falls below the threshold.
    this->jointIds_.clear();
    this->poses_ = poses; this->As_ = As; this->bs_ = bs;
    this->Cs_ = Cs; this->dls_ = dls; this->dus_ = dus;
    ScfrConstraint::updateBounds(data);
  }
  std::shared_ptr<ik_constraint::IKConstraint> SlideScfrConstraint::clone() const {
    std::shared_ptr<SlideScfrConstraint> result = std::make_shared<SlideScfrConstraint>(*this);
    result->positionConstraints_.clear();
    for(std::size_t i = 0; i < this->positionConstraints_.size(); ++i)
      result->positionConstraints_.push_back(std::static_pointer_cast<ik_constraint::PositionConstraint>(this->positionConstraints_[i]->clone()));
    return result;
  }
}
