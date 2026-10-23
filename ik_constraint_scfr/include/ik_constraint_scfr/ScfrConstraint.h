#pragma once
#include <ik_constraint/COMConstraint.h>
#include <scfr_solver/scfr_solver.h>

namespace ik_constraint_scfr {
  class ScfrConstraint : public ik_constraint::COMConstraint {
  public:
    explicit ScfrConstraint(const std::shared_ptr<const pinocchio::Model>& model);
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;
    const std::vector<pinocchio::JointIndex>& getJointIds() const { return this->jointIds_; }
    void setJointIds(const std::vector<pinocchio::JointIndex>& value) { this->jointIds_ = value; }
    const std::vector<Eigen::Isometry3d>& getPoses() const { return this->poses_; }
    void setPoses(const std::vector<Eigen::Isometry3d>& value) { this->poses_ = value; }
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& getAs() const { return this->As_; }
    void setAs(const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& value) { this->As_ = value; }
    const std::vector<Eigen::VectorXd>& getBs() const { return this->bs_; }
    void setBs(const std::vector<Eigen::VectorXd>& value) { this->bs_ = value; }
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& getCs() const { return this->Cs_; }
    void setCs(const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& value) { this->Cs_ = value; }
    const std::vector<Eigen::VectorXd>& getDls() const { return this->dls_; }
    void setDls(const std::vector<Eigen::VectorXd>& value) { this->dls_ = value; }
    const std::vector<Eigen::VectorXd>& getDus() const { return this->dus_; }
    void setDus(const std::vector<Eigen::VectorXd>& value) { this->dus_ = value; }
    const scfr_solver::SCFRParam& getSCFRParam() const { return this->SCFRParam_; }
    void setSCFRParam(const scfr_solver::SCFRParam& value) { this->SCFRParam_ = value; }
    const std::vector<Eigen::Vector2d>& getVertices() const { return this->vertices_; }
    bool hasRegion() const { return this->hasRegion_; }
    double getMass() const;
    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    double margin() const override;
  protected:
    // Empty jointIds means world poses; otherwise poses are joint-local.
    std::vector<pinocchio::JointIndex> jointIds_;
    std::vector<Eigen::Isometry3d> poses_;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As_, Cs_;
    std::vector<Eigen::VectorXd> bs_, dls_, dus_;
    scfr_solver::SCFRParam SCFRParam_;
    std::vector<Eigen::Vector2d> vertices_;
    bool hasRegion_ = false;
  };
}
