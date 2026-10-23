#pragma once
#include <ik_constraint_scfr/ScfrConstraint.h>
#include <ik_constraint/ANDConstraint.h>
#include <ik_constraint/CollisionConstraint.h>

namespace ik_constraint_scfr {
  class KeepCollisionScfrConstraint : public ik_constraint::ANDConstraint {
  public:
    KeepCollisionScfrConstraint(
      const std::shared_ptr<ScfrConstraint>& scfr,
      const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts);
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;
    const std::shared_ptr<ScfrConstraint>& getScfrConstraint() const { return this->scfrConstraint_; }
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& getKeepCollisionConstraints() const { return this->keepCollisionConstraints_; }
    const scfr_solver::SCFRParam& getBreakableSCFRParam() const { return this->breakableSCFRParam_; }
    void setBreakableSCFRParam(const scfr_solver::SCFRParam& value) { this->breakableSCFRParam_ = value; }
    const std::size_t& getMinimumContactCount() const { return this->minimumContactCount_; }
    void setMinimumContactCount(const std::size_t& value) { this->minimumContactCount_ = value; }
    int getBreakableIndex() const { return this->breakableIndex_; }
    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
  protected:
    std::shared_ptr<ScfrConstraint> scfrConstraint_;
    std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> > keepCollisionConstraints_;
    scfr_solver::SCFRParam breakableSCFRParam_;
    std::size_t minimumContactCount_ = 2;
    int breakableIndex_ = -1;
  private:
    static std::vector<std::shared_ptr<ik_constraint::IKConstraint> > makeChildren(
      const std::shared_ptr<ScfrConstraint>& scfr,
      const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts);
  };
  bool checkSCFRExistance(std::vector<Eigen::Isometry3d>& poses,
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As,
    const std::vector<Eigen::VectorXd>& bs,
    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs,
    const std::vector<Eigen::VectorXd>& dls,
    const std::vector<Eigen::VectorXd>& dus,
    const double& mass, const scfr_solver::SCFRParam& param = scfr_solver::SCFRParam());
}
