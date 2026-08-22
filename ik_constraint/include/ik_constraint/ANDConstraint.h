#pragma once

#include <ik_constraint/IKConstraint.h>

#include <memory>
#include <vector>

namespace ik_constraint{
  class ANDConstraint : public IKConstraint
  {
  public:
    explicit ANDConstraint(
      const std::vector<std::shared_ptr<IKConstraint> >& children);
    std::shared_ptr<IKConstraint> clone() const override;

    const std::vector<std::shared_ptr<IKConstraint> >& getChildren() const {
      return this->children_;
    }
    void setChildren(const std::vector<std::shared_ptr<IKConstraint> >& children);

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    double margin() const override;
    void drawOn(mujoco_viewer::Viewer& viewer) const override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    std::vector<std::shared_ptr<IKConstraint> > children_;

  private:
    static std::vector<std::shared_ptr<const pinocchio::Model> > collectModels(
      const std::vector<std::shared_ptr<IKConstraint> >& children);
    std::vector<const pinocchio::Data*> getChildConstData(
      const std::shared_ptr<IKConstraint>& child,
      const std::vector<const pinocchio::Data*>& data) const;
    std::vector<pinocchio::Data*> getChildData(
      const std::shared_ptr<IKConstraint>& child,
      const std::vector<pinocchio::Data*>& data) const;
    Eigen::SparseMatrix<double,Eigen::RowMajor> convertChildJacobian(
      const std::shared_ptr<IKConstraint>& child,
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& jacobian) const;
  };
}
