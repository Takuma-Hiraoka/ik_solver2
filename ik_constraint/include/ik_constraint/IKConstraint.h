#pragma once

#include <mujoco_viewer/mujoco_viewer.h>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Sparse>

#include <cstddef>
#include <memory>
#include <vector>

namespace ik_constraint{
  class IKConstraint
  {
  public:
    explicit IKConstraint(
      const std::vector<std::shared_ptr<const pinocchio::Model> >& models);
    virtual ~IKConstraint() = default;

    const std::vector<std::shared_ptr<const pinocchio::Model> >& getModels() const {
      return this->models_;
    }

    // boundsと、保持するModelに対するlocal Jacobianを派生classで更新する
    virtual void updateBounds(const std::vector<const pinocchio::Data*>& data) = 0;
    virtual void updateJacobian(const std::vector<pinocchio::Data*>& data) = 0;

    // local Jacobianをmodels全体のQP変数へ変換する
    void convertJacobian(const std::vector<std::shared_ptr<pinocchio::Model> >& models);

    virtual bool isSatisfied() const { return this->distance() == 0.0; }
    virtual double distance() const;
    virtual double margin() const;
    virtual void drawOn(mujoco_viewer::Viewer& viewer) const;

    const Eigen::VectorXd& getEq() const { return this->eq_; }
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobian() const { return this->jacobian_; }
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobianIneq() const { return this->jacobianIneq_; }
    const Eigen::VectorXd& getMinIneq() const { return this->minIneq_; }
    const Eigen::VectorXd& getMaxIneq() const { return this->maxIneq_; }
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobianExt() const { return this->jacobianExt_; }
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getJacobianIneqExt() const { return this->jacobianIneqExt_; }

    const int& debugLevel() const { return this->debugLevel_; }
    int& debugLevel() { return this->debugLevel_; }

    size_t getModelIndex(
      const std::shared_ptr<const pinocchio::Model>& model,
      const std::vector<std::shared_ptr<pinocchio::Model> >& models) const;
    size_t getVariableOffset(
      const std::shared_ptr<const pinocchio::Model>& model,
      const std::vector<std::shared_ptr<pinocchio::Model> >& models) const;
    static size_t getVariableDimension(const std::vector<std::shared_ptr<pinocchio::Model> >& models);
    static std::vector<std::shared_ptr<pinocchio::Data> > createDataVector(
      const std::vector<std::shared_ptr<pinocchio::Model> >& models);
    static pinocchio::Data& getData(
      const std::shared_ptr<const pinocchio::Model>& model,
      const std::vector<std::shared_ptr<pinocchio::Model> >& models,
      const std::vector<std::shared_ptr<pinocchio::Data> >& data);
    std::vector<const pinocchio::Data*> getConstDataVector(
      const std::vector<std::shared_ptr<pinocchio::Model> >& models,
      const std::vector<std::shared_ptr<pinocchio::Data> >& data) const;
    std::vector<pinocchio::Data*> getDataVector(
      const std::vector<std::shared_ptr<pinocchio::Model> >& models,
      const std::vector<std::shared_ptr<pinocchio::Data> >& data) const;

    static double clamp(const double& value, const double& limitValue);
    template<typename Derived>
    static typename Derived::PlainObject clamp(
      const Eigen::MatrixBase<Derived>& value,
      const Eigen::MatrixBase<Derived>& limitValue) {
      return value.array().max(-limitValue.array()).min(limitValue.array());
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    void setModels(const std::vector<std::shared_ptr<const pinocchio::Model> >& models);

    std::vector<std::shared_ptr<const pinocchio::Model> > models_;
    int debugLevel_ = 0;

    Eigen::VectorXd eq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    Eigen::VectorXd minIneq_;
    Eigen::VectorXd maxIneq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneq_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianExt_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianIneqExt_;
  };
}
