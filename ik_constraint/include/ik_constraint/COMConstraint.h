#pragma once

#include <ik_constraint/IKConstraint.h>

#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>
#include <Eigen/Sparse>

#include <memory>

namespace ik_constraint{
  class COMConstraint : public IKConstraint
  {
  public:
    explicit COMConstraint(const std::shared_ptr<const pinocchio::Model>& AModel);
    COMConstraint(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      const std::shared_ptr<const pinocchio::Model>& BModel);

    const std::shared_ptr<const pinocchio::Model>& getAModel() const { return this->AModel_; }
    void setAModel(const std::shared_ptr<const pinocchio::Model>& model);
    const Eigen::Vector3d& getALocalPosition() const { return this->ALocalPosition_; }
    void setALocalPosition(const Eigen::Vector3d& position) { this->ALocalPosition_ = position; }
    const std::shared_ptr<const pinocchio::Model>& getBModel() const { return this->BModel_; }
    void setBModel(const std::shared_ptr<const pinocchio::Model>& model);
    const Eigen::Vector3d& getBLocalPosition() const { return this->BLocalPosition_; }
    void setBLocalPosition(const Eigen::Vector3d& position) { this->BLocalPosition_ = position; }
    const Eigen::Matrix3d& getEvalRotation() const { return this->evalRotation_; }
    void setEvalRotation(const Eigen::Matrix3d& rotation) { this->evalRotation_ = rotation; }

    const Eigen::Vector3d& getMaxError() const { return this->maxError_; }
    void setMaxError(const Eigen::Vector3d& maxError) { this->maxError_ = maxError; }
    const double& getPrecision() const { return this->precision_; }
    void setPrecision(const double& precision) { this->precision_ = precision; }
    const Eigen::Vector3d& getWeight() const { return this->weight_; }
    void setWeight(const Eigen::Vector3d& weight) { this->weight_ = weight; }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& getC() const { return this->C_; }
    void setC(const Eigen::SparseMatrix<double,Eigen::RowMajor>& C) { this->C_ = C; }
    const Eigen::VectorXd& getDl() const { return this->dl_; }
    void setDl(const Eigen::VectorXd& dl) { this->dl_ = dl; }
    const Eigen::VectorXd& getDu() const { return this->du_; }
    void setDu(const Eigen::VectorXd& du) { this->du_ = du; }
    const Eigen::VectorXd& getMaxCErrorVector() const { return this->maxCErrorVector_; }
    void setMaxCErrorVector(const Eigen::VectorXd& maxCErrorVector) {
      this->maxCErrorVector_ = maxCErrorVector;
    }
    const double& getMaxCError() const { return this->maxCError_; }
    void setMaxCError(const double& maxCError) { this->maxCError_ = maxCError; }
    const double& getCPrecision() const { return this->CPrecision_; }
    void setCPrecision(const double& precision) { this->CPrecision_ = precision; }
    const double& getCTolerance() const { return this->CTolerance_; }
    void setCTolerance(const double& tolerance) { this->CTolerance_ = tolerance; }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    void drawOn(mujoco_viewer::Viewer& viewer) const override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    std::shared_ptr<const pinocchio::Model> AModel_;
    Eigen::Vector3d ALocalPosition_ = Eigen::Vector3d::Zero();
    std::shared_ptr<const pinocchio::Model> BModel_ = nullptr;
    Eigen::Vector3d BLocalPosition_ = Eigen::Vector3d::Zero();
    Eigen::Matrix3d evalRotation_ = Eigen::Matrix3d::Identity();

    Eigen::Vector3d maxError_ = Eigen::Vector3d::Constant(0.05);
    double precision_ = 1e-3;
    Eigen::Vector3d weight_ = Eigen::Vector3d::Ones();

    Eigen::SparseMatrix<double,Eigen::RowMajor> C_{0,3};
    Eigen::VectorXd dl_;
    Eigen::VectorXd du_;
    Eigen::VectorXd maxCErrorVector_;
    double maxCError_ = 0.05;
    double CPrecision_ = 1e-3;
    double CTolerance_ = 0.0;

    Eigen::Vector3d APosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d BPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d currentErrorEval_ = Eigen::Vector3d::Zero();
    Eigen::VectorXd currentUpper_;
    Eigen::VectorXd currentLower_;

  private:
    size_t getConstraintModelIndex(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    Eigen::Index getConstraintVariableOffset(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    void updateModels();
  };
}
