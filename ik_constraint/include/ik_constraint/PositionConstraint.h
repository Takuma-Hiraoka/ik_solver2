#pragma once

#include <ik_constraint/IKConstraint.h>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <Eigen/Core>

#include <memory>

namespace ik_constraint{
  class PositionConstraint : public IKConstraint
  {
  public:
    explicit PositionConstraint(const std::shared_ptr<const pinocchio::Model>& model);
    PositionConstraint(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      const std::shared_ptr<const pinocchio::Model>& BModel);

    const std::shared_ptr<const pinocchio::Model>& getAModel() const { return this->AModel_; }
    void setAModel(const std::shared_ptr<const pinocchio::Model>& model);
    const std::shared_ptr<const pinocchio::Model>& getBModel() const { return this->BModel_; }
    void setBModel(const std::shared_ptr<const pinocchio::Model>& model);
    const pinocchio::JointIndex& getAJointId() const { return this->AJointId_; }
    void setAJointId(const pinocchio::JointIndex& jointId) { this->AJointId_ = jointId; }
    const pinocchio::SE3& getALocalPose() const { return this->ALocalPose_; }
    void setALocalPose(const pinocchio::SE3& localPose) { this->ALocalPose_ = localPose; }
    const pinocchio::JointIndex& getBJointId() const { return this->BJointId_; }
    void setBJointId(const pinocchio::JointIndex& jointId) { this->BJointId_ = jointId; }
    const pinocchio::SE3& getBLocalPose() const { return this->BLocalPose_; }
    void setBLocalPose(const pinocchio::SE3& localPose) { this->BLocalPose_ = localPose; }
    const Eigen::Matrix<double,6,1>& getMaxError() const { return this->maxError_; }
    void setMaxError(const Eigen::Matrix<double,6,1>& maxError) { this->maxError_ = maxError; }
    const double& getPrecision() const { return this->precision_; }
    void setPrecision(const double& precision) { this->precision_ = precision; }
    const Eigen::Matrix<double,6,1>& getWeight() const { return this->weight_; }
    void setWeight(const Eigen::Matrix<double,6,1>& weight) { this->weight_ = weight; }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    void drawOn(mujoco_viewer::Viewer& viewer) const override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    std::shared_ptr<const pinocchio::Model> AModel_;
    std::shared_ptr<const pinocchio::Model> BModel_;
    pinocchio::JointIndex AJointId_ = 0;
    pinocchio::SE3 ALocalPose_ = pinocchio::SE3::Identity();
    pinocchio::JointIndex BJointId_ = 0;
    pinocchio::SE3 BLocalPose_ = pinocchio::SE3::Identity();
    Eigen::Matrix<double,6,1> maxError_ = Eigen::Matrix<double,6,1>::Constant(0.05);
    double precision_ = 1e-3;
    Eigen::Matrix<double,6,1> weight_ = Eigen::Matrix<double,6,1>::Ones();
    Eigen::Matrix<double,6,1> currentError_ = Eigen::Matrix<double,6,1>::Zero();
    pinocchio::SE3 APosition_ = pinocchio::SE3::Identity();
    pinocchio::SE3 BPosition_ = pinocchio::SE3::Identity();

  private:
    pinocchio::SE3 getPlacement(
      const std::shared_ptr<const pinocchio::Model>& model,
      const pinocchio::Data& data,
      pinocchio::JointIndex jointId,
      const pinocchio::SE3& localPose) const;
    Eigen::Matrix<double,6,Eigen::Dynamic> getJointJacobian(
      const std::shared_ptr<const pinocchio::Model>& model,
      pinocchio::Data& data,
      pinocchio::JointIndex jointId,
      const pinocchio::SE3& localPose) const;
    size_t getConstraintModelIndex(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    Eigen::Index getConstraintVariableOffset(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    void updateModels();
  };
}
