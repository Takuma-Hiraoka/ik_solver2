#pragma once

#include <ik_constraint/IKConstraint.h>

#include <pinocchio/multibody/model.hpp>

#include <memory>

namespace ik_constraint{
  class JointAngleConstraint : public IKConstraint
  {
  public:
    explicit JointAngleConstraint(const std::shared_ptr<const pinocchio::Model>& AModel);
    JointAngleConstraint(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      const std::shared_ptr<const pinocchio::Model>& BModel);
    std::shared_ptr<IKConstraint> clone() const override {
      return std::make_shared<JointAngleConstraint>(*this);
    }

    const std::shared_ptr<const pinocchio::Model>& getAModel() const { return this->AModel_; }
    void setAModel(const std::shared_ptr<const pinocchio::Model>& model);
    const pinocchio::JointIndex& getAJointId() const { return this->AJointId_; }
    void setAJointId(const pinocchio::JointIndex& jointId) { this->AJointId_ = jointId; }
    const double& getAOffset() const { return this->AOffset_; }
    void setAOffset(const double& offset) { this->AOffset_ = offset; }
    const std::shared_ptr<const pinocchio::Model>& getBModel() const { return this->BModel_; }
    void setBModel(const std::shared_ptr<const pinocchio::Model>& model);
    const pinocchio::JointIndex& getBJointId() const { return this->BJointId_; }
    void setBJointId(const pinocchio::JointIndex& jointId) { this->BJointId_ = jointId; }
    const double& getBOffset() const { return this->BOffset_; }
    void setBOffset(const double& offset) { this->BOffset_ = offset; }
    const double& getMaxError() const { return this->maxError_; }
    void setMaxError(const double& maxError) { this->maxError_ = maxError; }
    const double& getPrecision() const { return this->precision_; }
    void setPrecision(const double& precision) { this->precision_ = precision; }
    const double& getWeight() const { return this->weight_; }
    void setWeight(const double& weight) { this->weight_ = weight; }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;

  protected:
    std::shared_ptr<const pinocchio::Model> AModel_;
    pinocchio::JointIndex AJointId_ = 0;
    double AOffset_ = 0.0;
    std::shared_ptr<const pinocchio::Model> BModel_ = nullptr;
    pinocchio::JointIndex BJointId_ = 0;
    double BOffset_ = 0.0;
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    double currentError_ = 0.0;

  private:
    size_t getConstraintModelIndex(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    Eigen::Index getConstraintVariableOffset(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    double getJointValue(
      const std::shared_ptr<const pinocchio::Model>& model,
      const pinocchio::Data& data,
      pinocchio::JointIndex jointId) const;
    void checkJoint(
      const std::shared_ptr<const pinocchio::Model>& model,
      pinocchio::JointIndex jointId) const;
    void updateModels();
  };
}
