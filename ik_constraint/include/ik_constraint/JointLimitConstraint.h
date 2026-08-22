#pragma once

#include <ik_constraint/IKConstraint.h>

#include <pinocchio/multibody/model.hpp>

#include <memory>

namespace ik_constraint{
  class JointLimitConstraint : public IKConstraint
  {
  public:
    explicit JointLimitConstraint(const std::shared_ptr<const pinocchio::Model>& model);
    std::shared_ptr<IKConstraint> clone() const override {
      return std::make_shared<JointLimitConstraint>(*this);
    }

    const std::shared_ptr<const pinocchio::Model>& getModel() const { return this->model_; }
    void setModel(const std::shared_ptr<const pinocchio::Model>& model);
    const pinocchio::JointIndex& getJointId() const { return this->jointId_; }
    void setJointId(const pinocchio::JointIndex& jointId) { this->jointId_ = jointId; }
    const double& getMaxError() const { return this->maxError_; }
    void setMaxError(const double& maxError) { this->maxError_ = maxError; }
    const double& getPrecision() const { return this->precision_; }
    void setPrecision(const double& precision) { this->precision_ = precision; }
    const double& getWeight() const { return this->weight_; }
    void setWeight(const double& weight) { this->weight_ = weight; }
    const double& getTolerance() const { return this->tolerance_; }
    void setTolerance(const double& tolerance) { this->tolerance_ = tolerance; }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;

  protected:
    std::shared_ptr<const pinocchio::Model> model_;
    pinocchio::JointIndex jointId_ = 0;
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    double tolerance_ = 1e-3;
    double currentLower_ = 0.0;
    double currentUpper_ = 0.0;

  private:
    void checkJoint() const;
  };
}
