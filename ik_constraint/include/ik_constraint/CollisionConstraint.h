#pragma once

#include <ik_constraint/IKConstraint.h>

#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>

#include <memory>

namespace ik_constraint{
  class CollisionConstraint : public IKConstraint
  {
  public:
    CollisionConstraint(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      pinocchio::JointIndex AJointId,
      const std::shared_ptr<const pinocchio::Model>& BModel,
      pinocchio::JointIndex BJointId);

    const std::shared_ptr<const pinocchio::Model>& getAModel() const { return this->AModel_; }
    void setAModel(const std::shared_ptr<const pinocchio::Model>& model);
    const pinocchio::JointIndex& getAJointId() const { return this->AJointId_; }
    void setAJointId(const pinocchio::JointIndex& jointId) { this->AJointId_ = jointId; }
    const std::shared_ptr<const pinocchio::Model>& getBModel() const { return this->BModel_; }
    void setBModel(const std::shared_ptr<const pinocchio::Model>& model);
    const pinocchio::JointIndex& getBJointId() const { return this->BJointId_; }
    void setBJointId(const pinocchio::JointIndex& jointId) { this->BJointId_ = jointId; }
    const double& getTolerance() const { return this->tolerance_; }
    void setTolerance(const double& tolerance) { this->tolerance_ = tolerance; }
    const double& getMaxError() const { return this->maxError_; }
    void setMaxError(const double& maxError) { this->maxError_ = maxError; }
    const double& getPrecision() const { return this->precision_; }
    void setPrecision(const double& precision) { this->precision_ = precision; }
    const double& getWeight() const { return this->weight_; }
    void setWeight(const double& weight) { this->weight_ = weight; }
    const double& getVelocityDamper() const { return this->velocityDamper_; }
    void setVelocityDamper(const double& velocityDamper) {
      this->velocityDamper_ = velocityDamper;
    }
    const double& getIgnoreDistance() const { return this->ignoreDistance_; }
    void setIgnoreDistance(const double& ignoreDistance) {
      this->ignoreDistance_ = ignoreDistance;
    }
    const bool& getInvert() const { return this->invert_; }
    void setInvert(const bool& invert) { this->invert_ = invert; }
    const Eigen::Vector3d& getACurrentLocalPosition() const {
      return this->ACurrentLocalPosition_;
    }
    const Eigen::Vector3d& getBCurrentLocalPosition() const {
      return this->BCurrentLocalPosition_;
    }
    const Eigen::Vector3d& getCurrentDirection() const {
      return this->currentDirection_;
    }
    const double& getCurrentDistance() const { return this->currentDistance_; }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    double margin() const override;
    void drawOn(mujoco_viewer::Viewer& viewer) const override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  protected:
    virtual bool computeDistance(
      const std::shared_ptr<const pinocchio::Model>& AModel,
      pinocchio::JointIndex AJointId,
      const std::shared_ptr<const pinocchio::Model>& BModel,
      pinocchio::JointIndex BJointId,
      double& distance,
      Eigen::Vector3d& direction,
      Eigen::Vector3d& ALocalPosition,
      Eigen::Vector3d& BLocalPosition) = 0;

    std::shared_ptr<const pinocchio::Model> AModel_;
    pinocchio::JointIndex AJointId_ = 0;
    std::shared_ptr<const pinocchio::Model> BModel_;
    pinocchio::JointIndex BJointId_ = 0;
    double tolerance_ = 0.01;
    double maxError_ = 0.05;
    double precision_ = 1e-3;
    double weight_ = 1.0;
    double velocityDamper_ = 1.0;
    double ignoreDistance_ = 0.1;
    bool invert_ = false;

    Eigen::Vector3d ACurrentLocalPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d BCurrentLocalPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ACurrentWorldPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d BCurrentWorldPosition_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d currentDirection_ = Eigen::Vector3d::UnitX();
    double currentDistance_ = 0.0;

  private:
    void checkJoints() const;
    size_t getConstraintModelIndex(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    Eigen::Index getConstraintVariableOffset(
      const std::shared_ptr<const pinocchio::Model>& model) const;
    Eigen::Vector3d getWorldPosition(
      const pinocchio::Data& data,
      pinocchio::JointIndex jointId,
      const Eigen::Vector3d& localPosition) const;
    Eigen::Matrix<double,3,Eigen::Dynamic> getPointJacobian(
      const std::shared_ptr<const pinocchio::Model>& model,
      pinocchio::Data& data,
      pinocchio::JointIndex jointId,
      const Eigen::Vector3d& localPosition) const;
    void updateModels();
  };
}
