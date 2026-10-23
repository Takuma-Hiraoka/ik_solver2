#pragma once
#include <ik_constraint/CollisionConstraint.h>
#include <ik_constraint/ORConstraint.h>

namespace ik_constraint_scfr {
  class ORKeepCollisionConstraint : public ik_constraint::CollisionConstraint {
  public:
    explicit ORKeepCollisionConstraint(const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts);
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& getCollisionConstraints() const { return this->collisionConstraints_; }
    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    double margin() const override;
    void drawOn(mujoco_viewer::Viewer& viewer) const override;
  protected:
    bool computeDistance(const std::shared_ptr<const pinocchio::Model>& AModel,
      pinocchio::JointIndex AJointId, const std::shared_ptr<const pinocchio::Model>& BModel,
      pinocchio::JointIndex BJointId, double& distance, Eigen::Vector3d& direction,
      Eigen::Vector3d& ALocalPosition, Eigen::Vector3d& BLocalPosition) override;
  private:
    static const std::shared_ptr<ik_constraint::CollisionConstraint>& firstContact(
      const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts);
    static std::vector<std::shared_ptr<ik_constraint::IKConstraint> > makeChildren(
      const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts);
    std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> > collisionConstraints_;
    std::shared_ptr<ik_constraint::ORConstraint> selection_;
  };
}
