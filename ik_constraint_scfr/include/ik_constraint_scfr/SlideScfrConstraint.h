#pragma once
#include <ik_constraint_scfr/ScfrConstraint.h>
#include <ik_constraint/PositionConstraint.h>

namespace ik_constraint_scfr {
  class SlideScfrConstraint : public ScfrConstraint {
  public:
    explicit SlideScfrConstraint(const std::shared_ptr<const pinocchio::Model>& model) : ScfrConstraint(model) {}
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;
    const std::vector<std::shared_ptr<ik_constraint::PositionConstraint> >& getPositionConstraints() const { return this->positionConstraints_; }
    void setPositionConstraints(const std::vector<std::shared_ptr<ik_constraint::PositionConstraint> >& constraints);
    const Eigen::Matrix<double,6,1>& getSlideThreshold() const { return this->slideThreshold_; }
    void setSlideThreshold(const Eigen::Matrix<double,6,1>& value) { this->slideThreshold_ = value; }
    void resetInitialPoses() { this->initialPoses_.clear(); }
    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
  protected:
    std::vector<std::shared_ptr<ik_constraint::PositionConstraint> > positionConstraints_;
    std::vector<Eigen::Isometry3d> initialPoses_;
    Eigen::Matrix<double,6,1> slideThreshold_ =
      (Eigen::Matrix<double,6,1>() << 0.05, 0.05, 0.05, 0.2617993877991494, 0.2617993877991494, 0.2617993877991494).finished();
  };
}
