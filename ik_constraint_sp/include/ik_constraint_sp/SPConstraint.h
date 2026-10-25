#pragma once

#include <ik_constraint/IKConstraint.h>

namespace ik_constraint_sp {
  struct SupportPolygon {
    // Null model denotes a fixed world pose (jointId must be zero).
    std::shared_ptr<const pinocchio::Model> model;
    pinocchio::JointIndex jointId = 0;
    pinocchio::SE3 localPose = pinocchio::SE3::Identity();
    // Clockwise vertices in the XY plane of localPose, in metres.
    std::vector<Eigen::Vector2d> vertices;
  };

  class SPConstraint : public ik_constraint::IKConstraint {
  public:
    SPConstraint() = default;
    explicit SPConstraint(const std::vector<std::shared_ptr<const pinocchio::Model> >& models);
    std::shared_ptr<ik_constraint::IKConstraint> clone() const override;

    // Models whose masses contribute to the combined COM. Support-only models
    // are additionally registered in IKConstraint::getModels() for data mapping.
    void setModels(const std::vector<std::shared_ptr<const pinocchio::Model> >& models);
    const std::vector<std::shared_ptr<const pinocchio::Model> >& getCOMModels() const { return this->COMModels_; }
    // Current world poses are read in updateBounds. The Jacobian treats the
    // supports as fixed; constrain supporting joints separately if necessary.
    void setSupportPolygons(const std::vector<SupportPolygon>& polygons);
    const std::vector<SupportPolygon>& getSupportPolygons() const { return this->polygons_; }
    void setTolerance(double value);
    double getTolerance() const { return this->tolerance_; }
    void setPrecision(double value);
    double getPrecision() const { return this->precision_; }
    void setMaxError(double value);
    double getMaxError() const { return this->maxError_; }
    const Eigen::Vector3d& getCOM() const { return this->COM_; }
    const std::vector<Eigen::Vector3d>& getHull() const { return this->hull_; }

    void updateBounds(const std::vector<const pinocchio::Data*>& data) override;
    void updateJacobian(const std::vector<pinocchio::Data*>& data) override;
    bool isSatisfied() const override;
    double distance() const override;
    double margin() const override;
    void drawOn(mujoco_viewer::Viewer& viewer) const override;

  protected:
    void updateModels();
    size_t modelIndex(const std::shared_ptr<const pinocchio::Model>& model) const;
    void invalidate();
    std::vector<std::shared_ptr<const pinocchio::Model> > COMModels_;
    std::vector<SupportPolygon> polygons_;
    std::vector<double> masses_;
    double mass_ = 0.0;
    double tolerance_ = 0.0;
    double precision_ = 1e-4;
    double maxError_ = 0.02;
    Eigen::Vector3d COM_ = Eigen::Vector3d::Zero();
    std::vector<std::vector<Eigen::Vector3d> > worldPolygons_;
    std::vector<Eigen::Vector3d> hull_;
    Eigen::VectorXd margins_;
    bool valid_ = false;
  };
}
