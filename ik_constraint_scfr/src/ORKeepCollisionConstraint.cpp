#include <ik_constraint_scfr/ORKeepCollisionConstraint.h>
#include <stdexcept>
#include <limits>

namespace ik_constraint_scfr {
  const std::shared_ptr<ik_constraint::CollisionConstraint>& ORKeepCollisionConstraint::firstContact(
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts) {
    if(contacts.empty() || !contacts[0]) throw std::invalid_argument("ORKeepCollisionConstraint requires contacts");
    return contacts[0];
  }
  std::vector<std::shared_ptr<ik_constraint::IKConstraint> > ORKeepCollisionConstraint::makeChildren(
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts) {
    std::vector<std::shared_ptr<ik_constraint::IKConstraint> > children;
    for(std::size_t i = 0; i < contacts.size(); ++i) {
      if(!contacts[i]) throw std::invalid_argument("ORKeepCollisionConstraint: null contact");
      children.push_back(contacts[i]);
    }
    return children;
  }
  ORKeepCollisionConstraint::ORKeepCollisionConstraint(
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts)
    : ik_constraint::CollisionConstraint(firstContact(contacts)->getAModel(), firstContact(contacts)->getAJointId(),
        firstContact(contacts)->getBModel(), firstContact(contacts)->getBJointId()),
      collisionConstraints_(contacts), selection_(std::make_shared<ik_constraint::ORConstraint>(makeChildren(contacts))) {
    this->setModels(this->selection_->getModels());
  }
  bool ORKeepCollisionConstraint::computeDistance(const std::shared_ptr<const pinocchio::Model>&,
    pinocchio::JointIndex, const std::shared_ptr<const pinocchio::Model>&, pinocchio::JointIndex,
    double&, Eigen::Vector3d&, Eigen::Vector3d&, Eigen::Vector3d&) {
    throw std::logic_error("ORKeepCollisionConstraint delegates distance to its children");
  }
  void ORKeepCollisionConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    this->selection_->updateBounds(data);
    this->eq_ = this->selection_->getEq(); this->minIneq_ = this->selection_->getMinIneq();
    this->maxIneq_ = this->selection_->getMaxIneq();
    double maximumMargin = -std::numeric_limits<double>::infinity();
    std::size_t selected = 0;
    for(std::size_t i = 0; i < this->collisionConstraints_.size(); ++i) {
      if(this->collisionConstraints_[i]->margin() > maximumMargin) {
        maximumMargin = this->collisionConstraints_[i]->margin(); selected = i;
      }
    }
    const std::shared_ptr<ik_constraint::CollisionConstraint>& contact = this->collisionConstraints_[selected];
    this->AModel_ = contact->getAModel(); this->AJointId_ = contact->getAJointId();
    this->BModel_ = contact->getBModel(); this->BJointId_ = contact->getBJointId();
    this->ACurrentLocalPosition_ = contact->getACurrentLocalPosition();
    this->BCurrentLocalPosition_ = contact->getBCurrentLocalPosition();
    this->currentDirection_ = contact->getCurrentDirection(); this->currentDistance_ = contact->getCurrentDistance();
  }
  void ORKeepCollisionConstraint::updateJacobian(const std::vector<pinocchio::Data*>& data) {
    this->selection_->updateJacobian(data);
    this->jacobian_ = this->selection_->getJacobian(); this->jacobianIneq_ = this->selection_->getJacobianIneq();
    this->jacobianExt_ = this->selection_->getJacobianExt(); this->jacobianIneqExt_ = this->selection_->getJacobianIneqExt();
  }
  bool ORKeepCollisionConstraint::isSatisfied() const { return this->selection_->isSatisfied(); }
  double ORKeepCollisionConstraint::distance() const { return this->selection_->distance(); }
  double ORKeepCollisionConstraint::margin() const { return this->selection_->margin(); }
  void ORKeepCollisionConstraint::drawOn(mujoco_viewer::Viewer& viewer) const { this->selection_->drawOn(viewer); }
  std::shared_ptr<ik_constraint::IKConstraint> ORKeepCollisionConstraint::clone() const {
    std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> > contacts;
    for(std::size_t i = 0; i < this->collisionConstraints_.size(); ++i)
      contacts.push_back(std::static_pointer_cast<ik_constraint::CollisionConstraint>(this->collisionConstraints_[i]->clone()));
    std::shared_ptr<ORKeepCollisionConstraint> result = std::make_shared<ORKeepCollisionConstraint>(contacts);
    result->debugLevel_ = this->debugLevel_;
    return result;
  }
}
