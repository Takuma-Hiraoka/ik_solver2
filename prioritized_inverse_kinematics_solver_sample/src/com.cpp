#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/COMConstraint.h>
#include <ik_constraint/PositionConstraint.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main()
{
  const std::string packageShare =
    ament_index_cpp::get_package_share_directory("prioritized_inverse_kinematics_solver_sample");
  const std::string robotDirectory = packageShare + "/unitree_ros/robots/g1_description";
  const std::string urdfPath = robotDirectory + "/g1_29dof.urdf";
  const std::string mujocoModelPath = robotDirectory + "/g1_29dof.xml";

  std::shared_ptr<pinocchio::Model> model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), *model);

  Eigen::VectorXd configuration = pinocchio::neutral(*model);
  configuration[2] = 0.65;
  configuration[7+0] = -0.6;
  configuration[7+3] = 1.4;
  configuration[7+4] = -0.8;
  configuration[7+6] = -0.6;
  configuration[7+9] = 1.4;
  configuration[7+10] = -0.8;

  const pinocchio::JointIndex leftFootJointId =
    model->getJointId("left_ankle_roll_joint");
  const pinocchio::JointIndex rightFootJointId =
    model->getJointId("right_ankle_roll_joint");
  if(leftFootJointId >= model->njoints || rightFootJointId >= model->njoints) {
    std::cerr << "ankle roll joints were not found in the Pinocchio model" << std::endl;
    return 1;
  }

  pinocchio::Data initialData(*model);
  pinocchio::forwardKinematics(*model, initialData, configuration);
  pinocchio::centerOfMass(*model, initialData, configuration);

  std::shared_ptr<ik_constraint::PositionConstraint> leftFootConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  leftFootConstraint->setAJointId(leftFootJointId);
  leftFootConstraint->setBJointId(0);
  leftFootConstraint->setBLocalPose(initialData.oMi[leftFootJointId]);
  leftFootConstraint->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
  leftFootConstraint->setPrecision(1e-4);

  std::shared_ptr<ik_constraint::PositionConstraint> rightFootConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  rightFootConstraint->setAJointId(rightFootJointId);
  rightFootConstraint->setBJointId(0);
  rightFootConstraint->setBLocalPose(initialData.oMi[rightFootJointId]);
  rightFootConstraint->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
  rightFootConstraint->setPrecision(1e-4);

  const Eigen::Vector3d targetCOM =
    initialData.com[0] + Eigen::Vector3d(0.0, 0.0, -0.20);
  std::shared_ptr<ik_constraint::COMConstraint> COMConstraint =
    std::make_shared<ik_constraint::COMConstraint>(model);
  COMConstraint->setBLocalPosition(targetCOM);
  COMConstraint->setMaxError(Eigen::Vector3d::Constant(0.03));
  COMConstraint->setPrecision(1e-3);

  std::vector<std::shared_ptr<pinocchio::Model> > models{model};
  std::vector<Eigen::VectorXd> configurations{configuration};
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > constraints(2);
  constraints[0].push_back(leftFootConstraint);
  constraints[0].push_back(rightFootConstraint);
  constraints[1].push_back(COMConstraint);
  std::vector<std::shared_ptr<prioritized_qp_base::Task> > previousTasks;

  std::shared_ptr<mujoco_viewer::Viewer> viewer =
    std::make_shared<mujoco_viewer::Viewer>();
  viewer->viewModel(mujocoModelPath);
  if(!viewer->model()) {
    std::cerr << "Failed to load MuJoCo model: " << mujocoModelPath << std::endl;
    return 1;
  }

  prioritized_inverse_kinematics_solver::IKParam parameter;
  parameter.maxIteration = 100;
  parameter.convergeThre = 1e-5;
  parameter.debugLevel = 3;
  parameter.viewer = viewer;
  parameter.viewMilliseconds = -1;

  const bool solved = prioritized_inverse_kinematics_solver::solveIKLoop(
    models, configurations, constraints, previousTasks, parameter);
  std::cout << "IK result: " << (solved ? "succeeded" : "failed") << std::endl;
  std::cout << "Target COM: " << targetCOM.transpose() << std::endl;

  while(viewer->isOpen()) {
    viewer->updateScene();
    leftFootConstraint->drawOn(*viewer);
    rightFootConstraint->drawOn(*viewer);
    COMConstraint->drawOn(*viewer);
    viewer->drawScene();
    viewer->pollEvents();
  }

  return solved ? 0 : 1;
}
