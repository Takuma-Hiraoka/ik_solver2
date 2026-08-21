#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/PositionConstraint.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

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
  configuration[2] = 0.80;

  const pinocchio::JointIndex rootJointId = 1;
  const pinocchio::JointIndex rightWristJointId =
    model->getJointId("right_wrist_yaw_joint");
  if(rightWristJointId >= model->njoints) {
    std::cerr << "right_wrist_yaw_joint was not found in the Pinocchio model" << std::endl;
    return 1;
  }

  const pinocchio::SE3 handLocalPose(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.10, 0.0, 0.0));
  pinocchio::Data initialData(*model);
  pinocchio::forwardKinematics(*model, initialData, configuration);
  const pinocchio::SE3 initialHandPose =
    initialData.oMi[rightWristJointId] * handLocalPose;
  const Eigen::Vector3d targetPosition =
    initialHandPose.translation() + Eigen::Vector3d(0.15, -0.05, 0.10);

  std::shared_ptr<ik_constraint::PositionConstraint> baseConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  baseConstraint->setAJointId(rootJointId);
  baseConstraint->setBJointId(0);
  baseConstraint->setBLocalPose(initialData.oMi[rootJointId]);
  baseConstraint->setPrecision(1e-4);

  std::shared_ptr<ik_constraint::PositionConstraint> handConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  handConstraint->setAJointId(rightWristJointId);
  handConstraint->setALocalPose(handLocalPose);
  handConstraint->setBJointId(0);
  handConstraint->setBLocalPose(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), targetPosition));
  Eigen::Matrix<double,6,1> handWeight = Eigen::Matrix<double,6,1>::Zero();
  handWeight.head<3>().setOnes();
  handConstraint->setWeight(handWeight);
  handConstraint->setPrecision(1e-3);

  std::vector<std::shared_ptr<pinocchio::Model> > models{model};
  std::vector<Eigen::VectorXd> configurations{configuration};
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > constraints(2);
  constraints[0].push_back(baseConstraint);
  constraints[1].push_back(handConstraint);
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
  std::cout << "Target position: " << targetPosition.transpose() << std::endl;

  while(viewer->isOpen()) {
    viewer->updateScene();
    baseConstraint->drawOn(*viewer);
    handConstraint->drawOn(*viewer);
    viewer->drawScene();
    viewer->pollEvents();
  }

  return solved ? 0 : 1;
}
