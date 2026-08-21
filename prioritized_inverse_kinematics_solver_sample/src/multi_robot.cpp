#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/PositionConstraint.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

static Eigen::VectorXd makeInitialConfiguration(
  const pinocchio::Model& model,
  const double& x,
  const double& yaw)
{
  Eigen::VectorXd configuration = pinocchio::neutral(model);
  configuration[0] = x;
  configuration[2] = 0.65;
  configuration[5] = std::sin(0.5 * yaw);
  configuration[6] = std::cos(0.5 * yaw);
  configuration[7+0] = -0.6;
  configuration[7+3] = 1.4;
  configuration[7+4] = -0.8;
  configuration[7+6] = -0.6;
  configuration[7+9] = 1.4;
  configuration[7+10] = -0.8;
  return configuration;
}

static std::shared_ptr<ik_constraint::PositionConstraint> makeFootConstraint(
  const std::shared_ptr<pinocchio::Model>& model,
  const pinocchio::JointIndex& jointId,
  const pinocchio::SE3& initialPose)
{
  std::shared_ptr<ik_constraint::PositionConstraint> constraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  constraint->setAJointId(jointId);
  constraint->setBJointId(0);
  constraint->setBLocalPose(initialPose);
  constraint->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
  constraint->setPrecision(1e-4);
  return constraint;
}

int main()
{
  const std::string packageShare =
    ament_index_cpp::get_package_share_directory("prioritized_inverse_kinematics_solver_sample");
  const std::string robotDirectory = packageShare + "/unitree_ros/robots/g1_description";
  const std::string urdfPath = robotDirectory + "/g1_29dof.urdf";
  const std::string mujocoModelPath = packageShare + "/config/g1_29dof_multi_robot.xml";

  std::shared_ptr<pinocchio::Model> model0 = std::make_shared<pinocchio::Model>();
  std::shared_ptr<pinocchio::Model> model1 = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), *model0);
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), *model1);

  const double pi = 3.14159265358979323846;
  Eigen::VectorXd configuration0 = makeInitialConfiguration(*model0, -0.5, 0.0);
  Eigen::VectorXd configuration1 = makeInitialConfiguration(*model1, 0.5, pi);

  const pinocchio::JointIndex leftFootJointId0 =
    model0->getJointId("left_ankle_roll_joint");
  const pinocchio::JointIndex rightFootJointId0 =
    model0->getJointId("right_ankle_roll_joint");
  const pinocchio::JointIndex rightHandJointId0 =
    model0->getJointId("right_wrist_yaw_joint");
  const pinocchio::JointIndex leftFootJointId1 =
    model1->getJointId("left_ankle_roll_joint");
  const pinocchio::JointIndex rightFootJointId1 =
    model1->getJointId("right_ankle_roll_joint");
  const pinocchio::JointIndex rightHandJointId1 =
    model1->getJointId("right_wrist_yaw_joint");
  if(leftFootJointId0 >= model0->njoints || rightFootJointId0 >= model0->njoints ||
     rightHandJointId0 >= model0->njoints || leftFootJointId1 >= model1->njoints ||
     rightFootJointId1 >= model1->njoints || rightHandJointId1 >= model1->njoints) {
    std::cerr << "required G1 joints were not found in the Pinocchio models" << std::endl;
    return 1;
  }

  pinocchio::Data initialData0(*model0);
  pinocchio::Data initialData1(*model1);
  pinocchio::forwardKinematics(*model0, initialData0, configuration0);
  pinocchio::forwardKinematics(*model1, initialData1, configuration1);

  std::shared_ptr<ik_constraint::PositionConstraint> leftFootConstraint0 =
    makeFootConstraint(model0, leftFootJointId0, initialData0.oMi[leftFootJointId0]);
  std::shared_ptr<ik_constraint::PositionConstraint> rightFootConstraint0 =
    makeFootConstraint(model0, rightFootJointId0, initialData0.oMi[rightFootJointId0]);
  std::shared_ptr<ik_constraint::PositionConstraint> leftFootConstraint1 =
    makeFootConstraint(model1, leftFootJointId1, initialData1.oMi[leftFootJointId1]);
  std::shared_ptr<ik_constraint::PositionConstraint> rightFootConstraint1 =
    makeFootConstraint(model1, rightFootJointId1, initialData1.oMi[rightFootJointId1]);

  const pinocchio::SE3 handLocalPose(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.10, 0.0, 0.0));
  std::shared_ptr<ik_constraint::PositionConstraint> handConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model0, model1);
  handConstraint->setAJointId(rightHandJointId0);
  handConstraint->setALocalPose(handLocalPose);
  handConstraint->setBJointId(rightHandJointId1);
  handConstraint->setBLocalPose(handLocalPose);
  Eigen::Matrix<double,6,1> handWeight = Eigen::Matrix<double,6,1>::Zero();
  handWeight.head<3>().setOnes();
  handConstraint->setWeight(handWeight);
  handConstraint->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
  handConstraint->setPrecision(1e-3);

  std::vector<std::shared_ptr<pinocchio::Model> > models{model0, model1};
  std::vector<Eigen::VectorXd> configurations{configuration0, configuration1};
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > constraints(2);
  constraints[0].push_back(leftFootConstraint0);
  constraints[0].push_back(rightFootConstraint0);
  constraints[0].push_back(leftFootConstraint1);
  constraints[0].push_back(rightFootConstraint1);
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

  while(viewer->isOpen()) {
    viewer->updateScene();
    leftFootConstraint0->drawOn(*viewer);
    rightFootConstraint0->drawOn(*viewer);
    leftFootConstraint1->drawOn(*viewer);
    rightFootConstraint1->drawOn(*viewer);
    handConstraint->drawOn(*viewer);
    viewer->drawScene();
    viewer->pollEvents();
  }

  return solved ? 0 : 1;
}
