#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint_coal/CoalCollisionConstraint.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

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

static std::shared_ptr<ik_constraint_coal::CoalCollisionConstraint>
makeCollisionConstraint(
  const std::shared_ptr<pinocchio::Model>& model,
  const std::shared_ptr<pinocchio::GeometryModel>& geometryModel,
  const pinocchio::JointIndex& AJointId,
  const pinocchio::JointIndex& BJointId)
{
  std::shared_ptr<ik_constraint_coal::CoalCollisionConstraint> constraint =
    std::make_shared<ik_constraint_coal::CoalCollisionConstraint>(
      model, geometryModel, AJointId, model, geometryModel, BJointId);
  constraint->setIgnoreDistance(0.30);
  constraint->setPrecision(1e-3);
  return constraint;
}

int main()
{
  const std::string packageShare =
    ament_index_cpp::get_package_share_directory("prioritized_inverse_kinematics_solver_sample");
  const std::string robotDirectory = packageShare + "/unitree_ros/robots/g1_description";
  const std::string urdfPath = robotDirectory + "/g1_29dof.urdf";
  const std::string mujocoModelPath = robotDirectory + "/g1_29dof.xml";

  std::shared_ptr<pinocchio::Model> model = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(urdfPath, pinocchio::JointModelFreeFlyer(), *model);
  std::shared_ptr<pinocchio::GeometryModel> geometryModel =
    std::make_shared<pinocchio::GeometryModel>();
  pinocchio::urdf::buildGeom(
    *model, urdfPath, pinocchio::COLLISION, *geometryModel,
    std::vector<std::string>{robotDirectory});

  Eigen::VectorXd configuration = pinocchio::neutral(*model);
  configuration[2] = 0.65;
  configuration[7+0] = -0.6;
  configuration[7+3] = 1.4;
  configuration[7+4] = -0.8;
  configuration[7+6] = -0.6;
  configuration[7+9] = 1.4;
  configuration[7+10] = -0.8;

  const pinocchio::JointIndex rightHandJointId =
    model->getJointId("right_wrist_yaw_joint");
  const pinocchio::JointIndex leftHandJointId =
    model->getJointId("left_wrist_yaw_joint");
  const pinocchio::JointIndex rightFootJointId =
    model->getJointId("right_ankle_roll_joint");
  const pinocchio::JointIndex leftFootJointId =
    model->getJointId("left_ankle_roll_joint");
  if(rightHandJointId >= model->njoints || leftHandJointId >= model->njoints ||
     rightFootJointId >= model->njoints || leftFootJointId >= model->njoints) {
    std::cerr << "required G1 joints were not found in the Pinocchio model" << std::endl;
    return 1;
  }

  pinocchio::Data initialData(*model);
  pinocchio::forwardKinematics(*model, initialData, configuration);

  std::shared_ptr<ik_constraint::PositionConstraint> leftFootConstraint =
    makeFootConstraint(model, leftFootJointId, initialData.oMi[leftFootJointId]);
  std::shared_ptr<ik_constraint::PositionConstraint> rightFootConstraint =
    makeFootConstraint(model, rightFootJointId, initialData.oMi[rightFootJointId]);

  const pinocchio::SE3 handLocalPose(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.10, 0.0, 0.0));
  std::shared_ptr<ik_constraint::PositionConstraint> handTargetConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  handTargetConstraint->setAJointId(rightHandJointId);
  handTargetConstraint->setALocalPose(handLocalPose);
  handTargetConstraint->setBJointId(rightFootJointId);
  Eigen::Matrix<double,6,1> handTargetWeight = Eigen::Matrix<double,6,1>::Zero();
  handTargetWeight.head<3>().setOnes();
  handTargetConstraint->setWeight(handTargetWeight);
  handTargetConstraint->setMaxError(Eigen::Matrix<double,6,1>::Constant(0.03));
  handTargetConstraint->setPrecision(1e-3);

  const std::array<std::vector<std::string>,4> jointGroup{
    std::vector<std::string>{"left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint"},
    std::vector<std::string>{"right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"},
    std::vector<std::string>{"left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_pitch_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint"},
    std::vector<std::string>{"right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint", "right_wrist_pitch_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint"}
  };
  std::vector<std::shared_ptr<ik_constraint_coal::CoalCollisionConstraint> >
    collisionConstraints;
  for(size_t i=0; i<jointGroup.size(); i++) {
    for(size_t j=i+1; j<jointGroup.size(); j++) {
      for(size_t k=0; k<jointGroup[i].size(); k++) {
	for(size_t l=0; l<jointGroup[j].size(); l++) {
	  collisionConstraints.push_back(makeCollisionConstraint(model, geometryModel, model->getJointId(jointGroup[i][k]), model->getJointId(jointGroup[j][l])));
	}
      }
    }
  }

  std::vector<std::shared_ptr<pinocchio::Model> > models{model};
  std::vector<Eigen::VectorXd> configurations{configuration};
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > constraints(2);
  constraints[0].push_back(leftFootConstraint);
  constraints[0].push_back(rightFootConstraint);
  for(size_t i=0; i<collisionConstraints.size(); i++) {
    constraints[0].push_back(collisionConstraints[i]);
  }
  constraints[1].push_back(handTargetConstraint);
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
  parameter.debugLevel = 1;
  parameter.viewer = viewer;
  parameter.viewMilliseconds = -1;
  parameter.we = 1e2;

  const bool solved = prioritized_inverse_kinematics_solver::solveIKLoop(
    models, configurations, constraints, previousTasks, parameter);
  std::cout << "IK result: " << (solved ? "succeeded" : "low-priority target remains")
            << std::endl;

  while(viewer->isOpen()) {
    viewer->updateScene();
    leftFootConstraint->drawOn(*viewer);
    rightFootConstraint->drawOn(*viewer);
    handTargetConstraint->drawOn(*viewer);
    for(size_t i=0; i<collisionConstraints.size(); i++) {
      collisionConstraints[i]->drawOn(*viewer);
    }
    viewer->drawScene();
    viewer->pollEvents();
  }

  return 0;
}
