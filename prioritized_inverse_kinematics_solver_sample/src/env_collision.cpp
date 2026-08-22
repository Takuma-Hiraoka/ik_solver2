#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint_esdf/EsdfCollisionConstraint.h>
#include <mujoco_esdf/mujoco_esdf.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>

#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>

static void setGeometryPose(mjsGeom* geometry, const Eigen::Vector3d& position)
{
  geometry->pos[0] = position.x();
  geometry->pos[1] = position.y();
  geometry->pos[2] = position.z();
  geometry->quat[0] = 1.0;
  geometry->quat[1] = 0.0;
  geometry->quat[2] = 0.0;
  geometry->quat[3] = 0.0;
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
  const pinocchio::JointIndex rootJointId = 1;

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

  const pinocchio::SE3 handLocalPose(
    Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.10, 0.0, 0.0));
  pinocchio::Data initialData(*model);
  pinocchio::forwardKinematics(*model, initialData, configuration);
  const Eigen::Vector3d initialHandPosition =
    (initialData.oMi[rightHandJointId] * handLocalPose).translation();
  const Eigen::Vector3d targetPosition =
    initialHandPosition + Eigen::Vector3d(0.0, 0.0, -0.3);

  char error[1024] = {0};
  std::shared_ptr<mjSpec> specification(
    mj_parseXML(mujocoModelPath.c_str(), nullptr, error, sizeof(error)), mj_deleteSpec);
  if(!specification) {
    std::cerr << "Failed to parse MuJoCo model: " << error << std::endl;
    return 1;
  }
  mjsBody* worldBody = mjs_findBody(specification.get(), "world");
  if(worldBody == nullptr) {
    std::cerr << "MuJoCo world body was not found" << std::endl;
    return 1;
  }

  mjsGeom* box = mjs_addGeom(worldBody, nullptr);
  box->type = mjGEOM_BOX;
  setGeometryPose(box, Eigen::Vector3d(0.4, -0.1, 0.2));
  box->size[0] = 0.065;
  box->size[1] = 0.065;
  box->size[2] = 0.40;
  box->rgba[0] = 0.85f;
  box->rgba[1] = 0.25f;
  box->rgba[2] = 0.15f;
  box->rgba[3] = 1.0f;

  mjsGeom* cylinder = mjs_addGeom(worldBody, nullptr);
  cylinder->type = mjGEOM_CYLINDER;
  setGeometryPose(cylinder, Eigen::Vector3d(0.4, -0.1, 0.65));
  cylinder->size[0] = 0.2;
  cylinder->size[1] = 0.05;
  cylinder->rgba[0] = 0.20f;
  cylinder->rgba[1] = 0.45f;
  cylinder->rgba[2] = 0.90f;
  cylinder->rgba[3] = 1.0f;

  std::vector<const mjsGeom*> obstacleGeometries{box, cylinder};
  const Eigen::Vector3d mapMinimum(-1.0, -1.0, 0.0);
  const Eigen::Vector3d mapMaximum(1.4, 1.0, 1.8);
  std::shared_ptr<voxblox::EsdfMap> esdfMap = mujoco_esdf::createEsdfMap(
    obstacleGeometries, mapMinimum, mapMaximum, 0.03, 16);

  std::shared_ptr<mjModel> mujocoModel(
    mj_compile(specification.get(), nullptr), mj_deleteModel);
  if(!mujocoModel) {
    std::cerr << "Failed to compile MuJoCo model: " << mjs_getError(specification.get()) << std::endl;
    return 1;
  }

  std::shared_ptr<ik_constraint::PositionConstraint> leftFootConstraint =
    makeFootConstraint(model, leftFootJointId, initialData.oMi[leftFootJointId]);
  std::shared_ptr<ik_constraint::PositionConstraint> rightFootConstraint =
    makeFootConstraint(model, rightFootJointId, initialData.oMi[rightFootJointId]);

  std::shared_ptr<ik_constraint::PositionConstraint> handConstraint =
    std::make_shared<ik_constraint::PositionConstraint>(model);
  handConstraint->setAJointId(rightHandJointId);
  handConstraint->setALocalPose(handLocalPose);
  handConstraint->setBJointId(0);
  handConstraint->setBLocalPose(
    pinocchio::SE3(Eigen::Matrix3d::Identity(), targetPosition));
  Eigen::Matrix<double,6,1> handWeight = Eigen::Matrix<double,6,1>::Zero();
  handWeight.head<3>().setOnes();
  handConstraint->setWeight(handWeight);
  handConstraint->setPrecision(1e-3);

  std::set<pinocchio::JointIndex> collisionJointIds;
  for(std::size_t geometryIndex = 0;
      geometryIndex < geometryModel->geometryObjects.size(); ++geometryIndex) {
    const pinocchio::JointIndex jointId =
      geometryModel->geometryObjects[geometryIndex].parentJoint;
    if(jointId > 0 && jointId < model->njoints) {
      collisionJointIds.insert(jointId);
    }
  }
  std::vector<std::shared_ptr<ik_constraint_esdf::EsdfCollisionConstraint> >
    collisionConstraints;
  for(std::set<pinocchio::JointIndex>::const_iterator jointIterator =
      collisionJointIds.begin(); jointIterator != collisionJointIds.end(); ++jointIterator) {
    const pinocchio::JointIndex jointId = *jointIterator;
    std::shared_ptr<ik_constraint_esdf::EsdfCollisionConstraint> constraint =
      std::make_shared<ik_constraint_esdf::EsdfCollisionConstraint>(
      model, geometryModel, jointId);
    constraint->setField(esdfMap);
    constraint->setResolution(0.04);
    constraint->setMinDistance(-0.03);
    constraint->setMaxDistance(0.50);
    constraint->setTolerance(0.05);
    constraint->setIgnoreDistance(0.50);
    constraint->setPrecision(1e-3);
    collisionConstraints.push_back(constraint);
  }

  std::vector<std::shared_ptr<pinocchio::Model> > models{model};
  std::vector<Eigen::VectorXd> configurations{configuration};
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > constraints(2);
  constraints[0].push_back(leftFootConstraint);
  constraints[0].push_back(rightFootConstraint);
  for(std::size_t index = 0; index < collisionConstraints.size(); ++index) {
    constraints[0].push_back(collisionConstraints[index]);
  }
  constraints[1].push_back(handConstraint);
  std::vector<std::shared_ptr<prioritized_qp_base::Task> > previousTasks;

  std::shared_ptr<mujoco_viewer::Viewer> viewer =
    std::make_shared<mujoco_viewer::Viewer>();
  viewer->viewModel(mujocoModel);
  if(!viewer->model()) {
    return 1;
  }

  prioritized_inverse_kinematics_solver::IKParam parameter;
  parameter.maxIteration = 100;
  parameter.convergeThre = 1e-5;
  parameter.debugLevel = 3;
  parameter.viewer = viewer;
  parameter.viewMilliseconds = 20;
  parameter.we = 1e2;

  const bool solved = prioritized_inverse_kinematics_solver::solveIKLoop(
    models, configurations, constraints, previousTasks, parameter);
  std::cout << "IK result: " << (solved ? "succeeded" : "low-priority target remains")
            << std::endl;

  while(viewer->isOpen()) {
    viewer->updateScene();
    leftFootConstraint->drawOn(*viewer);
    rightFootConstraint->drawOn(*viewer);
    handConstraint->drawOn(*viewer);
    for(std::size_t index = 0; index < collisionConstraints.size(); ++index) {
      collisionConstraints[index]->drawOn(*viewer);
    }
    viewer->drawScene();
    viewer->pollEvents();
  }
  return 0;
}
