#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ik_constraint/JointLimitConstraint.h>
#include <ik_constraint/PositionConstraint.h>
#include <ik_constraint_sp/SPConstraint.h>
#include <pinocchio_primitive_models/pinocchio_primitive_models.h>
#include <pinocchio_mujoco_converter/pinocchio_mujoco_converter.h>
#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <iostream>
#include <cmath>
#include <string>

int main(int argc, char** argv) {
  const bool headless = argc > 1 && std::string(argv[1]) == "--headless";
  const std::string robotDir = ament_index_cpp::get_package_share_directory(
    "prioritized_inverse_kinematics_solver_sample")+"/unitree_ros/robots/g1_description";
  std::shared_ptr<pinocchio::Model> robot = std::make_shared<pinocchio::Model>();
  pinocchio::urdf::buildModel(robotDir+"/g1_29dof.urdf",pinocchio::JointModelFreeFlyer(),*robot);
  Eigen::VectorXd q = pinocchio::neutral(*robot);
  q[2] = 0.65;
  const std::vector<std::string> sides{"left_","right_"};
  for(size_t i=0; i<sides.size(); i++) {
    q[robot->joints[robot->getJointId(sides[i]+"hip_pitch_joint")].idx_q()] = -0.6;
    q[robot->joints[robot->getJointId(sides[i]+"knee_joint")].idx_q()] = 1.4;
    q[robot->joints[robot->getJointId(sides[i]+"ankle_pitch_joint")].idx_q()] = -0.8;
    q[robot->joints[robot->getJointId(sides[i]+"shoulder_pitch_joint")].idx_q()] = -0.4;
    q[robot->joints[robot->getJointId(sides[i]+"elbow_joint")].idx_q()] = 1.2;
  }
  pinocchio::Data initial(*robot);
  pinocchio::forwardKinematics(*robot,initial,q);
  const pinocchio::JointIndex leftFoot = robot->getJointId("left_ankle_roll_joint");
  // Place the initial sole plane on the viewer's ground.
  q[2] -= initial.oMi[leftFoot].translation().z()-0.035;
  pinocchio::forwardKinematics(*robot,initial,q);
  pinocchio::centerOfMass(*robot,initial,q);

  const pinocchio::SE3 handLocal(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0.10,0,0));
  std::vector<pinocchio::JointIndex> hands;
  std::vector<pinocchio::SE3> handPoses;
  for(size_t i=0; i<sides.size(); i++) {
    hands.push_back(robot->getJointId(sides[i]+"wrist_yaw_joint"));
    handPoses.push_back(initial.oMi[hands.back()]*handLocal);
  }
  const Eigen::Vector3d boxPosition = 0.5*(handPoses[0].translation()+handPoses[1].translation());
  const Eigen::Vector3d boxSize(0.22,std::abs(handPoses[0].translation().y()-handPoses[1].translation().y()),0.18);
  const double boxMass = 8.0;
  std::shared_ptr<pinocchio::Model> box = std::make_shared<pinocchio::Model>();
  pinocchio_primitive_models::createBox(boxSize,boxMass,*box);
  Eigen::VectorXd boxQ = pinocchio::neutral(*box);
  boxQ.head<3>() = boxPosition;
  const pinocchio::SE3 boxPose(Eigen::Matrix3d::Identity(),boxPosition);

  std::shared_ptr<ik_constraint_sp::SPConstraint> support = std::make_shared<ik_constraint_sp::SPConstraint>();
  support->setModels({robot,box});
  support->setTolerance(0.01);
  std::vector<ik_constraint_sp::SupportPolygon> polygons;
  std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > > constraints(3);
  for(size_t i=0; i<sides.size(); i++) {
    const pinocchio::JointIndex foot = robot->getJointId(sides[i]+"ankle_roll_joint");
    std::shared_ptr<ik_constraint::PositionConstraint> fixedFoot = std::make_shared<ik_constraint::PositionConstraint>(robot);
    fixedFoot->setAJointId(foot);
    fixedFoot->setBLocalPose(initial.oMi[foot]);
    fixedFoot->setPrecision(1e-4);
    constraints[0].push_back(fixedFoot);
    ik_constraint_sp::SupportPolygon polygon;
    polygon.model = robot;
    polygon.jointId = foot;
    polygon.localPose.translation() = Eigen::Vector3d(0.03,0,-0.035);
    polygon.vertices = {Eigen::Vector2d(-0.10,-0.045),Eigen::Vector2d(-0.10,0.045),
                        Eigen::Vector2d(0.10,0.045),Eigen::Vector2d(0.10,-0.045)};
    polygons.push_back(polygon);

    // Keep both grasps rigid relative to the moving box.
    std::shared_ptr<ik_constraint::PositionConstraint> grasp = std::make_shared<ik_constraint::PositionConstraint>(robot,box);
    grasp->setAJointId(hands[i]);
    grasp->setALocalPose(handLocal);
    grasp->setBJointId(1);
    grasp->setBLocalPose(boxPose.inverse()*handPoses[i]);
    grasp->setPrecision(1e-4);
    constraints[1].push_back(grasp);
  }
  support->setSupportPolygons(polygons);
  constraints[0].push_back(support);
  for(pinocchio::JointIndex j=2; j<robot->njoints; j++) {
    std::shared_ptr<ik_constraint::JointLimitConstraint> limit = std::make_shared<ik_constraint::JointLimitConstraint>(robot);
    limit->setJointId(j);
    constraints[0].push_back(limit);
  }
  std::shared_ptr<ik_constraint::PositionConstraint> boxTarget = std::make_shared<ik_constraint::PositionConstraint>(box);
  boxTarget->setAJointId(1);
  pinocchio::SE3 target = boxPose;
  target.translation() += Eigen::Vector3d(0.20,0,0.0);
  boxTarget->setBLocalPose(target);
  boxTarget->setPrecision(1e-3);
  constraints[2].push_back(boxTarget);

  prioritized_inverse_kinematics_solver::IKParam param;
  param.maxIteration = 300;
  param.debugLevel = headless ? 1 : 3;
  param.viewMilliseconds = 50;
  std::shared_ptr<mujoco_viewer::Viewer> viewer;
  char error[1024] = {0};
  std::shared_ptr<mjSpec> spec(mj_parseXML((robotDir+"/g1_29dof.xml").c_str(),nullptr,error,sizeof(error)),mj_deleteSpec);
  if(!spec) { std::cerr << error << std::endl; return 1; }
  pinocchio_mujoco_converter::createBox(boxSize,boxMass,*spec,"carried_box");
  std::shared_ptr<mjModel> mjModelPtr(mj_compile(spec.get(),nullptr),mj_deleteModel);
  if(!mjModelPtr) { std::cerr << mjs_getError(spec.get()) << std::endl; return 1; }
  if(mjModelPtr->nq != robot->nq+box->nq) {
    std::cerr << "Pinocchio and MuJoCo configuration dimensions differ" << std::endl;
    return 1;
  }
  if(!headless) {
    viewer = std::make_shared<mujoco_viewer::Viewer>();
    viewer->viewModel(mjModelPtr);
    param.viewer = viewer;
  }
  std::vector<std::shared_ptr<pinocchio::Model> > models{robot,box};
  std::vector<Eigen::VectorXd> configurations{q,boxQ};
  std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
  const bool solved = prioritized_inverse_kinematics_solver::solveIKLoop(models,configurations,constraints,tasks,param);
  std::cout << "IK result: " << (solved ? "succeeded" : "failed")
            << "\nCombined COM: " << support->getCOM().transpose()
            << "\nSupport margin: " << support->margin() << " [m]" << std::endl;
  while(viewer && viewer->isOpen()) {
    viewer->updateScene();
    support->drawOn(*viewer);
    for(size_t i=0; i<constraints[1].size(); i++) constraints[1][i]->drawOn(*viewer);
    viewer->drawScene();
    viewer->pollEvents();
  }
  return solved ? 0 : 1;
}
