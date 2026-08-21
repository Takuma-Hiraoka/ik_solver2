#pragma once

#include <ik_constraint/IKConstraint.h>
#include <mujoco_viewer/mujoco_viewer.h>
#include <prioritized_qp_base/PrioritizedQPBaseSolver.h>
#include <prioritized_qp_osqp/prioritized_qp_osqp.h>

#include <Eigen/Core>

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

namespace prioritized_inverse_kinematics_solver {
  class IKParam {
  public:
    size_t maxIteration = 1;
    size_t minIteration = 0;
    std::vector<double> dqWeight;
    double wn = 1e-6;
    std::vector<double> wnVec;
    double we = 1e0;
    std::vector<double> weVec;
    double wmax = 1e-1;
    std::vector<double> wmaxVec;
    int debugLevel = 0;
    bool checkFinalState = true;
    double convergeThre = 5e-3;
    int satisfiedConvergeLevel = -1;
    size_t pathOutputLoop = 1;
    std::shared_ptr<mujoco_viewer::Viewer> viewer = nullptr;
    int viewMilliseconds = 0;
  };

  inline void generateOSQPTask(
    std::shared_ptr<prioritized_qp_base::Task>& task,
    int debugLevel) {
    std::shared_ptr<prioritized_qp_osqp::Task> taskOSQP =
      std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
    if(!taskOSQP) {
      task = std::make_shared<prioritized_qp_osqp::Task>();
      taskOSQP = std::dynamic_pointer_cast<prioritized_qp_osqp::Task>(task);
    }
    taskOSQP->solver().settings()->setVerbosity(debugLevel > 1);
    taskOSQP->solver().settings()->setMaxIteration(4000);
    taskOSQP->solver().settings()->setAbsoluteTolerance(1e-3);
    taskOSQP->solver().settings()->setRelativeTolerance(1e-3);
    taskOSQP->solver().settings()->setScaledTerimination(true);
  }

  bool solveIKLoop(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
    const IKParam& param = IKParam(),
    std::shared_ptr<std::vector<std::vector<Eigen::VectorXd> > > path = nullptr,
    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc =
      generateOSQPTask);

  bool solveIKLoop(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    const std::vector<std::shared_ptr<ik_constraint::IKConstraint> >& rejections,
    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
    const IKParam& param = IKParam(),
    std::shared_ptr<std::vector<std::vector<Eigen::VectorXd> > > path = nullptr,
    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc =
      generateOSQPTask);
}
