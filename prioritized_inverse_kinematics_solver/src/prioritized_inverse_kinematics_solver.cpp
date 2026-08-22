#include <prioritized_inverse_kinematics_solver/prioritized_inverse_kinematics_solver.h>

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/Sparse>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>

#include <sys/select.h>
#include <unistd.h>

namespace prioritized_inverse_kinematics_solver {
  static void checkInputs(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<Eigen::VectorXd>& configurations,
    const IKParam& param) {
    if(models.empty()) throw std::invalid_argument("at least one Pinocchio model is required");
    if(models.size() != configurations.size()) {
      throw std::invalid_argument("model and configuration vector sizes do not match");
    }
    for(size_t i=0; i<models.size(); i++) {
      if(!models[i]) throw std::invalid_argument("model vector contains a null model");
      for(size_t j=0; j<i; j++) {
        if(models[i].get() == models[j].get()) {
          throw std::invalid_argument("model vector contains a duplicate model");
        }
      }
      if(configurations[i].size() != models[i]->nq) {
        throw std::invalid_argument("configuration dimension does not match model.nq");
      }
    }
    if(param.pathOutputLoop == 0) {
      throw std::invalid_argument("pathOutputLoop must be greater than zero");
    }
  }

  static void updateModelData(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data) {
    for(size_t i=0; i<models.size(); i++) {
      pinocchio::forwardKinematics(*models[i], *data[i], configurations[i]);
      pinocchio::computeJointJacobians(*models[i], *data[i], configurations[i]);
      pinocchio::updateFramePlacements(*models[i], *data[i]);
      pinocchio::centerOfMass(*models[i], *data[i], configurations[i]);
    }
  }

  static void updateConstraints(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    bool updateJacobian) {
    for(size_t i=0; i<ikcList.size(); i++) {
      for(size_t j=0; j<ikcList[i].size(); j++) {
        if(!ikcList[i][j]) throw std::invalid_argument("constraint list contains null constraint");
        const std::vector<const pinocchio::Data*> constraintDataConst =
          ikcList[i][j]->getConstDataVector(models, data);
        ikcList[i][j]->updateBounds(constraintDataConst);
        if(updateJacobian) {
          const std::vector<pinocchio::Data*> constraintData =
            ikcList[i][j]->getDataVector(models, data);
          ikcList[i][j]->updateJacobian(constraintData);
          ikcList[i][j]->convertJacobian(models);
        }
      }
    }
  }

  static bool checkConstraintsSatisfied(
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    int checkLevel = std::numeric_limits<int>::max()) {
    bool satisfied = true;
    for(size_t i=0; i<ikcList.size(); i++) {
      if(checkLevel < 0 || static_cast<int>(i) > checkLevel) break;
      for(size_t j=0; j<ikcList[i].size(); j++) {
        if(!ikcList[i][j]->isSatisfied()) satisfied = false;
      }
    }
    return satisfied;
  }

  static void updateRejections(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<std::shared_ptr<pinocchio::Data> >& data,
    const std::vector<std::shared_ptr<ik_constraint::IKConstraint> >& rejections) {
    for(size_t i=0; i<rejections.size(); i++) {
      if(!rejections[i]) throw std::invalid_argument("rejection list contains null constraint");
      const std::vector<const pinocchio::Data*> constraintData =
        rejections[i]->getConstDataVector(models, data);
      rejections[i]->updateBounds(constraintData);
    }
  }

  static bool checkRejectionsSatisfied(
    const std::vector<std::shared_ptr<ik_constraint::IKConstraint> >& rejections) {
    for(size_t i=0; i<rejections.size(); i++) {
      if(!rejections[i]->isSatisfied()) return false;
    }
    return true;
  }

  static void drawConstraints(
    mujoco_viewer::Viewer& viewer,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList) {
    for(size_t i=0; i<ikcList.size(); i++) {
      for(size_t j=0; j<ikcList[i].size(); j++) {
        ikcList[i][j]->drawOn(viewer);
      }
    }
  }

  static void updateViewer(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    const std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    const IKParam& param) {
    if(param.debugLevel <= 2 || !param.viewer) return;
    if(!param.viewer->model() || !param.viewer->data()) {
      throw std::invalid_argument("viewer does not have a MuJoCo model");
    }

    Eigen::Index configurationSize = 0;
    for(size_t i=0; i<configurations.size(); i++) {
      configurationSize += configurations[i].size();
    }
    if(configurationSize != param.viewer->model()->nq) {
      throw std::invalid_argument("configuration dimension does not match MuJoCo model.nq");
    }

    Eigen::Index modelOffset = 0;
    for(size_t i=0; i<models.size(); i++) {
      for(Eigen::Index j=0; j<configurations[i].size(); j++) {
        param.viewer->data()->qpos[modelOffset + j] = configurations[i][j];
      }

      for(pinocchio::JointIndex jointId=1; jointId<models[i]->joints.size(); jointId++) {
        const std::string jointType = models[i]->joints[jointId].shortname();
        const Eigen::Index jointConfigurationIndex = models[i]->idx_qs[jointId];
        if(jointType == "JointModelFreeFlyer") {
          const Eigen::Index quaternionIndex = modelOffset + jointConfigurationIndex + 3;
          param.viewer->data()->qpos[quaternionIndex] =
            configurations[i][jointConfigurationIndex + 6];
          param.viewer->data()->qpos[quaternionIndex + 1] =
            configurations[i][jointConfigurationIndex + 3];
          param.viewer->data()->qpos[quaternionIndex + 2] =
            configurations[i][jointConfigurationIndex + 4];
          param.viewer->data()->qpos[quaternionIndex + 3] =
            configurations[i][jointConfigurationIndex + 5];
        } else if(jointType == "JointModelSpherical") {
          const Eigen::Index quaternionIndex = modelOffset + jointConfigurationIndex;
          param.viewer->data()->qpos[quaternionIndex] =
            configurations[i][jointConfigurationIndex + 3];
          param.viewer->data()->qpos[quaternionIndex + 1] =
            configurations[i][jointConfigurationIndex];
          param.viewer->data()->qpos[quaternionIndex + 2] =
            configurations[i][jointConfigurationIndex + 1];
          param.viewer->data()->qpos[quaternionIndex + 3] =
            configurations[i][jointConfigurationIndex + 2];
        }
      }
      modelOffset += configurations[i].size();
    }

    mj_forward(param.viewer->model(), param.viewer->data());
    param.viewer->updateScene();
    drawConstraints(*param.viewer, ikcList);
    param.viewer->drawScene();
    param.viewer->pollEvents();
    if(param.viewMilliseconds >= 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(param.viewMilliseconds));
    } else {
      while(param.viewer->isOpen()) {
        fd_set readFileDescriptors;
        FD_ZERO(&readFileDescriptors);
        FD_SET(STDIN_FILENO, &readFileDescriptors);
        timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        const int inputReady = select(
          STDIN_FILENO + 1, &readFileDescriptors, nullptr, nullptr, &timeout);
        if(inputReady < 0) {
          throw std::runtime_error("failed to check standard input while updating viewer");
        }
        if(inputReady > 0) {
          std::getchar();
          break;
        }
        param.viewer->updateScene();
        drawConstraints(*param.viewer, ikcList);
        param.viewer->drawScene();
        param.viewer->pollEvents();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  static bool solveIKOnce(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
    const IKParam& param,
    const std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)>& taskGeneratorFunc) {
    const size_t dimension = ik_constraint::IKConstraint::getVariableDimension(models);

    if(prevTasks.size() != ikcList.size()) {
      prevTasks.clear();
      prevTasks.resize(ikcList.size(), nullptr);
    }

    for(size_t i=0; i<ikcList.size(); i++) {
      taskGeneratorFunc(prevTasks[i], param.debugLevel);
      prevTasks[i]->toSolve() = (i != 0);

      Eigen::Index numEqs = 0;
      Eigen::Index numIneqs = 0;
      Eigen::Index numExts = 0;
      for(size_t j=0; j<ikcList[i].size(); j++) {
        numEqs += ikcList[i][j]->getEq().rows();
        numIneqs += ikcList[i][j]->getMinIneq().rows();
        numExts += std::max(
          ikcList[i][j]->getJacobianExt().cols(),
          ikcList[i][j]->getJacobianIneqExt().cols());
      }

      prevTasks[i]->A().resize(numEqs, static_cast<Eigen::Index>(dimension));
      prevTasks[i]->b().resize(numEqs);
      prevTasks[i]->C().resize(numIneqs, static_cast<Eigen::Index>(dimension));
      prevTasks[i]->dl().resize(numIneqs);
      prevTasks[i]->du().resize(numIneqs);
      prevTasks[i]->wa() = Eigen::VectorXd::Ones(numEqs);
      prevTasks[i]->wc() = Eigen::VectorXd::Ones(numIneqs);
      prevTasks[i]->A_ext().resize(numEqs, numExts);
      prevTasks[i]->C_ext().resize(numIneqs, numExts);

      Eigen::Index eqIndex = 0;
      Eigen::Index ineqIndex = 0;
      Eigen::Index extIndex = 0;
      for(size_t j=0; j<ikcList[i].size(); j++) {
        const std::shared_ptr<ik_constraint::IKConstraint>& constraint = ikcList[i][j];
        prevTasks[i]->A().middleRows(eqIndex, constraint->getEq().rows()) = constraint->getJacobian();
        prevTasks[i]->b().segment(eqIndex, constraint->getEq().rows()) = constraint->getEq();
        prevTasks[i]->C().middleRows(ineqIndex, constraint->getMinIneq().rows()) = constraint->getJacobianIneq();
        prevTasks[i]->dl().segment(ineqIndex, constraint->getMinIneq().rows()) = constraint->getMinIneq();
        prevTasks[i]->du().segment(ineqIndex, constraint->getMaxIneq().rows()) = constraint->getMaxIneq();

        Eigen::SparseMatrix<double,Eigen::ColMajor> aExt(
          constraint->getJacobianExt().rows(), numExts);
        aExt.middleCols(extIndex, constraint->getJacobianExt().cols()) = constraint->getJacobianExt();
        prevTasks[i]->A_ext().middleRows(eqIndex, aExt.rows()) = aExt;
        Eigen::SparseMatrix<double,Eigen::ColMajor> cExt(
          constraint->getJacobianIneqExt().rows(), numExts);
        cExt.middleCols(extIndex, constraint->getJacobianIneqExt().cols()) =
          constraint->getJacobianIneqExt();
        prevTasks[i]->C_ext().middleRows(ineqIndex, cExt.rows()) = cExt;

        eqIndex += constraint->getEq().rows();
        ineqIndex += constraint->getMinIneq().rows();
        extIndex += std::max(
          constraint->getJacobianExt().cols(),
          constraint->getJacobianIneqExt().cols());
      }

      double sumError = prevTasks[i]->b().squaredNorm();
      for(Eigen::Index j=0; j<prevTasks[i]->dl().size(); j++) {
        if(prevTasks[i]->dl()[j] > 0) sumError += std::pow(prevTasks[i]->dl()[j], 2);
        if(prevTasks[i]->du()[j] < 0) sumError += std::pow(prevTasks[i]->du()[j], 2);
      }
      const double wmax = param.wmaxVec.size() == ikcList.size() ? param.wmaxVec[i] : param.wmax;
      const double we = param.weVec.size() == ikcList.size() ? param.weVec[i] : param.we;
      const double wn = param.wnVec.size() == ikcList.size() ? param.wnVec[i] : param.wn;
      const double weight = std::min(wmax, sumError * we + wn);
      prevTasks[i]->w() = Eigen::VectorXd::Ones(static_cast<Eigen::Index>(dimension)) * weight;
      prevTasks[i]->w_ext() = Eigen::VectorXd::Ones(numExts) * weight;
      if(param.dqWeight.size() == dimension) {
        for(size_t j=0; j<dimension; j++) prevTasks[i]->w()[j] *= param.dqWeight[j];
      }
      if(param.debugLevel > 1) prevTasks[i]->name() = std::string("Task") + std::to_string(i);
    }

    Eigen::VectorXd result;
    if(!prioritized_qp_base::solve(prevTasks, result, param.debugLevel)) {
      std::cerr << "[PrioritizedIK] prioritized_qp_base::solve failed" << std::endl;
      return true;
    }
    if(!result.allFinite()) {
      std::cerr << "[PrioritizedIK] ERROR nan/inf is found" << std::endl;
      return true;
    }

    Eigen::Index offset = 0;
    for(size_t i=0; i<models.size(); i++) {
      configurations[i] = pinocchio::integrate(
        *models[i], configurations[i], result.segment(offset, models[i]->nv));
      offset += models[i]->nv;
    }
    return result.norm() < param.convergeThre;
  }

  bool solveIKLoop(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
    const IKParam& param,
    std::shared_ptr<std::vector<std::vector<Eigen::VectorXd> > > path,
    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc) {
    return solveIKLoop(
      models, configurations, ikcList,
      std::vector<std::shared_ptr<ik_constraint::IKConstraint> >(),
      prevTasks, param, path, taskGeneratorFunc);
  }

  bool solveIKLoop(
    const std::vector<std::shared_ptr<pinocchio::Model> >& models,
    std::vector<Eigen::VectorXd>& configurations,
    const std::vector<std::vector<std::shared_ptr<ik_constraint::IKConstraint> > >& ikcList,
    const std::vector<std::shared_ptr<ik_constraint::IKConstraint> >& rejections,
    std::vector<std::shared_ptr<prioritized_qp_base::Task> >& prevTasks,
    const IKParam& param,
    std::shared_ptr<std::vector<std::vector<Eigen::VectorXd> > > path,
    std::function<void(std::shared_ptr<prioritized_qp_base::Task>&,int)> taskGeneratorFunc) {
    checkInputs(models, configurations, param);
    if(!taskGeneratorFunc) throw std::invalid_argument("taskGeneratorFunc is empty");

    std::vector<std::shared_ptr<pinocchio::Data> > data =
      ik_constraint::IKConstraint::createDataVector(models);
    updateModelData(models, configurations, data);
    updateConstraints(models, data, ikcList, true);
    updateRejections(models, data, rejections);

    if(path) {
      path->clear();
      path->push_back(configurations);
    }
    if(!checkRejectionsSatisfied(rejections)) return false;

    for(size_t loop=0; loop<param.maxIteration; loop++) {
      const std::chrono::steady_clock::time_point loopStartTime =
        std::chrono::steady_clock::now();
      const std::vector<Eigen::VectorXd> previousConfigurations = configurations;
      const bool converged = solveIKOnce(
        models, configurations, ikcList, prevTasks, param, taskGeneratorFunc);
      updateModelData(models, configurations, data);
      updateConstraints(models, data, ikcList, true);
      updateRejections(models, data, rejections);
      updateViewer(models, configurations, ikcList, param);

      const bool outputPath = path && (loop + 1) % param.pathOutputLoop == 0;
      if(outputPath) path->push_back(configurations);

      bool terminate = false;
      bool satisfied = false;
      if(!checkRejectionsSatisfied(rejections)) {
        terminate = true;
        configurations = previousConfigurations;
        updateModelData(models, configurations, data);
        updateConstraints(models, data, ikcList, true);
        updateRejections(models, data, rejections);
        if(outputPath) path->pop_back();
        satisfied = checkConstraintsSatisfied(ikcList);
      } else if(loop + 1 >= param.maxIteration) {
        terminate = true;
        satisfied = param.checkFinalState && checkConstraintsSatisfied(ikcList);
      } else if(converged && checkConstraintsSatisfied(ikcList, param.satisfiedConvergeLevel)) {
        terminate = true;
        satisfied = checkConstraintsSatisfied(ikcList);
      } else if(loop >= param.minIteration && checkConstraintsSatisfied(ikcList)) {
        terminate = true;
        satisfied = true;
      }

      if(param.debugLevel > 0) {
        const std::chrono::steady_clock::time_point loopEndTime =
          std::chrono::steady_clock::now();
        const double time =
          std::chrono::duration<double>(loopEndTime - loopStartTime).count();
        std::cerr << "[PrioritizedIK] solveIKLoop loop: " << loop
                  << " time: " << time << "[s]." << std::endl;
      }

      if(terminate) {
        if(path) path->push_back(configurations);
        return satisfied;
      }
    }
    return false;
  }
}
