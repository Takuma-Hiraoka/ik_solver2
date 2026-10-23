#pragma once
#include <ik_constraint/IKConstraint.h>
#include <stdexcept>

namespace ik_constraint_scfr { namespace detail {
  template<class DataPointer>
  std::vector<DataPointer> selectData(
    const std::vector<std::shared_ptr<const pinocchio::Model> >& allModels,
    const std::vector<DataPointer>& data,
    const std::vector<std::shared_ptr<const pinocchio::Model> >& models) {
    if(allModels.size() != data.size()) throw std::invalid_argument("SCFR: invalid data vector");
    std::vector<DataPointer> result;
    for(std::size_t i = 0; i < models.size(); ++i) {
      bool found = false;
      for(std::size_t j = 0; j < allModels.size(); ++j) {
        if(models[i] == allModels[j]) {
          if(!data[j]) throw std::invalid_argument("SCFR: null data");
          result.push_back(data[j]); found = true; break;
        }
      }
      if(!found) throw std::invalid_argument("SCFR: unregistered model");
    }
    return result;
  }
  inline Eigen::SparseMatrix<double,Eigen::RowMajor> contactWrenchMatrix() {
    Eigen::SparseMatrix<double,Eigen::RowMajor> C(11,6);
    C.insert(0,2) = 1.0;
    for(int axis = 0; axis < 2; ++axis) {
      C.insert(1+axis*2,axis) = 1.0; C.insert(1+axis*2,2) = 0.2;
      C.insert(2+axis*2,axis) = -1.0; C.insert(2+axis*2,2) = 0.2;
    }
    for(int axis = 0; axis < 3; ++axis) {
      const double coefficient = axis == 2 ? 0.005 : 0.05;
      C.insert(5+axis*2,3+axis) = 1.0; C.insert(5+axis*2,2) = coefficient;
      C.insert(6+axis*2,3+axis) = -1.0; C.insert(6+axis*2,2) = coefficient;
    }
    return C;
  }
} }
