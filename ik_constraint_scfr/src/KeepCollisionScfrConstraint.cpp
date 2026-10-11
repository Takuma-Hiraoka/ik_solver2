#include <ik_constraint_scfr/KeepCollisionScfrConstraint.h>
#include <clpeigen/clpeigen.h>
#include <ik_constraint_scfr/contact_utils.h>
#include <limits>
#include <iostream>

namespace ik_constraint_scfr {
  std::vector<std::shared_ptr<ik_constraint::IKConstraint> > KeepCollisionScfrConstraint::makeChildren(
    const std::shared_ptr<ScfrConstraint>& scfr,
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts) {
    if(!scfr) throw std::invalid_argument("KeepCollisionScfrConstraint: null SCFR");
    std::vector<std::shared_ptr<ik_constraint::IKConstraint> > children;
    for(std::size_t i = 0; i < contacts.size(); ++i) {
      if(!contacts[i]) throw std::invalid_argument("KeepCollisionScfrConstraint: null contact");
      children.push_back(contacts[i]);
    }
    children.push_back(scfr);
    return children;
  }
  KeepCollisionScfrConstraint::KeepCollisionScfrConstraint(
    const std::shared_ptr<ScfrConstraint>& scfr,
    const std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> >& contacts)
    : ik_constraint::ANDConstraint(makeChildren(scfr, contacts)),
      scfrConstraint_(scfr), keepCollisionConstraints_(contacts) {}

  void KeepCollisionScfrConstraint::updateBounds(const std::vector<const pinocchio::Data*>& data) {
    std::vector<std::size_t> indices;
    std::vector<double> margins;
    std::vector<Eigen::Isometry3d> poses;
    for(std::size_t i = 0; i < this->keepCollisionConstraints_.size(); ++i) {
      const std::shared_ptr<ik_constraint::CollisionConstraint>& contact = this->keepCollisionConstraints_[i];
      contact->updateBounds(detail::selectData(this->models_, data, contact->getModels()));
      if(!contact->isSatisfied()) continue;
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation() = contact->getBCurrentLocalPosition();
      if(contact->getBModel()) {
        const std::vector<const pinocchio::Data*> BData = detail::selectData(this->models_, data,
          std::vector<std::shared_ptr<const pinocchio::Model> >(1, contact->getBModel()));
        const Eigen::Vector3d localPosition = pose.translation();
        pose.translation() = BData[0]->oMi[contact->getBJointId()].act(localPosition);
      }
      const Eigen::Vector3d z = contact->getCurrentDirection().normalized();
      if(!z.allFinite()) throw std::invalid_argument("KeepCollisionScfrConstraint: invalid contact normal");
      const Eigen::Vector3d x = z.unitOrthogonal();
      pose.linear().col(0) = x;
      pose.linear().col(1) = z.cross(x);
      pose.linear().col(2) = z;
      indices.push_back(i); margins.push_back(contact->margin()); poses.push_back(pose);
    }
    this->breakableIndex_ = -1;
    double minimumMargin = std::numeric_limits<double>::infinity();
    if(indices.size() > this->minimumContactCount_) {
      for(std::size_t omitted = 0; omitted < indices.size(); ++omitted) {
        std::vector<Eigen::Isometry3d> candidatePoses;
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As, Cs;
        std::vector<Eigen::VectorXd> bs, dls, dus;
        for(std::size_t i = 0; i < indices.size(); ++i) {
          if(i == omitted) continue;
          candidatePoses.push_back(poses[i]); As.emplace_back(0,6); bs.emplace_back(0);
          Cs.push_back(detail::contactWrenchMatrix()); dls.push_back(Eigen::VectorXd::Zero(11));
          Eigen::VectorXd upper = Eigen::VectorXd::Constant(11,1e10); upper[0] = 2000.0; dus.push_back(upper);
        }
        if(margins[omitted] < minimumMargin && checkSCFRExistance(candidatePoses, As, bs, Cs, dls, dus,
           this->scfrConstraint_->getMass(), this->breakableSCFRParam_)) {
          minimumMargin = margins[omitted]; this->breakableIndex_ = static_cast<int>(indices[omitted]);
        }
      }
    }
    std::vector<Eigen::Isometry3d> retainedPoses;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As, Cs;
    std::vector<Eigen::VectorXd> bs, dls, dus;
    this->children_.clear();
    for(std::size_t i = 0; i < indices.size(); ++i) {
      if(static_cast<int>(indices[i]) == this->breakableIndex_) continue;
      this->children_.push_back(this->keepCollisionConstraints_[indices[i]]);
      retainedPoses.push_back(poses[i]); As.emplace_back(0,6); bs.emplace_back(0);
      Cs.push_back(detail::contactWrenchMatrix()); dls.push_back(Eigen::VectorXd::Zero(11));
      Eigen::VectorXd upper = Eigen::VectorXd::Constant(11,1e10); upper[0] = 2000.0; dus.push_back(upper);
    }
    this->scfrConstraint_->setJointIds(std::vector<pinocchio::JointIndex>());
    this->scfrConstraint_->setPoses(retainedPoses); this->scfrConstraint_->setAs(As); this->scfrConstraint_->setBs(bs);
    this->scfrConstraint_->setCs(Cs); this->scfrConstraint_->setDls(dls); this->scfrConstraint_->setDus(dus);
    this->children_.push_back(this->scfrConstraint_);
    // Keep the full model list stable even when a contact is released.
    ik_constraint::ANDConstraint::updateBounds(data);
  }
  std::shared_ptr<ik_constraint::IKConstraint> KeepCollisionScfrConstraint::clone() const {
    std::vector<std::shared_ptr<ik_constraint::CollisionConstraint> > contacts;
    for(std::size_t i = 0; i < this->keepCollisionConstraints_.size(); ++i)
      contacts.push_back(std::static_pointer_cast<ik_constraint::CollisionConstraint>(this->keepCollisionConstraints_[i]->clone()));
    std::shared_ptr<KeepCollisionScfrConstraint> result = std::make_shared<KeepCollisionScfrConstraint>(
      std::static_pointer_cast<ScfrConstraint>(this->scfrConstraint_->clone()), contacts);
    result->minimumContactCount_ = this->minimumContactCount_; result->breakableSCFRParam_ = this->breakableSCFRParam_;
    result->debugLevel_ = this->debugLevel_;
    return result;
  }
    inline bool appendRow(const std::vector<Eigen::VectorXd>& vs, Eigen::VectorXd& vout){
    size_t rows = 0;
    for(size_t i=0;i<vs.size();i++){
      rows += vs[i].size();
    }
    vout.resize(rows);
    size_t idx = 0;
    for(size_t i=0;i<vs.size();i++){
      vout.segment(idx,vs[i].size()) = vs[i];
      idx += vs[i].size();
    }

    return true;
  }

  inline bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(Mout.rows(),0);//もとのMoutのrowを用いる
      return true;
    }
    size_t rows = Ms[0].rows();
    size_t cols = 0;
    for(size_t i=0;i<Ms.size();i++){
      cols += Ms[i].cols();
      if(Ms[i].rows() != rows){
        std::cerr << "[appendCol] Ms[i].rows() " << Ms[i].rows() << " != rows " << rows << std::endl;
        return false;
      }
    }
    Mout.resize(rows,cols);
    size_t idx = 0;
    for(size_t i=0;i<Ms.size();i++){
      Mout.middleCols(idx,Ms[i].cols()) = Ms[i];
      idx += Ms[i].cols();
    }

    return true;
  }

  inline bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
    if(Ms.size() == 0) {
      Mout.resize(0,0);
      return true;
    }
    size_t cols = 0;
    size_t rows = 0;
    for(size_t i=0;i<Ms.size();i++){
      rows += Ms[i].rows();
      cols += Ms[i].cols();
    }
    Mout.resize(rows,cols);
    size_t idx_row = 0;
    size_t idx_col = 0;
    for(size_t i=0;i<Ms.size();i++){
      Eigen::SparseMatrix<double, Eigen::ColMajor> M_ColMajor(Ms[i].rows(),cols);
      M_ColMajor.middleCols(idx_col,Ms[i].cols()) = Ms[i];
      Mout.middleRows(idx_row,M_ColMajor.rows()) = M_ColMajor;
      idx_row += Ms[i].rows();
      idx_col += Ms[i].cols();
    }

    return true;
  }

  bool checkSCFRExistance(std::vector<Eigen::Isometry3d>& poses,
                          const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& As, // pose local frame. 列は6(F N)
                          const std::vector<Eigen::VectorXd>& bs,
                          const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Cs, // pose local frame. 列は6(F N)
                          const std::vector<Eigen::VectorXd>& dls,
                          const std::vector<Eigen::VectorXd>& dus,
                          const double& m, // robotの質量
                          const scfr_solver::SCFRParam& param) {
    if(poses.size() != As.size() ||
       poses.size() != bs.size() ||
       poses.size() != Cs.size() ||
       poses.size() != dls.size() ||
       poses.size() != dus.size()){
      std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch" << std::endl;
      return false;
    }

    if(poses.size() == 0) {
      return false;
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd b;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C;
    Eigen::VectorXd dl;
    Eigen::VectorXd du;
    {
      // compute A, b, C, dl, du
      // x = [dpx dpy dw1 dw2 ...]^T
      // wはpose座標系．poseまわり.サイズは6

      // Grasp Matrix Gx = h
      // 原点まわりのつりあい. 座標軸はworld系の向き
      Eigen::SparseMatrix<double,Eigen::RowMajor> G;
      Eigen::VectorXd h = Eigen::VectorXd::Zero(6);
      {
        h[2] = m*9.80665;

        std::vector<Eigen::SparseMatrix<double,Eigen::ColMajor> > Gs;
        {
          Eigen::SparseMatrix<double,Eigen::ColMajor> G01(6,2);
          G01.insert(3,1) = -m*9.80665;
          G01.insert(4,0) = m*9.80665;
          Gs.push_back(G01);
        }

        for(size_t i=0;i<poses.size();i++){
          Eigen::SparseMatrix<double,Eigen::ColMajor> GraspMatrix(6,6);
          {
            const Eigen::Isometry3d& pos = poses[i];
            const Eigen::Matrix3d& R = pos.linear();
            Eigen::Matrix3d p_x;
            p_x << 0.0, -pos.translation()(2), pos.translation()(1),
              pos.translation()(2), 0.0, -pos.translation()(0),
              -pos.translation()(1), pos.translation()(0), 0.0;
            const Eigen::Matrix3d& p_x_R = p_x * R;

            std::vector<Eigen::Triplet<double> > G_tripletList;
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(row,col,R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,col,p_x_R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,3+col,R(row,col)));
              }
            }
            GraspMatrix.setFromTriplets(G_tripletList.begin(), G_tripletList.end());
          }
          Gs.push_back(GraspMatrix);
        }

        Eigen::SparseMatrix<double,Eigen::ColMajor> G_ColMajor;
        appendCol(Gs,G_ColMajor);
        G = G_ColMajor;
      }

      //接触力制約
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_contact;
      Eigen::VectorXd b_contact;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_contact;
      Eigen::VectorXd dl_contact;
      Eigen::VectorXd du_contact;
      {
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > A_contacts;
        std::vector<Eigen::VectorXd> b_contacts;
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > C_contacts;
        std::vector<Eigen::VectorXd> dl_contacts;
        std::vector<Eigen::VectorXd> du_contacts;

        {
          Eigen::SparseMatrix<double,Eigen::RowMajor> A01(0,2);
          Eigen::VectorXd b01(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C01(0,2);
          Eigen::VectorXd dl01(0);
          Eigen::VectorXd du01(0);
          A_contacts.push_back(A01);
          b_contacts.push_back(b01);
          C_contacts.push_back(C01);
          dl_contacts.push_back(dl01);
          du_contacts.push_back(du01);
        }

        for (size_t i=0;i<poses.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A = As[i];
          Eigen::VectorXd b = bs[i];
          Eigen::SparseMatrix<double,Eigen::RowMajor> C = Cs[i];
          Eigen::VectorXd dl = dls[i];
          Eigen::VectorXd du = dus[i];
          if(A.cols() != 6 ||
             C.cols() != 6 ||
             A.rows() != b.size() ||
             C.rows() != dl.size() ||
             C.rows() != du.size()){
            std::cerr << "[" <<__FUNCTION__ << "] dimension mismatch" << std::endl;
            return false;
          }
          A_contacts.push_back(A);
          b_contacts.push_back(b);
          C_contacts.push_back(C);
          dl_contacts.push_back(dl);
          du_contacts.push_back(du);
        }
        appendDiag(A_contacts,A_contact);
        appendRow(b_contacts,b_contact);
        appendDiag(C_contacts,C_contact);
        appendRow(dl_contacts,dl_contact);
        appendRow(du_contacts,du_contact);
      }

      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(G.rows()+A_contact.rows(),G.cols());
      A.topRows(G.rows()) = G;
      A.bottomRows(A_contact.rows()) = A_contact;
      b = Eigen::VectorXd(h.size()+b_contact.size());
      b.head(h.rows()) = h;
      b.tail(b_contact.rows()) = b_contact;
      C = C_contact;
      dl = dl_contact;
      du = du_contact;
    }

    if(param.debugLevel){
      std::cerr << "before check" << std::endl;
      std::cerr << "A" << std::endl << A << std::endl;
      std::cerr << "b" << std::endl << b << std::endl;
      std::cerr << "C" << std::endl << C << std::endl;
      std::cerr << "dl" << std::endl << dl << std::endl;
      std::cerr << "du" << std::endl << du << std::endl;
    }

    // initialize solver
    // lbM <= Mx <= ubM
    // lb <= x <= ub
    Eigen::SparseMatrix<double,Eigen::RowMajor> M(A.rows()+C.rows(),A.cols());
    M.topRows(A.rows()) = A;
    M.bottomRows(C.rows()) = C;
    Eigen::VectorXd lbM(A.rows()+C.rows());
    lbM.head(b.rows()) = b;
    lbM.tail(dl.rows()) = dl;
    Eigen::VectorXd ubM(A.rows()+C.rows());
    ubM.head(b.rows()) = b;
    ubM.tail(du.rows()) = du;
    Eigen::VectorXd lb(A.cols());
    for(size_t i=0;i<lb.rows();i++) lb[i] = -std::numeric_limits<double>::max();
    Eigen::VectorXd ub(A.cols());
    for(size_t i=0;i<ub.rows();i++) ub[i] = std::numeric_limits<double>::max();

    Eigen::VectorXd o = Eigen::VectorXd::Zero(M.cols());
    clpeigen::solver solver;
    solver.initialize(o,M,lbM,ubM,lb,ub,param.debugLevel);
    solver.model().setPrimalTolerance(param.lpTolerance);//default 1e-7. vertexを見逃さないようにする
    solver.model().setDualTolerance(param.lpTolerance);

    Eigen::VectorXd solution;
    o.head<2>() = Eigen::Vector2d(1,0); // 適当
    solver.updateObjective(o);
    if (solver.solve()) {
      return true;
    } else {
      if(param.debugLevel) std::cerr << "[" <<__FUNCTION__ << "] failed" << std::endl;
      return false;
    }

  }

}
