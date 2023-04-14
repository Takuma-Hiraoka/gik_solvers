#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_CNOIDSTATESPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_CNOIDSTATESPACE_H

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <cnoid/Body>

namespace global_inverse_kinematics_solver{

  OMPL_CLASS_FORWARD(CnoidRealVectorStateSpace); // GIKStateSpacePtrを定義 (shared_ptr)
  class CnoidRealVectorStateSpace : public ompl::base::RealVectorStateSpace{
  public:
    class StateType : public ompl::base::RealVectorStateSpace::StateType {
    public:
      StateType(const std::vector<cnoid::LinkPtr>& links) : links_(links) {}
      virtual void link2State();
      virtual void state2Link();
    protected:
      const std::vector<cnoid::LinkPtr>& links_;
    };
    CnoidRealVectorStateSpace(const std::vector<cnoid::LinkPtr>& links, unsigned int dim = 0)
      : RealVectorStateSpace(dim),
        links_(links)
    {
    }
    ompl::base::State *allocState() const override;
    protected:
      const std::vector<cnoid::LinkPtr> links_;
  };
  class CnoidSE3StateSpace : public ompl::base::SE3StateSpace{
  public:
    class StateType : public ompl::base::SE3StateSpace::StateType {
    public:
      StateType(const cnoid::LinkPtr& link) : link_(link) {}
      virtual void link2State();
      virtual void state2Link();
    protected:
      const cnoid::LinkPtr& link_;
    };
    CnoidSE3StateSpace(const cnoid::LinkPtr& link)
      : SE3StateSpace(),
        link_(link)
    {
    }
    ompl::base::State *allocState() const override;
    protected:
      const cnoid::LinkPtr link_;
  };

  void state2Link(const ompl::base::State *state);
  void link2State(const ompl::base::State *state);

  ompl::base::StateSpacePtr createAmbientSpace(const std::vector<cnoid::LinkPtr>& variables);
};

#endif
