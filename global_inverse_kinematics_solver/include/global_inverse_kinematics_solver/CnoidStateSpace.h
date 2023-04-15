#ifndef GLOBAL_INVERSE_KINEMATICS_SOLVER_CNOIDSTATESPACE_H
#define GLOBAL_INVERSE_KINEMATICS_SOLVER_CNOIDSTATESPACE_H

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <cnoid/Body>

namespace global_inverse_kinematics_solver{

  OMPL_CLASS_FORWARD(CnoidRealVectorStateSpace); // StateSpacePtrを定義 (shared_ptr)
  class CnoidRealVectorStateSpace : public ompl::base::RealVectorStateSpace{
  public:
    CnoidRealVectorStateSpace(const std::vector<cnoid::LinkPtr>& links, unsigned int dim = 0)
      : RealVectorStateSpace(dim),
        links_(links)
    {
    }
    virtual void link2State(ompl::base::State* state);
    virtual void state2Link(const ompl::base::State* state);
    const std::vector<cnoid::LinkPtr>& links() const { return links_; }
    protected:
      const std::vector<cnoid::LinkPtr> links_;
  };
  OMPL_CLASS_FORWARD(CnoidSE3StateSpace); // StateSpacePtrを定義 (shared_ptr)
  class CnoidSE3StateSpace : public ompl::base::SE3StateSpace{
  public:
    CnoidSE3StateSpace(const cnoid::LinkPtr& link)
      : SE3StateSpace(),
        link_(link)
    {
    }
    virtual void link2State(ompl::base::State* state);
    virtual void state2Link(const ompl::base::State* state);
    const cnoid::LinkPtr& link() const { return link_; }
    protected:
      const cnoid::LinkPtr link_;
  };

  void state2Link(const ompl::base::StateSpacePtr& space, const ompl::base::State *state);
  void link2State(const ompl::base::StateSpacePtr& space, ompl::base::State *state);
  std::vector<cnoid::LinkPtr> getLinks(const ompl::base::StateSpacePtr& space);
  void getLinks(const ompl::base::StateSpacePtr& space, std::vector<cnoid::LinkPtr>& links);

  ompl::base::StateSpacePtr createAmbientSpace(const std::vector<cnoid::LinkPtr>& variables);
};

#endif
