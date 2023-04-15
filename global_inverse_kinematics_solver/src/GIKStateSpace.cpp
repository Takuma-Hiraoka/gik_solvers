#include <global_inverse_kinematics_solver/GIKStateSpace.h>
#include <global_inverse_kinematics_solver/GIKConstraint.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace global_inverse_kinematics_solver{

  void GIKStateSpace::setup() {
    if (setup_)
      return;

    if (si_ == nullptr)
      throw ompl::Exception("ompl::base::ConstrainedStateSpace::setup(): "
                            "Must associate a SpaceInformation object to the ConstrainedStateSpace via"
                            "setStateInformation() before use.");

    WrapperStateSpace::setup();

    setDelta(delta_);  // This makes some setup-related calls
    setup_ = true;

    setDelta(delta_);

    // Call again to make sure information propagates properly to both wrapper
    // and underlying space.
    WrapperStateSpace::setup();

    // vectorを使ったprojectionをしないので、以下をコメントアウトしている

    // // Check if stride length of underlying state variables is 1
    // auto *state = space_->allocState();
    // bool flag = true;
    // for (unsigned int i = 1; i < space_->getDimension() && flag; ++i)
    //   {
    //     std::size_t newStride = space_->getValueAddressAtIndex(state, i) - space_->getValueAddressAtIndex(state, i - 1);
    //     flag = newStride == 1;
    //   }
    // space_->freeState(state);

    // if (!flag)
    //   throw ompl::Exception("ompl::base::ConstrainedStateSpace::setup(): "
    //                         "Stride length of member variables != 1, cannot translate into dense vector.");

  }

};
