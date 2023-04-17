#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace global_inverse_kinematics_solver_sample{
  void sample1_4limb();
  class sample1_4limbItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample1_4limbItem>("sample1_4limbItem"); }
  protected:
    virtual void main() override{ sample1_4limb(); return; }
  };
  typedef cnoid::ref_ptr<sample1_4limbItem> sample1_4limbItemPtr;

  void sample2_desk();
  class sample2_deskItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<sample2_deskItem>("sample2_deskItem"); }
  protected:
    virtual void main() override{ sample2_desk(); return; }
  };
  typedef cnoid::ref_ptr<sample2_deskItem> sample2_deskItemPtr;

  class GlobalInverseKinematicsSolverSamplePlugin : public cnoid::Plugin
  {
  public:

    GlobalInverseKinematicsSolverSamplePlugin() : Plugin("GlobalInverseKinematicsSolverSample")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      sample1_4limbItem::initializeClass(this);
      sample2_deskItem::initializeClass(this);
      return true;
    }
  };


}

CNOID_IMPLEMENT_PLUGIN_ENTRY(global_inverse_kinematics_solver_sample::GlobalInverseKinematicsSolverSamplePlugin)
