#ifndef AEPLANNER_NODELET_H
#define AEPLANNER_NODELET_H

#include <nodelet/nodelet.h>

#include <aeplanner/aeplanner.h>

namespace aeplanner
{
  class AEPlannerNodelet : public nodelet::Nodelet
  {
    private:
      AEPlanner * aeplanner_;

    public:
      virtual void onInit();
      ~AEPlannerNodelet();
  };
}

# endif
