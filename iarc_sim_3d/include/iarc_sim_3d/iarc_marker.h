#ifndef __IARC_MARKER_H__ 
#define __IARC_MARKER_H__ 

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Grid.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <geometry_msgs/Point.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
  namespace rendering
  {
    class IARCMarkerPlugin : public VisualPlugin
    {
      public:
        IARCMarkerPlugin();
        virtual ~IARCMarkerPlugin();
        void Load( VisualPtr _parent, sdf::ElementPtr _sdf );
        void Update();
      private:
        std::string model_name;
        event::ConnectionPtr cxn; 
        VisualPtr visual;

        DynamicLines* grid;
        DynamicLines* goal;
        DynamicLines* fail;
    };
  }
}

#endif
