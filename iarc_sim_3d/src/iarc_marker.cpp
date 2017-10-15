#include "iarc_sim_3d/iarc_marker.h"
#include <iostream>

#define DIM 10

namespace gazebo{
    namespace rendering{

        IARCMarkerPlugin::IARCMarkerPlugin():
            VisualPlugin(),
            grid(nullptr),
            visual(nullptr)
        {

        }
        IARCMarkerPlugin::~IARCMarkerPlugin(){
            if(grid != nullptr){
                this->visual->DeleteDynamicLine(grid);
                grid = nullptr;
            }
        }
        void IARCMarkerPlugin::Update(){
            for(auto& l : {grid, goal, fail}){
                if(l != nullptr){
                    l->setVisibilityFlags(GZ_VISIBILITY_ALL);
                }
            }
            if(visual != nullptr)
                visual->SetVisible(true);
        }
        void IARCMarkerPlugin::Load(VisualPtr _parent, sdf::ElementPtr _sdf){

            visual = _parent;
            visual->SetScale(math::Vector3(1.0,1.0,1.0));

            auto scene = visual->GetScene();

            grid = this->visual->CreateDynamicLine(RENDERING_LINE_LIST);
            grid->setMaterial("IARC/WhiteLine");
            goal = this->visual->CreateDynamicLine(RENDERING_LINE_LIST);
            goal->setMaterial("IARC/GreenLine");
            fail = this->visual->CreateDynamicLine(RENDERING_LINE_LIST);
            fail->setMaterial("IARC/RedLine");

            const float z = 0.01;

            for(int d = -DIM+1; d <= DIM-1; ++d){
                // vertical lines
                grid->AddPoint(d,-DIM,z); //default white
                grid->AddPoint(d, DIM,z);
                // horizontal lines
                grid->AddPoint(-DIM,d,z);
                grid->AddPoint( DIM,d,z);
            }
            grid->AddPoint( DIM,-DIM,z);
            grid->AddPoint( DIM, DIM,z);
            grid->AddPoint(-DIM,-DIM,z);
            grid->AddPoint(-DIM, DIM,z);

            //boundaries
            goal->AddPoint(-DIM, DIM,z);
            goal->AddPoint( DIM, DIM,z);
            fail->AddPoint(-DIM,-DIM,z);
            fail->AddPoint( DIM,-DIM,z);

            grid->setVisibilityFlags(GZ_VISIBILITY_ALL);
            goal->setVisibilityFlags(GZ_VISIBILITY_ALL);
            fail->setVisibilityFlags(GZ_VISIBILITY_ALL);

            visual->SetVisible(true);

            cxn = event::Events::ConnectRender(
                    boost::bind(&IARCMarkerPlugin::Update, this));
        }

        GZ_REGISTER_VISUAL_PLUGIN(IARCMarkerPlugin);
    }
}
