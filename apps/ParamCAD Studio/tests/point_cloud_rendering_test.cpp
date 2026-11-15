#include "core/PointCloudSelection.h"
#include "rendering/PointCloudActor.h"
#include <QCoreApplication>
#include <vtkActor.h>
#include <vtkDataArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <iostream>
#include <stdexcept>

void require(bool ok,const char* message) {if(!ok)throw std::runtime_error(message);}
int main(int argc,char** argv) {
    QCoreApplication app(argc,argv);
    try {
        PlyCloud cloud;cloud.positions={{0,0,0},{1,0,0},{2,0,0}};
        cloud.colors={{12,34,56},{78,90,12},{34,56,78}};
        PointCloudSelection selection;selection.setCloud(cloud);
        PointCloudActor actor;require(actor.setCloud(cloud),"set cloud");actor.setAnnotations(&selection);
        auto* base=vtkPolyDataMapper::SafeDownCast(actor.actor()->GetMapper())->GetInput();
        auto* overlay=vtkPolyDataMapper::SafeDownCast(actor.selectionActor()->GetMapper())->GetInput();
        auto* colors=base->GetPointData()->GetScalars();const auto initial=colors->GetMTime();
        selection.select({1});actor.setAnnotations(&selection);
        require(colors->GetMTime()==initial,"selecting a point must not rewrite all base colors");
        require(overlay->GetNumberOfPoints()==1 && actor.selectionActor()->GetVisibility(),"selection overlay must show one point");
        double point[3];overlay->GetPoint(0,point);require(point[0]==1,"overlay must use the selected source position");
        require(base->GetNumberOfPoints()==3 && !actor.selectionActor()->GetPickable(),"source picking IDs must stay stable");
        selection.select({2});actor.setAnnotations(&selection);overlay->GetPoint(0,point);
        require(point[0]==2 && colors->GetMTime()==initial,"moving selection updates only the overlay");
        require(selection.assign("plane","wall")>0,"assign region");actor.setAnnotations(&selection);
        require(colors->GetMTime()>initial,"membership edit refreshes base colors");
        actor.setAnnotations(nullptr);require(!actor.selectionActor()->GetVisibility(),"capture hides selection overlay");
        require(colors->GetComponent(2,0)==34 && colors->GetComponent(2,1)==56,"capture restores source colors");
        actor.setAnnotations(&selection);actor.setVisible(false);
        require(!actor.selectionActor()->GetVisibility(),"hidden cloud hides selection");
        actor.setVisible(true);require(actor.selectionActor()->GetVisibility(),"selection visibility restored");
        actor.clear();actor.setVisible(true);require(!actor.selectionActor()->GetVisibility(),"clear removes stale selection");
        require(actor.setCloud(cloud),"reload cloud");actor.setVisible(true);
        require(!actor.selectionActor()->GetVisibility(),"loading a new cloud never shows old selected points");
        std::cout<<"Point cloud rendering cache test passed\n";
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
