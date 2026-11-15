#include "rendering/BodyActor.h"
#include <BRepPrimAPI_MakeBox.hxx>
#include <QApplication>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkImageData.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkTexture.h>
#include <vtkWindowToImageFilter.h>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <vector>

int main(int argc,char** argv) {
    QApplication app(argc,argv);
    try {
        const bool interactive = argc < 2 || std::string(argv[1]) != "full";
        BodyActor body;body.setShape(BRepPrimAPI_MakeBox(30,40,50).Shape());
        vtkNew<vtkRenderer> renderer;renderer->AddActor(body.surfaceActor());renderer->ResetCamera();
        vtkNew<vtkRenderWindow> window;window->SetOffScreenRendering(1);window->SetMultiSamples(0);
        window->SetSize(800,600);window->AddRenderer(renderer);
        auto* camera=renderer->GetActiveCamera();
        for(int i=0;i<4;++i){body.updateMaterialPreview(camera);window->Render();}
        std::vector<double> frames;
        for(int i=0;i<40;++i) {
            camera->Azimuth(1.0);
            const auto start=std::chrono::steady_clock::now();
            body.updateMaterialPreview(camera, interactive);window->Render();
            frames.push_back(std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count());
        }
        std::sort(frames.begin(),frames.end());
        auto* shadow=body.surfaceActor()->GetProperty()->GetTexture("slShadowMap")->GetInput();
        if(shadow->GetDimensions()[0]!=(interactive?512:1024))throw std::runtime_error("Incorrect interactive shadow size");
        body.updateMaterialPreview(camera, false);window->Render();
        if(shadow->GetDimensions()[0]!=1024)throw std::runtime_error("Full shadow quality was not restored");
        const auto pixels=[&] {
            vtkNew<vtkWindowToImageFilter> capture;
            capture->SetInput(window);capture->SetInputBufferTypeToRGB();capture->ReadFrontBufferOff();
            capture->ShouldRerenderOff();capture->Update();
            const auto* data=static_cast<unsigned char*>(capture->GetOutput()->GetScalarPointer());
            return std::vector<unsigned char>(data,data+800*600*3);
        };
        const auto full=pixels();
        if(!std::any_of(full.begin(),full.end(),[](auto value){return value!=0;}))throw std::runtime_error("Empty rendered image");
        body.updateMaterialPreview(camera,true);window->Render();const auto low=pixels();
        double meanDifference=0;
        for(std::size_t i=0;i<low.size();++i)meanDifference+=std::abs(int(low[i])-int(full[i]));
        meanDifference/=low.size();
        if(meanDifference>5)throw std::runtime_error("Interactive shadows changed projector alignment or shading");
        body.updateMaterialPreview(camera,false);window->Render();
        if(pixels()!=full)throw std::runtime_error("Full-quality pixels changed after interaction");
        std::cout<<"{\"frames\":40,\"p50_ms\":"<<frames[20]<<",\"p95_ms\":"<<frames[38]
                 <<",\"interactive_mean_rgb_difference\":"<<meanDifference<<",\"restored_pixels_exact\":true}\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
