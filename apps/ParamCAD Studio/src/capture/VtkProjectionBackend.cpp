#include "capture/VtkProjectionBackend.h"

#include <QDir>
#include <QThread>
#include <QTimer>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkOpenGLRenderWindow.h>
#include <vtkOpenGLVertexBufferObject.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkShaderProperty.h>
#include <vtkTexture.h>
#include <vtkUniforms.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace capture {
namespace {
vtkSmartPointer<vtkMatrix4x4> vtkMatrix(const Matrix4& m) {
    auto result=vtkSmartPointer<vtkMatrix4x4>::New();
    result->DeepCopy(m.data());
    return result;
}

void applyCamera(vtkCamera* camera,const Device& device,const Matrix4& world,double near,double far) {
    camera->SetPosition(world[3],world[7],world[11]);
    camera->SetFocalPoint(world[3]+world[2],world[7]+world[6],world[11]+world[10]);
    camera->SetViewUp(-world[1],-world[5],-world[9]);
    camera->SetClippingRange(near,far);
    vtkNew<vtkMatrix4x4> projection;
    projection->Zero();
    projection->SetElement(0,0,2*device.K[0]/device.width);
    projection->SetElement(0,2,1-2*device.K[2]/device.width);
    projection->SetElement(1,1,2*device.K[4]/device.height);
    projection->SetElement(1,2,2*device.K[5]/device.height-1);
    projection->SetElement(2,2,-(far+near)/(far-near));
    projection->SetElement(2,3,-2*far*near/(far-near));
    projection->SetElement(3,2,-1);
    camera->SetExplicitProjectionTransformMatrix(projection);
    camera->SetUseExplicitProjectionTransformMatrix(true);
}

vtkSmartPointer<vtkTexture> texture(vtkImageData* data,bool interpolate=false) {
    auto result=vtkSmartPointer<vtkTexture>::New();
    result->SetInputData(data);
    result->SetColorModeToDirectScalars();
    result->SetInterpolate(interpolate);
    result->RepeatOff();
    result->EdgeClampOn();
    return result;
}

vtkSmartPointer<vtkImageData> imageData(const QImage& source) {
    const QImage image=source.convertToFormat(QImage::Format_RGBA8888);
    auto result=vtkSmartPointer<vtkImageData>::New();
    result->SetDimensions(image.width(),image.height(),1);
    result->AllocateScalars(VTK_UNSIGNED_CHAR,4);
    for(int y=0;y<image.height();++y)
        std::memcpy(result->GetScalarPointer(0,y,0),image.constScanLine(image.height()-1-y),
                    static_cast<std::size_t>(image.width())*4);
    return result;
}

void addMap(vtkProperty* property,const char* name,const QString& path) {
    QImage map;
    if (!path.isEmpty()) map=QImage(path);
    if(map.isNull()) {map=QImage(1,1,QImage::Format_RGBA8888);map.fill(Qt::white);}
    property->SetTexture(name,texture(imageData(map),true));
}

// These shaders evaluate incident projected light and a GGX material, not an
// albedo decal. World-space projection and a projector Z-buffer control visibility.
constexpr const char* fragmentDeclarations=R"glsl(
//VTK::PositionVC::Dec
in vec3 slWorldPosition;
in vec3 slWorldNormal;
in vec2 slSurfaceUV;
vec3 slLinear(vec3 c) {
  return mix(pow((c+0.055)/1.055,vec3(2.4)),c/12.92,lessThanEqual(c,vec3(0.04045)));
}
)glsl";

constexpr const char* illumination=R"glsl(
vec3 slBase=slBaseColor*slLinear(texture(slBaseMap,slSurfaceUV).rgb);
float slMetal=clamp(slMetallic*texture(slMetalMap,slSurfaceUV).r,0.0,1.0);
float slRough=clamp(slRoughness*texture(slRoughMap,slSurfaceUV).r,0.04,1.0);
vec3 slRadiance=slAmbient*slBase;
vec3 slP=(slWorldToProjector*vec4(slWorldPosition,1.0)).xyz;
if(slP.z>slNear && slP.z<slFar) {
  vec2 slPixel=vec2(slFx*slP.x/slP.z+slCx,slFy*slP.y/slP.z+slCy);
  vec2 slUV=slPixel/vec2(slWidth,slHeight);
  float slDepth=0.5*((slFar+slNear)/(slFar-slNear)-2.0*slFar*slNear/((slFar-slNear)*slP.z))+0.5;
  vec3 slPackedDepth=texture(slShadowMap,vec2(slUV.x,1.0-slUV.y)).rgb;
  float slShadow=dot(slPackedDepth,vec3(65536.0,256.0,1.0))/65793.0;
  vec3 slN=normalize(slWorldNormal);
  vec3 slL=normalize(slProjectorCenter-slWorldPosition);
  float slNL=max(dot(slN,slL),0.0);
  // A fixed bias cannot cover the depth spread of one shadow texel at grazing
  // angles, which shows up as projector-grid shaped self-shadow acne. The bias
  // grows with the local slope while staying small on front-facing surfaces.
  float slSlope=sqrt(max(1.0-slNL*slNL,0.0))/max(slNL,0.0001);
  float slTexelDepth=(slFar*slNear)/max((slFar-slNear)*slP.z*max(slShadowFy,1.0),1.0);
  float slBias=max(0.00003,2.0*slSlope*slTexelDepth);
  bool slVisible=all(greaterThanEqual(slUV,vec2(0.0))) && all(lessThan(slUV,vec2(1.0))) && slDepth<=slShadow+slBias;
  if(slVisible && slNL>0.0) {
    vec3 slV=normalize(slCameraCenter-slWorldPosition);
    float slNV=max(dot(slN,slV),0.0);
    if(slNV>0.0) {
      vec3 slH=normalize(slL+slV);
      float slNH=max(dot(slN,slH),0.0),slVH=max(dot(slV,slH),0.0);
      float slA=slRough*slRough,slA2=slA*slA;
      float slDen=slNH*slNH*(slA2-1.0)+1.0;
      float slD=slA2/(3.14159265359*slDen*slDen);
      float slK=(slRough+1.0)*(slRough+1.0)/8.0;
      float slG=(slNL/(slNL*(1.0-slK)+slK))*(slNV/(slNV*(1.0-slK)+slK));
      vec3 slF0=mix(vec3(0.04),slBase,slMetal);
      vec3 slF=slF0+(vec3(1.0)-slF0)*pow(1.0-slVH,5.0);
      vec3 slBRDF=(vec3(1.0)-slF)*(1.0-slMetal)*slBase/3.14159265359+slD*slG*slF/max(4.0*slNL*slNV,0.000001);
      float slSignal=texture(slPatternMap,vec2(slUV.x,1.0-slUV.y)).r;
      float slDistance2=dot(slProjectorCenter-slWorldPosition,slProjectorCenter-slWorldPosition);
      slRadiance+=slBRDF*slNL*slSignal*slIntensity*slReference2/max(slDistance2,0.000001);
    }
  }
}
gl_FragData[0]=vec4(clamp(slRadiance*slExposure,0.0,1.0),1.0);
)glsl";
}

struct VtkProjectionBackend::State {
    RenderJob job;
    std::vector<Frame> frames;
    std::size_t next=0;
    bool running=false,cancelled=false;
    int shadowStation=-1;
    int shadowWidth=0,shadowHeight=0;
    float shadowFy=0;
    vtkSmartPointer<vtkRenderWindow> window,shadowWindow;
    vtkSmartPointer<vtkRenderer> renderer,shadowRenderer;
    vtkSmartPointer<vtkActor> actor;
    vtkSmartPointer<vtkImageData> patternData,shadowData;
};

VtkProjectionBackend::VtkProjectionBackend(QObject* parent):RenderBackend(parent),m_state(std::make_unique<State>()) {}
VtkProjectionBackend::~VtkProjectionBackend()=default;
QJsonObject VtkProjectionBackend::capabilities() const {
    return {{"backend","vtk_projection"},{"version",vtkVersion::GetVTKVersion()},
            {"illumination","direct"},{"indirect_reflection",false},
            {"pixel_convention","top-left; centers=(i+0.5,j+0.5)"},{"intensity_encoding","linear_uint8"},
            {"material_model","GGX metallic-roughness"},{"shadow_mapping",true},
            {"shadow_depth_encoding","RGB24"},{"shadow_depth_bias",0.00003},
            {"shadow_bias_model","constant + slope-scaled"},{"shadow_map_min_extent",1024},
            {"samples_per_pixel",1},{"random_seed",0},{"max_surface_bounces",1},
            {"pattern_filter","nearest Gray; linear phase"}};
}
QString VtkProjectionBackend::validate(const RenderJob& job) const {return job.validate();}
void VtkProjectionBackend::cancel() {m_state->cancelled=true;}

void VtkProjectionBackend::start(const RenderJob& job) {
    if(m_state->running) {emit finished(false,"A rendering job is already active");return;}
    if(QThread::currentThread()!=thread()) {emit finished(false,"VTK capture must run on its owning thread");return;}
    const QString error=validate(job);
    if(!error.isEmpty()) {emit finished(false,error);return;}
    m_state=std::make_unique<State>();
    auto& state=*m_state;
    state.job=job;state.frames=job.frames();state.running=true;
    // The projector grid limits shadow-map detail even when the camera image is
    // larger. Scale the depth pass to at least 1024 texels per axis while keeping
    // the projector aspect ratio and field of view unchanged.
    const int shadowExtent=std::max(job.projector.width,job.projector.height);
    const int shadowScale=std::max(1,(1024+shadowExtent-1)/shadowExtent);
    state.shadowWidth=job.projector.width*shadowScale;
    state.shadowHeight=job.projector.height*shadowScale;
    state.shadowFy=static_cast<float>(job.projector.K[4]*shadowScale);
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    vtkNew<vtkFloatArray> normals;
    normals->SetNumberOfComponents(3);
    vtkNew<vtkFloatArray> uv;
    uv->SetNumberOfComponents(2);uv->SetName("captureUV");
    for(std::size_t i=0;i<job.geometry.positions.size();++i) {
        points->InsertNextPoint(job.geometry.positions[i].data());
        normals->InsertNextTuple(job.geometry.normals[i].data());
        uv->InsertNextTuple(job.geometry.uv[i].data());
    }
    vtkNew<vtkCellArray> cells;
    for(const auto& face:job.geometry.triangles) {
        const vtkIdType triangle[3]{face[0],face[1],face[2]};cells->InsertNextCell(3,triangle);
    }
    vtkNew<vtkPolyData> mesh;
    mesh->SetPoints(points);mesh->SetPolys(cells);mesh->GetPointData()->SetNormals(normals);
    mesh->GetPointData()->SetTCoords(uv);
    vtkNew<vtkOpenGLPolyDataMapper> mapper;
    mapper->SetInputData(mesh);mapper->ScalarVisibilityOff();
    mapper->SetVBOShiftScaleMethod(vtkOpenGLVertexBufferObject::DISABLE_SHIFT_SCALE);
    state.actor=vtkSmartPointer<vtkActor>::New();state.actor->SetMapper(mapper);
    state.actor->GetProperty()->SetInterpolationToPhong();
    auto* shader=state.actor->GetShaderProperty();
    shader->AddVertexShaderReplacement("//VTK::PositionVC::Dec",true,
        "//VTK::PositionVC::Dec\nout vec3 slWorldPosition;\nout vec3 slWorldNormal;\nout vec2 slSurfaceUV;\n",false);
    shader->AddVertexShaderReplacement("//VTK::PositionVC::Impl",true,
        "//VTK::PositionVC::Impl\nslWorldPosition=vertexMC.xyz;\nslWorldNormal=normalMC;\nslSurfaceUV=tcoord;\n",false);
    shader->AddFragmentShaderReplacement("//VTK::PositionVC::Dec",true,fragmentDeclarations,false);
    shader->AddFragmentShaderReplacement("//VTK::Light::Impl",true,illumination,false);
    shader->AddFragmentShaderReplacement("//VTK::TCoord::Impl",true,"",false);
    auto* uniforms=shader->GetFragmentCustomUniforms();
    uniforms->SetUniform3f("slBaseColor",job.material.baseColor.data());
    uniforms->SetUniformf("slMetallic",static_cast<float>(job.material.metallic));
    uniforms->SetUniformf("slRoughness",static_cast<float>(job.material.roughness));
    uniforms->SetUniformf("slAmbient",static_cast<float>(job.ambient));
    uniforms->SetUniformf("slNear",static_cast<float>(job.nearPlane));
    uniforms->SetUniformf("slFar",static_cast<float>(job.farPlane));
    uniforms->SetUniformf("slFx",static_cast<float>(job.projector.K[0]));
    uniforms->SetUniformf("slFy",static_cast<float>(job.projector.K[4]));
    uniforms->SetUniformf("slShadowFy",state.shadowFy);
    uniforms->SetUniformf("slCx",static_cast<float>(job.projector.K[2]));
    uniforms->SetUniformf("slCy",static_cast<float>(job.projector.K[5]));
    uniforms->SetUniformf("slWidth",static_cast<float>(job.projector.width));
    uniforms->SetUniformf("slHeight",static_cast<float>(job.projector.height));
    uniforms->SetUniformf("slIntensity",static_cast<float>(job.lightIntensity));
    uniforms->SetUniformf("slReference2",static_cast<float>(job.referenceDistance*job.referenceDistance));
    uniforms->SetUniformf("slExposure",1);
    vtkNew<vtkMatrix4x4> initial;
    uniforms->SetUniformMatrix("slWorldToProjector",initial);
    const double center[3]{0,0,0};
    uniforms->SetUniform3f("slProjectorCenter",center);uniforms->SetUniform3f("slCameraCenter",center);
    addMap(state.actor->GetProperty(),"slBaseMap",job.material.baseColorMap);
    addMap(state.actor->GetProperty(),"slMetalMap",job.material.metallicMap);
    addMap(state.actor->GetProperty(),"slRoughMap",job.material.roughnessMap);
    state.patternData=imageData(patternImage(job.patterns.front(),job.projector.width,job.projector.height,job.phasePeriod));
    state.actor->GetProperty()->SetTexture("slPatternMap",texture(state.patternData));
    state.shadowData=vtkSmartPointer<vtkImageData>::New();
    state.shadowData->SetDimensions(state.shadowWidth,state.shadowHeight,1);
    // Pack Z into three bytes: ordinary normalized texture storage otherwise
    // permits driver-dependent float precision or scalar lookup conversion.
    state.shadowData->AllocateScalars(VTK_UNSIGNED_CHAR,3);
    std::fill_n(static_cast<unsigned char*>(state.shadowData->GetScalarPointer()),
                state.shadowWidth*state.shadowHeight*3,static_cast<unsigned char>(255));
    state.actor->GetProperty()->SetTexture("slShadowMap",texture(state.shadowData));
    state.renderer=vtkSmartPointer<vtkRenderer>::New();state.renderer->SetBackground(0,0,0);
    state.renderer->UseFXAAOff();state.renderer->AddActor(state.actor);
    state.window=vtkSmartPointer<vtkRenderWindow>::New();
    state.window->SetOffScreenRendering(1);state.window->SetMultiSamples(0);
    state.window->SetSize(job.camera.width,job.camera.height);state.window->AddRenderer(state.renderer);
    if(auto* gl=vtkOpenGLRenderWindow::SafeDownCast(state.window))gl->SetUseSRGBColorSpace(false);
    vtkNew<vtkOpenGLPolyDataMapper> shadowMapper;shadowMapper->SetInputData(mesh);shadowMapper->ScalarVisibilityOff();
    vtkNew<vtkActor> shadowActor;shadowActor->SetMapper(shadowMapper);
    state.shadowRenderer=vtkSmartPointer<vtkRenderer>::New();state.shadowRenderer->AddActor(shadowActor);
    state.shadowWindow=vtkSmartPointer<vtkRenderWindow>::New();state.shadowWindow->SetOffScreenRendering(1);
    state.shadowWindow->SetMultiSamples(0);state.shadowWindow->SetSize(state.shadowWidth,state.shadowHeight);
    state.shadowWindow->AddRenderer(state.shadowRenderer);
    QTimer::singleShot(0,this,&VtkProjectionBackend::nextFrame);
}

void VtkProjectionBackend::nextFrame() {
    auto& state=*m_state;
    if(!state.running)return;
    if(state.cancelled || state.next==state.frames.size()) {
        state.running=false;state.window=nullptr;state.shadowWindow=nullptr;
        emit finished(state.cancelled,{});return;
    }
    const auto frame=state.frames[state.next];
    const auto& job=state.job;
    const auto cameraWorld=multiply(job.stations[frame.station].rigToWorld,job.camera.deviceToRig);
    const auto projectorWorld=multiply(job.stations[frame.station].rigToWorld,job.projector.deviceToRig);
    if(state.shadowStation!=frame.station) {
        applyCamera(state.shadowRenderer->GetActiveCamera(),job.projector,projectorWorld,job.nearPlane,job.farPlane);
        state.shadowWindow->Render();
        vtkNew<vtkWindowToImageFilter> depth;
        depth->SetInput(state.shadowWindow);depth->SetInputBufferTypeToZBuffer();depth->ReadFrontBufferOff();
        depth->ShouldRerenderOff();depth->Update();
        const auto* source=static_cast<float*>(depth->GetOutput()->GetScalarPointer());
        auto* destination=static_cast<unsigned char*>(state.shadowData->GetScalarPointer());
        for(int i=0;i<state.shadowWidth*state.shadowHeight;++i) {
            const auto packed=static_cast<unsigned int>(std::llround(std::clamp(source[i],0.f,1.f)*16777215.0));
            destination[3*i]=static_cast<unsigned char>(packed>>16);
            destination[3*i+1]=static_cast<unsigned char>((packed>>8)&255);
            destination[3*i+2]=static_cast<unsigned char>(packed&255);
        }
        state.shadowData->Modified();
        state.shadowStation=frame.station;
    }
    applyCamera(state.renderer->GetActiveCamera(),job.camera,cameraWorld,job.nearPlane,job.farPlane);
    auto worldToProjector=vtkMatrix(projectorWorld);worldToProjector->Invert();
    // vtkUniforms' flat matrix upload uses OpenGL column-major ordering.
    worldToProjector->Transpose();
    auto* uniforms=state.actor->GetShaderProperty()->GetFragmentCustomUniforms();
    uniforms->SetUniformMatrix("slWorldToProjector",worldToProjector);
    const double projectorCenter[3]{projectorWorld[3],projectorWorld[7],projectorWorld[11]};
    const double cameraCenter[3]{cameraWorld[3],cameraWorld[7],cameraWorld[11]};
    uniforms->SetUniform3f("slProjectorCenter",projectorCenter);uniforms->SetUniform3f("slCameraCenter",cameraCenter);
    uniforms->SetUniformf("slExposure",static_cast<float>(job.exposures[frame.exposure]));
    state.patternData->DeepCopy(imageData(patternImage(job.patterns[frame.pattern],job.projector.width,
                                                       job.projector.height,job.phasePeriod)));
    state.actor->GetProperty()->GetTexture("slPatternMap")->SetInterpolate(job.patterns[frame.pattern].kind=="phase");
    state.window->Render();
    vtkNew<vtkWindowToImageFilter> capture;
    capture->SetInput(state.window);capture->SetInputBufferTypeToRGB();capture->ReadFrontBufferOff();
    capture->ShouldRerenderOff();capture->Update();
    auto* pixels=capture->GetOutput();
    QImage image(job.camera.width,job.camera.height,QImage::Format_RGB888);
    for(int y=0;y<job.camera.height;++y)
        std::memcpy(image.scanLine(y),pixels->GetScalarPointer(0,job.camera.height-1-y,0),
                    static_cast<std::size_t>(job.camera.width)*3);
    const QString path=QDir(job.directory).filePath(frame.image);
    if(!QDir().mkpath(QFileInfo(path).absolutePath()) || !image.save(path,"PNG")) {
        state.running=false;emit finished(false,"Cannot save captured frame: "+path);return;
    }
    ++state.next;
    emit frameCompleted(frame.station,frame.exposure,frame.pattern,frame.image);
    emit progress(static_cast<int>(state.next),static_cast<int>(state.frames.size()));
    QTimer::singleShot(0,this,&VtkProjectionBackend::nextFrame);
}
} // namespace capture
