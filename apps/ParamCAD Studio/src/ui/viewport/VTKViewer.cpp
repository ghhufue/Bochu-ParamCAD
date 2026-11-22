#include "ui/viewport/VTKViewer.h"
#include <vtkMath.h>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include <QByteArray>
#include <QSize>

#include <vtkActor.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDataArray.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkUnsignedCharArray.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkImageResize.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cmath>

capture::Geometry VTKViewer::captureGeometry() const
{
    capture::Geometry result;
    if(!hasBody())return result;
    auto* actor=m_bodyActor.surfaceActor();
    auto* mapper=vtkPolyDataMapper::SafeDownCast(actor->GetMapper());
    if(!mapper)return result;
    mapper->Update();auto* mesh=mapper->GetInput();
    if(!mesh || !mesh->GetPointData()->GetNormals())return result;
    auto* transform=actor->GetMatrix();
    vtkNew<vtkMatrix4x4> normalTransform;
    vtkMatrix4x4::Invert(transform,normalTransform);normalTransform->Transpose();
    double bounds[6];mesh->GetBounds(bounds);
    // Real drawings contain degenerate faces whose points end up without a
    // usable normal. Drop those points and the triangles that reference them
    // instead of discarding the whole body.
    constexpr std::uint32_t invalid=~std::uint32_t{0};
    std::vector<std::uint32_t> remap(static_cast<std::size_t>(mesh->GetNumberOfPoints()),invalid);
    for(vtkIdType i=0;i<mesh->GetNumberOfPoints();++i) {
        double p[4]{0,0,0,1},world[4],n[4]{0,0,0,0},normal[4];
        mesh->GetPoint(i,p);transform->MultiplyPoint(p,world);
        mesh->GetPointData()->GetNormals()->GetTuple(i,n);normalTransform->MultiplyPoint(n,normal);
        const double length=std::sqrt(normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2]);
        if(!std::isfinite(length) || length<=0)continue;
        remap[static_cast<std::size_t>(i)]=static_cast<std::uint32_t>(result.positions.size());
        result.positions.push_back({world[0],world[1],world[2]});
        result.normals.push_back({normal[0]/length,normal[1]/length,normal[2]/length});
        int axis=0;for(int j=1;j<3;++j)if(std::abs(n[j])>std::abs(n[axis]))axis=j;
        const int u=(axis+1)%3,v=(axis+2)%3;
        result.uv.push_back({(p[u]-bounds[2*u])/std::max(bounds[2*u+1]-bounds[2*u],1e-9),
                             (p[v]-bounds[2*v])/std::max(bounds[2*v+1]-bounds[2*v],1e-9)});
    }
    vtkIdType count=0;const vtkIdType* indices=nullptr;
    mesh->GetPolys()->InitTraversal();
    while(mesh->GetPolys()->GetNextCell(count,indices)) {
        if(count!=3)return {};
        const auto a=remap[static_cast<std::size_t>(indices[0])];
        const auto b=remap[static_cast<std::size_t>(indices[1])];
        const auto c=remap[static_cast<std::size_t>(indices[2])];
        if(a==invalid || b==invalid || c==invalid)continue;
        result.triangles.push_back({a,b,c});
    }
    return result;
}

namespace {

// Gate the window as well as our timer, since the Qt adapter and VTK
// interactor can request a frame without calling VTKViewer::renderNow().
class ViewportRenderWindow final : public vtkGenericOpenGLRenderWindow
{
public:
    static ViewportRenderWindow* New();
    vtkTypeMacro(ViewportRenderWindow, vtkGenericOpenGLRenderWindow);

    void setActive(bool active) { m_active = active; }
    bool active() const { return m_active; }
    void Render() override { if (m_active) Superclass::Render(); }

private:
    bool m_active = true;
};

vtkStandardNewMacro(ViewportRenderWindow);

constexpr double kPiOver180 = 3.14159265358979323846 / 180.0;
constexpr int kCameraPanelIntervalMs = 50;

// How far from a sketch entity a click still counts as hitting it, in screen
// pixels. Sketch lines are hairline-thin, so a few pixels is what makes them
// selectable with an ordinary mouse while staying precise.
constexpr double kSketchPickPixelRadius = 6.0;

// A drawing tool anchors on an existing vertex from this many pixels away. It is
// a little wider than a pick so snapping feels sticky without being grabby.
constexpr double kSketchSnapPixelRadius = 12.0;

// Extent of the 2D sketch view. The datum planes are drawn 80 mm across, so this
// frames one of them with a small margin.
constexpr double kSketchViewHalfExtent = 44.0;

// Distance of the sketch camera from the plane. An orthographic projection
// ignores it for scale, so it only has to sit clear of the geometry.
constexpr double kSketchViewDistance = 1000.0;

void cross(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

bool normalize(double v[3])
{
    const double length = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (length <= 1.0e-12) {
        return false;
    }
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
    return true;
}

}  // namespace

VTKViewer::VTKViewer(QWidget* parent, PointCloudDocument* document, bool renderingActive)
    : QVTKOpenGLNativeWidget(parent)
    , m_renderWindow(vtkSmartPointer<ViewportRenderWindow>::New())
    , m_renderer(vtkSmartPointer<vtkRenderer>::New())
    , m_referenceRenderer(vtkSmartPointer<vtkRenderer>::New())
    , m_pointCloudDocument(document ? document : new PointCloudDocument(this))
{
    static_cast<ViewportRenderWindow*>(m_renderWindow.GetPointer())->setActive(renderingActive);
    setObjectName(QStringLiteral("vtkViewport"));
    setMinimumSize(480, 320);
    setFocusPolicy(Qt::StrongFocus);

    m_renderTimer.setSingleShot(true);
    m_renderTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_renderTimer, &QTimer::timeout, this, &VTKViewer::renderNow);
    m_cameraNotifyTimer.setSingleShot(true);
    m_cameraNotifyTimer.setTimerType(Qt::PreciseTimer);
    connect(&m_cameraNotifyTimer, &QTimer::timeout, this, [this] {
        if (m_cameraDirty) publishCameraChange();
    });

    setRenderWindow(m_renderWindow);
    m_renderWindow->AddRenderer(m_renderer);
    m_renderer->SetBackground(0.10, 0.11, 0.13);
    m_renderer->SetBackground2(0.17, 0.18, 0.21);
    m_renderer->GradientBackgroundOn();

    // SolidWorks-style navigation: the left button is reserved for picking and
    // placing geometry, so the camera lives on the buttons that stay held down -
    // the middle one rotates, the right one pans - and on the wheel.
    m_interactorStyle = vtkSmartPointer<SketchInteractorStyle>::New();
    m_interactorStyle->leftButtonPressed = [this](int x, int y) {
        onLeftButtonPressed(x, y);
    };
    m_interactorStyle->mouseMoved = [this](int x, int y) { onMouseMoved(x, y); };
    m_interactorStyle->cancelRequested = [this]() { onCancelRequested(); };
    m_interactorStyle->panMoved = [this]() {
        if (!m_boundaryInspectionRenderer) emit cameraPanned();
    };
    m_interactorStyle->cameraInteractionStarted = [this]() { beginCameraInteraction(); };
    m_interactorStyle->cameraInteractionFinished = [this]() { endCameraInteraction(); };
    m_interactorStyle->renderRequested = [this]() { requestRender(); };
    m_interactorStyle->keyPressed = [this](const std::string& keySym) {
        onKeyPressed(keySym);
    };
    m_renderWindow->GetInteractor()->SetInteractorStyle(m_interactorStyle);

    m_renderer->AddActor(m_bodyActor.surfaceActor());
    m_renderer->AddActor(m_bodyActor.edgeActor());
    m_renderer->AddActor(m_bodyActor.highlightActor());
    m_renderer->AddActor(m_pointCloudActor.actor());
    m_renderer->AddActor(m_pointCloudActor.selectionActor());
    m_renderer->AddActor(m_pointCloudActor.surfaceActor());
    m_renderer->AddActor(m_pointCloudActor.surfaceHighlightActor());
    m_assemblyBoundaryActor=vtkSmartPointer<vtkActor>::New();m_assemblyBoundaryActor->SetPickable(false);
    m_assemblyBoundaryActor->GetProperty()->SetLineWidth(1.5);m_assemblyBoundaryActor->GetProperty()->LightingOff();
    m_assemblyBoundaryActor->SetVisibility(false);m_renderer->AddActor(m_assemblyBoundaryActor);
    m_renderer->AddActor(m_sketchActor.actor());
    m_renderer->AddActor(m_sketchActor.draftActor());
    m_datumPlanes.attach(m_renderer);
    createOrientationAxes();
    m_renderWindow->SetNumberOfLayers(3);
    m_referenceRenderer->SetLayer(2);
    m_referenceRenderer->InteractiveOff();
    m_referenceRenderer->SetActiveCamera(m_renderer->GetActiveCamera());
    m_renderWindow->AddRenderer(m_referenceRenderer);
    m_boxReferenceActor.attach(m_referenceRenderer);
    m_projectionPreview.attach(m_renderer, m_referenceRenderer);
    m_cadSolidPreview.attach(m_renderer, m_referenceRenderer);

    CameraParameters initialCamera;
    CameraController::apply(m_renderer->GetActiveCamera(), initialCamera);

    m_cameraCallback = vtkSmartPointer<vtkCallbackCommand>::New();
    m_cameraCallback->SetClientData(this);
    m_cameraCallback->SetCallback(&VTKViewer::onCameraModified);
    m_cameraObserverTag = m_renderer->GetActiveCamera()->AddObserver(
        vtkCommand::ModifiedEvent, m_cameraCallback);

    connect(m_pointCloudDocument, &PointCloudDocument::cloudChanged,
            this, &VTKViewer::syncPointCloud);
    connect(m_pointCloudDocument, &PointCloudDocument::documentChanged, this, [this] {
        const auto surfaceRevision = pointCloudSelection().surfaceRevision();
        if (m_boundaryInspectionRenderer && m_boundaryInspectionRevision != surfaceRevision)
            closeBoundaryInspection();
        m_pointCloudActor.setPointMask(pointCloudSelection().deleted());
        if(m_boundaryPointCount && (m_boundaryPointCount!=m_pointCloudDocument->pointCount() ||
            m_boundaryRevision!=surfaceRevision))setAssemblyBoundaries({},{});
        if (!m_boundaryInspectionRenderer) m_renderer->ResetCameraClippingRange();
    });
    connect(m_pointCloudDocument, &PointCloudDocument::selectionChanged,
            this, &VTKViewer::refreshPointCloudSelection);
    if (m_pointCloudDocument->hasCloud()) syncPointCloud();
    renderNow();
}
VTKViewer::~VTKViewer()
{
    if (m_renderer && m_renderer->GetActiveCamera() && m_cameraObserverTag != 0) {
        m_renderer->GetActiveCamera()->RemoveObserver(m_cameraObserverTag);
    }
}

CameraParameters VTKViewer::cameraParameters() const
{
    return CameraController::parameters(m_renderer->GetActiveCamera());
}

void VTKViewer::setBodyShape(const TopoDS_Shape& shape)
{
    clearProjectionPreview();
    endBoxReferenceSelection();
    m_bodyActor.setShape(shape);
    m_displayedBody = shape;
    m_pickedFaceId = -1;
    ++m_bodyRevision;
    m_renderer->ResetCameraClippingRange();
    emit bodyShapeChanged(shape);
}

void VTKViewer::setProjectionPreviewBody(const TopoDS_Shape& shape, double scaleToCloud)
{
    m_projectionPreview.setBody(shape, scaleToCloud);
    m_hasProjectionPreview = true;
    m_renderer->ResetCameraClippingRange();
    requestRender();
}

void VTKViewer::setProjectionPreviewOperations(const std::vector<reconstruction::ProjectionOperation>& operations,
                                               double scaleToCloud, int selectedStep)
{
    m_projectionPreview.setOperations(operations, m_pointCloudDocument->sourceCloud(), scaleToCloud, selectedStep);
    m_hasProjectionPreview = true;
    requestRender();
}

void VTKViewer::setProjectionPreviewBodyVisible(bool visible)
{
    m_projectionPreview.setBodyVisible(visible);
    m_renderer->ResetCameraClippingRange();
    requestRender();
}

void VTKViewer::clearProjectionPreview()
{
    if (!m_hasProjectionPreview) return;
    m_projectionPreview.clear();
    m_hasProjectionPreview = false;
    requestRender();
}

void VTKViewer::updateCadPreview(bool resetSurfaces,
    const std::vector<std::shared_ptr<const PointCloudSurfaceMesh>>& surfaces,
    const std::optional<TopoDS_Shape>& solid, double scaleToCloud)
{
    if (resetSurfaces) {
        for (const auto& actor : m_cadSurfacePreviews) m_renderer->RemoveActor(actor);
        m_cadSurfacePreviews.clear();
    }
    for (const auto& mesh : surfaces) {
        auto actor = PointCloudActor::createSurfacePreview(*mesh, int(m_cadSurfacePreviews.size()) + 1);
        actor->SetVisibility(m_pointCloudVisible);
        m_renderer->AddActor(actor);
        m_cadSurfacePreviews.push_back(std::move(actor));
    }
    if (solid) {
        m_hasCadSolidPreview = !solid->IsNull();
        if (m_hasCadSolidPreview) {
            m_cadSolidPreview.setBody(*solid, scaleToCloud);
            m_cadSolidPreview.setBodyVisible(m_pointCloudVisible);
        } else {
            m_cadSolidPreview.clear();
        }
        m_renderer->ResetCameraClippingRange();
    }
    requestRender();
}

void VTKViewer::clearCadPreview()
{
    if (m_cadSurfacePreviews.empty() && !m_hasCadSolidPreview) return;
    updateCadPreview(true, {}, TopoDS_Shape{}, 1.0);
}

void VTKViewer::clearBody()
{
    clearProjectionPreview();
    endBoxReferenceSelection();
    m_bodyActor.clear();
    m_displayedBody.Nullify();
    m_pickedFaceId = -1;
    ++m_bodyRevision;
    m_renderer->ResetCameraClippingRange();
    emit bodyShapeChanged(TopoDS_Shape{});
}

bool VTKViewer::hasBody() const noexcept
{
    return m_bodyActor.hasShape();
}

bool VTKViewer::beginBoxReferenceSelection(const modeling::BoxPrimitiveParams& box)
{
    if (m_sketchInteraction || !hasBody() || !objectVisible()) return false;
    closeBoundaryInspection();
    m_boxReferenceActor.setBox(box);
    m_boxReferenceActor.setTransform(m_bodyActor.surfaceActor()->GetMatrix());
    m_sketchActor.actor()->SetUserMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    m_sketchActor.draftActor()->SetUserMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    m_referenceSelection=true;
    m_datumPlanes.setVisible(false);
    renderNow();
    return true;
}

void VTKViewer::endBoxReferenceSelection()
{
    if (!m_referenceSelection) return;
    m_referenceSelection=false;
    m_boxReferenceActor.setVisible(false);
    m_datumPlanes.setVisible(m_datumPlanesVisible);
    renderNow();
}

void VTKViewer::renderNow()
{
    renderFrame(m_cameraInteracting);
}

bool VTKViewer::renderingActive() const noexcept
{
    return static_cast<const ViewportRenderWindow*>(m_renderWindow.GetPointer())->active();
}

void VTKViewer::setRenderingActive(bool active)
{
    auto* window = static_cast<ViewportRenderWindow*>(m_renderWindow.GetPointer());
    if (window->active() == active) return;
    window->setActive(active);
    if (active) {
        requestRender();
    } else {
        m_renderTimer.stop();
        endCameraInteraction();
    }
}

void VTKViewer::renderFrame(bool interacting)
{
    m_renderTimer.stop();
    if (!renderingActive()) return;
    // The preview light rides on the camera, so its pose and shadow map are
    // refreshed from the pose the frame is about to be drawn with.
    if (!m_boundaryInspectionRenderer)
        m_bodyActor.updateMaterialPreview(m_renderer->GetActiveCamera(), interacting);
    m_interactionFrameRendered = interacting;
    m_renderWindow->Render();
}

void VTKViewer::requestRender()
{
    if (!renderingActive() || m_renderTimer.isActive()) return;
    // Merge requests already in the Qt event queue, without imposing a frame
    // interval. Qt/OpenGL presentation can then use the monitor's full refresh
    // rate instead of being held below it by a 17 ms software timer.
    m_renderTimer.start(0);
}

void VTKViewer::publishCameraChange()
{
    m_cameraNotifyTimer.stop();
    m_cameraDirty = false;
    emit cameraChanged(cameraParameters());
}

void VTKViewer::beginCameraInteraction()
{
    if (m_cameraDirty) publishCameraChange();
    m_cameraInteracting = true;
}

void VTKViewer::endCameraInteraction()
{
    m_cameraInteracting = false;
    // Deliver the exact final pose and image even for a gesture shorter than
    // either timer interval. renderNow/captureImage also cancel stale frames.
    if (m_cameraDirty) publishCameraChange();
    if (m_renderTimer.isActive() || m_interactionFrameRendered) renderNow();
}

double VTKViewer::parallelScale() const { return m_renderer->GetActiveCamera()->GetParallelScale(); }
void VTKViewer::setParallelScale(double scale) { m_renderer->GetActiveCamera()->SetParallelScale(scale); renderNow(); }
double VTKViewer::pointCloudPointSize() const { return m_pointCloudActor.actor()->GetProperty()->GetPointSize(); }

void VTKViewer::setSketch(const modeling::SketchFeatureParams& params,
                          const sketchapp::PlaneFrame& frame)
{
    m_sketchActor.setSketch(params, frame);
    m_sketchActor.actor()->SetUserMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    m_sketchActor.draftActor()->SetUserMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    m_renderer->ResetCameraClippingRange();
    renderNow();
}

void VTKViewer::clearSketch()
{
    m_sketchActor.clear();
    renderNow();
}

bool VTKViewer::hasSketch() const noexcept
{
    return m_sketchActor.hasSketch();
}

void VTKViewer::setViewportTool(ViewportTool tool)
{
    m_tool = tool;
    if (tool != ViewportTool::Line && tool != ViewportTool::Rectangle &&
        tool != ViewportTool::Circle) {
        clearSketchDraft();
    }
}

ViewportTool VTKViewer::viewportTool() const noexcept
{
    return m_tool;
}

void VTKViewer::setSketchInteractionEnabled(bool enabled)
{
    if (enabled) closeBoundaryInspection();
    if (enabled) endBoxReferenceSelection();
    m_sketchInteraction = enabled;
    emit sketchInteractionChanged(enabled);
    if (!enabled) {
        clearSketchDraft();
    }
}

bool VTKViewer::sketchInteractionEnabled() const noexcept
{
    return m_sketchInteraction;
}

void VTKViewer::setSketchPlane(const sketchapp::PlaneFrame& frame)
{
    m_sketchPlane = frame;
    m_hasSketchPlane = true;
}

void VTKViewer::clearSketchPlane()
{
    m_hasSketchPlane = false;
}

void VTKViewer::alignToSketchPlane(const sketchapp::PlaneFrame& modelFrame, bool fitSketch)
{
    vtkNew<vtkTransform> placement;
    placement->SetMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    sketchapp::PlaneFrame frame;
    placement->TransformPoint(modelFrame.origin, frame.origin);
    placement->TransformNormal(modelFrame.normal, frame.normal);
    placement->TransformVector(modelFrame.yDirection, frame.yDirection);
    normalize(frame.normal);
    normalize(frame.yDirection);
    double halfHeight = kSketchViewHalfExtent;
    double distance = kSketchViewDistance;
    const auto* bounds = m_sketchActor.actor()->GetBounds();
    if (fitSketch && bounds && vtkMath::AreBoundsInitialized(bounds)) {
        double right[3];
        vtkMath::Cross(frame.yDirection, frame.normal, right);
        normalize(right);
        double width = 0, height = 0;
        for (int i = 0; i < 8; ++i) {
            const double offset[]{bounds[i&1]-frame.origin[0],
                                  bounds[2 + ((i>>1)&1)]-frame.origin[1],
                                  bounds[4 + ((i>>2)&1)]-frame.origin[2]};
            width = std::max(width, std::abs(vtkMath::Dot(offset, right)));
            height = std::max(height, std::abs(vtkMath::Dot(offset, frame.yDirection)));
        }
        const double aspect = double(std::max(1, this->width())) / std::max(1, this->height());
        halfHeight = 1.15 * std::max({width/aspect, height, 1e-3});
        distance = std::max(distance, 2*halfHeight);
    }
    alignCameraToPlane(m_renderer, frame.origin, frame.normal, frame.yDirection, distance, halfHeight);
    renderNow();
}

void VTKViewer::alignCameraToPlane(vtkRenderer* renderer, const double center[3], const double normal[3],
                                   const double up[3], double distance, double halfHeight)
{
    auto* camera = renderer->GetActiveCamera();
    const double position[3] = {
        center[0] + normal[0] * distance, center[1] + normal[1] * distance,
        center[2] + normal[2] * distance};
    camera->SetPosition(position);
    camera->SetFocalPoint(center);
    camera->SetViewUp(up);
    camera->OrthogonalizeViewUp();
    camera->SetParallelProjection(1);
    camera->SetParallelScale(halfHeight);
    renderer->ResetCameraClippingRange();
}

bool VTKViewer::hasSketchVertexAt(const modeling::Point2D& point,
                                  double tolerance) const
{
    modeling::Point2D vertex;
    return m_sketchActor.snapPointAt(point, tolerance, vertex);
}

void VTKViewer::highlightFace(int faceId)
{
    m_highlightedFace = faceId;
    m_bodyActor.highlightFace(faceId);
    renderNow();
}

void VTKViewer::clearFaceHighlight()
{
    m_highlightedFace = -1;
    m_bodyActor.clearHighlight();
    renderNow();
}

void VTKViewer::setDatumPlanesVisible(bool visible)
{
    m_datumPlanesVisible = visible;
    m_datumPlanes.setVisible(visible && !m_referenceSelection);
    renderNow();
}

void VTKViewer::highlightDatumPlane(modeling::DatumPlane plane)
{
    m_datumPlanes.setHighlightedPlane(plane);
    renderNow();
}

void VTKViewer::clearDatumPlaneHighlight()
{
    m_datumPlanes.setHighlightedPlane(std::nullopt);
    renderNow();
}

void VTKViewer::setSketchDraftAnchor(const modeling::Point2D& anchor)
{
    m_draftAnchor = anchor;
}

void VTKViewer::clearSketchDraft()
{
    m_draftAnchor.reset();
    m_sketchActor.clearDraft();
}

// Intersects the display ray of one pixel with the active sketch plane and
// reports the hit in sketch millimetres. This is what makes a click land on the
// face the sketch was created on, whatever its orientation is.
//
// The ray is built from the camera basis rather than through
// vtkViewport::SetDisplayPoint(), whose pixel convention differs between
// render-window and widget coordinates once the display is scaled.
bool VTKViewer::sketchPointAt(int x, int y, modeling::Point2D& point)
{
    if (!m_hasSketchPlane) {
        return false;
    }
    int* const size = m_renderWindow->GetSize();
    if (size == nullptr || size[0] <= 0 || size[1] <= 0) {
        return false;
    }
    const double width = static_cast<double>(size[0]);
    const double height = static_cast<double>(size[1]);

    vtkCamera* camera = m_renderer->GetActiveCamera();
    double forward[3] = {0.0, 0.0, -1.0};
    double upHint[3] = {0.0, 1.0, 0.0};
    camera->GetDirectionOfProjection(forward);
    camera->GetViewUp(upHint);

    // right = forward x up, then up = right x forward, which is the screen basis
    // VTK itself uses; rebuilding it here keeps the mapping explicit.
    double right[3] = {1.0, 0.0, 0.0};
    double up[3] = {0.0, 1.0, 0.0};
    cross(forward, upHint, right);
    if (!normalize(right)) {
        return false;
    }
    cross(right, forward, up);
    if (!normalize(up)) {
        return false;
    }

    // The event position has its origin at the bottom-left of the render window,
    // which is the sign convention used for the view-plane offsets below.
    const double nx = 2.0 * static_cast<double>(x) / width - 1.0;
    const double ny = 2.0 * static_cast<double>(y) / height - 1.0;
    const double aspect = width / height;

    double origin[3] = {0.0, 0.0, 0.0};
    camera->GetPosition(origin);
    double direction[3] = {0.0, 0.0, -1.0};
    if (camera->GetParallelProjection()) {
        const double scale = camera->GetParallelScale();
        for (int axis = 0; axis < 3; ++axis) {
            origin[axis] += nx * aspect * scale * right[axis] + ny * scale * up[axis];
            direction[axis] = forward[axis];
        }
    } else {
        const double slope = std::tan(camera->GetViewAngle() * kPiOver180 / 2.0);
        for (int axis = 0; axis < 3; ++axis) {
            direction[axis] = forward[axis] + nx * aspect * slope * right[axis] +
                              ny * slope * up[axis];
        }
    }

    // Geometry stays in model coordinates even when the body is transformed
    // for display. Intersect the inverse-transformed ray with the model plane.
    vtkNew<vtkTransform> inverse;
    inverse->SetMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    inverse->Inverse();
    inverse->TransformPoint(origin, origin);
    inverse->TransformVector(direction, direction);

    // The ray direction is normalized so the plane distance below is a real
    // length; vtkPlane::IntersectWithLine cannot be used here because it treats
    // its parameter as a fraction along the segment and would reject any hit
    // farther than the segment it is given.
    if (!normalize(direction)) {
        return false;
    }
    const double denominator = m_sketchPlane.normal[0] * direction[0] +
                               m_sketchPlane.normal[1] * direction[1] +
                               m_sketchPlane.normal[2] * direction[2];
    if (std::abs(denominator) < 1.0e-9) {
        // The ray runs parallel to the sketch plane, so there is nothing to hit.
        return false;
    }
    const double numerator =
        m_sketchPlane.normal[0] * (m_sketchPlane.origin[0] - origin[0]) +
        m_sketchPlane.normal[1] * (m_sketchPlane.origin[1] - origin[1]) +
        m_sketchPlane.normal[2] * (m_sketchPlane.origin[2] - origin[2]);
    const double distance = numerator / denominator;
    if (distance <= 0.0) {
        return false;
    }
    const double hit[3] = {origin[0] + distance * direction[0],
                           origin[1] + distance * direction[1],
                           origin[2] + distance * direction[2]};

    const double offset[3] = {hit[0] - m_sketchPlane.origin[0],
                              hit[1] - m_sketchPlane.origin[1],
                              hit[2] - m_sketchPlane.origin[2]};
    point.x = offset[0] * m_sketchPlane.xDirection[0] +
              offset[1] * m_sketchPlane.xDirection[1] +
              offset[2] * m_sketchPlane.xDirection[2];
    point.y = offset[0] * m_sketchPlane.yDirection[0] +
              offset[1] * m_sketchPlane.yDirection[1] +
              offset[2] * m_sketchPlane.yDirection[2];
    return true;
}

bool VTKViewer::snapSketchPointAt(int x, int y, modeling::Point2D& point)
{
    modeling::Point2D raw;
    modeling::Point2D neighbour;
    if (!sketchPointAt(x, y, raw) || !sketchPointAt(x + 1, y, neighbour)) {
        return false;
    }
    point = raw;

    // One pixel measured in sketch units keeps the snap radius constant on
    // screen, whatever the zoom or how oblique the plane is.
    const double unitsPerPixel = std::hypot(neighbour.x - raw.x, neighbour.y - raw.y);
    if (unitsPerPixel <= 0.0) {
        return true;
    }
    modeling::Point2D vertex;
    if (m_sketchActor.snapPointAt(raw, unitsPerPixel * kSketchSnapPixelRadius, vertex)) {
        point = vertex;
    }
    return true;
}

bool VTKViewer::pickedBodyPlane(int faceId, modeling::Plane3d& plane, modeling::Vec3& point) const
{
    const auto* info = m_bodyActor.faceInfo(faceId);
    if (!info || !info->planar || m_pickedFaceId != faceId) return false;
    plane = {{info->origin[0],info->origin[1],info->origin[2]},
             {info->normal[0],info->normal[1],info->normal[2]},
             {info->xDirection[0],info->xDirection[1],info->xDirection[2]}};
    point = m_pickedModelPoint;
    return true;
}

int VTKViewer::pickPointCloudRegionAt(int x, int y)
{
    if (!hasPointCloud() || !m_pointCloudVisible) return 0;
    vtkNew<vtkCellPicker> picker;
    picker->PickFromListOn();
    picker->AddPickList(m_pointCloudActor.surfaceActor());
    picker->SetTolerance(1.e-6);
    picker->Pick(static_cast<double>(x), static_cast<double>(y), 0.0, m_renderer);
    return m_pointCloudActor.surfaceRegionOfCell(picker->GetCellId());
}

bool VTKViewer::pickBodyFaceAt(int x, int y, int& faceId,
                               sketchapp::PlaneFrame& plane, bool& planar)
{
    if (!m_bodyActor.hasShape()) {
        return false;
    }

    vtkNew<vtkCellPicker> picker;
    picker->PickFromListOn();
    picker->AddPickList(m_bodyActor.surfaceActor());
    picker->Pick(static_cast<double>(x), static_cast<double>(y), 0.0, m_renderer);

    const vtkIdType cellId = picker->GetCellId();
    if (cellId < 0) {
        return false;
    }
    const int hitFace = m_bodyActor.faceIdOfCell(cellId);
    const BodyFaceInfo* info = m_bodyActor.faceInfo(hitFace);
    if (info == nullptr) {
        return false;
    }

    faceId = hitFace;
    planar = info->planar;
    if (!info->planar) {
        return true;
    }

    // The exact plane and picked point stay in model coordinates. Display
    // placement is applied to sketch actors and removed from drawing rays.
    for (int axis = 0; axis < 3; ++axis) {
        plane.origin[axis] = info->origin[axis];
        plane.normal[axis] = info->normal[axis];
        plane.xDirection[axis] = info->xDirection[axis];
    }
    cross(plane.normal, plane.xDirection, plane.yDirection);
    vtkNew<vtkTransform> inverse;
    inverse->SetMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    inverse->Inverse();
    double point[3];
    inverse->TransformPoint(picker->GetPickPosition(),point);
    m_pickedFaceId = hitFace;
    m_pickedModelPoint = {point[0],point[1],point[2]};
    return true;
}

bool VTKViewer::pickDatumPlaneAt(int x, int y, modeling::DatumPlane& plane)
{
    if (!m_datumPlanes.visible()) {
        return false;
    }

    // Only the translucent surfaces are offered to the picker, so the outlines
    // and the name labels cannot steal a click from the plane behind them.
    vtkNew<vtkCellPicker> picker;
    picker->PickFromListOn();
    for (modeling::DatumPlane candidate : {modeling::DatumPlane::XY, modeling::DatumPlane::YZ,
                                           modeling::DatumPlane::XZ}) {
        picker->AddPickList(m_datumPlanes.surfaceActor(candidate));
    }
    picker->Pick(static_cast<double>(x), static_cast<double>(y), 0.0, m_renderer);

    const std::optional<modeling::DatumPlane> hit = m_datumPlanes.planeOfActor(picker->GetActor());
    if (!hit.has_value()) {
        return false;
    }
    plane = *hit;
    return true;
}

bool VTKViewer::pickSketchEntityAt(int x, int y, modeling::SketchEntityId& entityId)
{
    if (!m_sketchActor.hasSketch()) {
        return false;
    }

    modeling::Point2D click;
    modeling::Point2D neighbour;
    if (!sketchPointAt(x, y, click) || !sketchPointAt(x + 1, y, neighbour)) {
        return false;
    }

    // Sketch entities are zero-width on screen, so a click is accepted from a
    // narrow band around them. Measuring one pixel in sketch units keeps that
    // band constant on screen no matter how far away or how oblique the plane is,
    // and it avoids a geometric picker that a hairline never satisfies.
    const double unitsPerPixel = std::hypot(neighbour.x - click.x, neighbour.y - click.y);
    if (unitsPerPixel <= 0.0) {
        return false;
    }

    const modeling::SketchEntityId hit =
        m_sketchActor.entityAt(click, unitsPerPixel * kSketchPickPixelRadius);
    if (hit == modeling::kInvalidSketchEntityId) {
        return false;
    }
    entityId = hit;
    return true;
}

void VTKViewer::onLeftButtonPressed(int x, int y)
{
    if (m_boundaryInspectionRenderer) return;
    if (m_referenceSelection) {
        const int anchor=m_boxReferenceActor.pick(m_referenceRenderer,x,y);
        if (anchor>=0) { endBoxReferenceSelection(); emit boxReferencePicked(anchor); }
        return;
    }
    if (m_sketchInteraction) {
        if (m_tool == ViewportTool::Select) {
            modeling::SketchEntityId entityId = modeling::kInvalidSketchEntityId;
            if (pickSketchEntityAt(x, y, entityId)) {
                emit sketchEntityPicked(entityId);
            } else {
                emit sketchSelectionCleared();
            }
            return;
        }

        // Every drawing tool anchors on existing vertices: the click is snapped
        // onto the nearest one, and only a click that misses them all reaches the
        // window as a free point.
        modeling::Point2D point;
        if (snapSketchPointAt(x, y, point)) {
            emit sketchPointRequested(point);
        }
        return;
    }

    const int region = pickPointCloudRegionAt(x, y);
    emit pointCloudFacePicked(region);
    int faceId = -1;
    bool planar = false;
    sketchapp::PlaneFrame plane;
    if (pickBodyFaceAt(x, y, faceId, plane, planar)) {
        emit facePicked(faceId, planar, plane);
        return;
    }

    auto* interactor = m_renderWindow->GetInteractor();
    const bool extendSelection = interactor->GetControlKey() || interactor->GetShiftKey();
    if (region > 0) {
        if (!extendSelection) {
            m_pointCloudDocument->selectRegion(region);
            return;
        }
        const auto ids = pointCloudSelection().regionPoints(region);
        auto selected = pointCloudSelection().selected();
        const std::set<vtkIdType> current(selected.begin(), selected.end());
        if (std::all_of(ids.begin(), ids.end(), [&](auto id) { return current.count(id) != 0; })) {
            const std::set<vtkIdType> members(ids.begin(), ids.end());
            selected.erase(std::remove_if(selected.begin(), selected.end(), [&](auto id) {
                return members.count(id) != 0;
            }), selected.end());
            emit pointCloudFacePicked(0);
            m_pointCloudDocument->select(selected);
        } else {
            m_pointCloudDocument->select(ids, true);
        }
        return;
    }

    // Datum planes can cross a model or a fitted cloud surface.
    modeling::DatumPlane datumPlane = modeling::DatumPlane::XY;
    if (pickDatumPlaneAt(x, y, datumPlane)) {
        emit datumPlanePicked(datumPlane);
        return;
    }
    if (hasPointCloud() && m_pointCloudVisible && !extendSelection && !pointCloudSelection().selected().empty())
        m_pointCloudDocument->clearSelection();
    emit emptyPicked();
}

void VTKViewer::onMouseMoved(int x, int y)
{
    if (!m_sketchInteraction || !m_draftAnchor.has_value()) {
        return;
    }
    modeling::Point2D cursor;
    if (!snapSketchPointAt(x, y, cursor)) {
        return;
    }

    const modeling::Point2D anchor = *m_draftAnchor;
    switch (m_tool) {
    case ViewportTool::Line:
        m_sketchActor.setDraft(
            modeling::Line2D{anchor.x, anchor.y, cursor.x, cursor.y});
        break;
    case ViewportTool::Rectangle: {
        const double x0 = std::min(anchor.x, cursor.x);
        const double y0 = std::min(anchor.y, cursor.y);
        m_sketchActor.setDraft(modeling::Rectangle2D{
            x0, y0, std::abs(cursor.x - anchor.x), std::abs(cursor.y - anchor.y)});
        break;
    }
    case ViewportTool::Circle: {
        const double radius = std::hypot(cursor.x - anchor.x, cursor.y - anchor.y);
        m_sketchActor.setDraft(modeling::Circle2D{anchor.x, anchor.y, radius});
        break;
    }
    default:
        return;
    }
    renderNow();
}

void VTKViewer::onCancelRequested()
{
    if (m_boundaryInspectionRenderer) return;
    if (m_referenceSelection) {
        endBoxReferenceSelection(); emit boxReferenceSelectionCancelled(); return;
    }
    clearSketchDraft();
    renderNow();
    emit sketchCancelled();
}

void VTKViewer::onKeyPressed(const std::string& keySym)
{
    if (m_boundaryInspectionRenderer) {
        if (keySym == "Escape") closeBoundaryInspection();
        return;
    }
    if (m_referenceSelection && keySym=="Escape") {
        endBoxReferenceSelection(); emit boxReferenceSelectionCancelled(); return;
    }
    // Outside a sketch the Delete key means nothing here, so it stays unclaimed
    // for whatever widget owns the focus.
    if (!m_sketchInteraction) {
        return;
    }
    if (keySym == "Delete") {
        emit sketchDeleteRequested();
    }
}

bool VTKViewer::captureImage(const QString& filePath, int width, int height)
{
    if (!renderingActive() || filePath.isEmpty() || width <= 0 || height <= 0) {
        return false;
    }
    closeBoundaryInspection();

    // A photo is of the model. The datum planes, the corner orientation marker
    // and the face highlight are editing aids, so they are taken out of the frame
    // for the write and put back exactly as they were, which keeps a capture
    // usable as reconstruction input whatever the editor was showing.
    const bool datumPlanesWereVisible = m_datumPlanesVisible;
    const int highlightedFace = m_highlightedFace;
    const bool boundariesWereVisible=m_assemblyBoundaryActor->GetVisibility()!=0;m_assemblyBoundaryActor->SetVisibility(false);
    m_referenceRenderer->DrawOff();
    m_pointCloudActor.setAnnotations(nullptr);
    m_datumPlanes.setVisible(false);
    m_bodyActor.clearHighlight();
    if (m_orientationWidget) {
        m_orientationWidget->SetEnabled(0);
    }

    renderFrame(false);

    vtkNew<vtkWindowToImageFilter> windowCapture;
    windowCapture->SetInput(m_renderWindow);
    windowCapture->SetInputBufferTypeToRGB();
    windowCapture->ReadFrontBufferOff();
    windowCapture->ShouldRerenderOff();
    windowCapture->Update();

    vtkNew<vtkImageResize> resize;
    resize->SetInputConnection(windowCapture->GetOutputPort());
    resize->SetResizeMethodToOutputDimensions();
    resize->SetOutputDimensions(width, height, 1);
    resize->InterpolateOn();

    vtkNew<vtkPNGWriter> writer;
    const QByteArray nativePath = filePath.toLocal8Bit();
    writer->SetFileName(nativePath.constData());
    writer->SetInputConnection(resize->GetOutputPort());
    writer->Write();
    const bool written = writer->GetErrorCode() == 0;

    m_datumPlanes.setVisible(datumPlanesWereVisible && !m_referenceSelection);
    m_referenceRenderer->DrawOn();
    m_pointCloudActor.setAnnotations(&pointCloudSelection());
    m_assemblyBoundaryActor->SetVisibility(boundariesWereVisible);
    if (highlightedFace >= 0) {
        m_bodyActor.highlightFace(highlightedFace);
    }
    if (m_orientationWidget) {
        m_orientationWidget->SetEnabled(1);
    }
    renderNow();

    return written;
}

bool VTKViewer::captureViewport(const QString& filePath)
{
    if (!renderingActive() || filePath.isEmpty()) {
        return false;
    }

    renderFrame(false);

    vtkNew<vtkWindowToImageFilter> windowCapture;
    windowCapture->SetInput(m_renderWindow);
    windowCapture->SetInputBufferTypeToRGB();
    windowCapture->ReadFrontBufferOff();
    windowCapture->ShouldRerenderOff();
    windowCapture->Update();

    vtkNew<vtkPNGWriter> writer;
    const QByteArray nativePath = filePath.toLocal8Bit();
    writer->SetFileName(nativePath.constData());
    writer->SetInputConnection(windowCapture->GetOutputPort());
    writer->Write();

    return writer->GetErrorCode() == 0;
}

QSize VTKViewer::captureSourceSize() const
{
    int* const size = m_renderWindow->GetSize();
    if (size == nullptr || size[0] <= 0 || size[1] <= 0) {
        return QSize();
    }
    return QSize(size[0], size[1]);
}

void VTKViewer::syncPointCloud()
{
    clearCadPreview();
    closeBoundaryInspection();
    clearProjectionPreview();
    setAssemblyBoundaries({},{});
    if (m_pointCloudDocument->hasCloud()) {
        m_pointCloudActor.setCloud(m_pointCloudDocument->sourceCloud());
        m_pointCloudActor.setPointMask(pointCloudSelection().deleted());
        m_pointCloudVisible = true;
        m_pointCloudActor.setVisible(true);
        emit pointCloudVisibilityChanged(true);
        emit pointCloudLoaded();
    } else {
        m_pointCloudActor.clear();
        m_pointCloudVisible = false;
        emit pointCloudVisibilityChanged(false);
    }
    m_renderer->ResetCameraClippingRange();
    requestRender();
}

bool VTKViewer::hasPointCloud() const noexcept { return m_pointCloudDocument->hasCloud(); }

void VTKViewer::resetCameraToPointCloud()
{
    if (m_boundaryInspectionRenderer) return;
    if (!m_pointCloudActor.hasCloud()) {
        return;
    }
    double bounds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    m_pointCloudActor.bounds(bounds);
    m_renderer->ResetCameraClippingRange();
    m_renderer->ResetCamera(bounds);
    m_renderer->ResetCameraClippingRange();
    renderNow();
    publishCameraChange();
}

bool VTKViewer::loadBodyTexture(const QString& imagePath, QString& error)
{
    if (!m_bodyActor.setTextureFromFile(imagePath, error)) {
        return false;
    }
    m_bodyTexturePath = imagePath;
    renderNow();
    return true;
}

void VTKViewer::clearBodyTexture()
{
    m_bodyTexturePath.clear();
    m_bodyActor.clearTexture();
    renderNow();
}

void VTKViewer::setBodyTextureData(vtkImageData* image, const QString& path)
{
    m_bodyActor.setTextureData(image);
    m_bodyTexturePath = path;
    renderNow();
}

void VTKViewer::setBodyTextureEnabled(bool enabled)
{
    m_bodyActor.setTextureEnabled(enabled);
    renderNow();
}

bool VTKViewer::hasBodyTexture() const noexcept
{
    return m_bodyActor.hasTexture();
}

void VTKViewer::setTextureProjection(int axis)
{
    m_bodyActor.setTextureProjection(axis);
    renderNow();
}

int VTKViewer::textureProjection() const noexcept
{
    return m_bodyActor.textureProjection();
}

void VTKViewer::setPointCloudVisible(bool visible)
{
    m_pointCloudVisible = visible;
    m_pointCloudActor.setVisible(visible);
    for (const auto& actor : m_cadSurfacePreviews) actor->SetVisibility(visible);
    if (m_hasCadSolidPreview) m_cadSolidPreview.setBodyVisible(visible);
    m_assemblyBoundaryActor->SetVisibility(visible && m_boundaryPointCount==m_pointCloudDocument->pointCount() && m_assemblyBoundaryActor->GetMapper()!=nullptr);
    emit pointCloudVisibilityChanged(visible);
    renderNow();
}

void VTKViewer::setAssemblyBoundaries(const std::vector<std::array<double,6>>& lines,const std::vector<bool>& closed,
                                    const std::vector<int>& regions)
{
    m_assemblyBoundaryLines=lines;m_assemblyBoundaryClosed=closed;m_assemblyBoundaryRegions=regions;
    m_boundaryPointCount=lines.empty()?0:m_pointCloudDocument->pointCount();
    m_boundaryRevision=pointCloudSelection().surfaceRevision();
    refreshAssemblyBoundaries();
}

void VTKViewer::setShowAllAssemblyBoundaries(bool enabled)
{
    m_showAllAssemblyBoundaries=enabled;refreshAssemblyBoundaries();
}

void VTKViewer::refreshAssemblyBoundaries()
{
    if(!m_assemblyBoundaryActor)return;
    m_assemblyBoundarySegments=0;
    if(m_assemblyBoundaryLines.empty()){m_assemblyBoundaryActor->SetVisibility(false);m_assemblyBoundaryActor->SetMapper(nullptr);return;}
    std::set<int> selectedRegions;
    const auto& selection=pointCloudSelection();
    for(auto id:selection.selected())if(id>=0 && size_t(id)<selection.memberships().size())
        selectedRegions.insert(selection.memberships()[size_t(id)]);
    // Deduplicate only at numerical display precision, well below fitting/weld
    // tolerances. This does not move source geometry or merge thin-wall faces.
    std::array<double,3> lower{INFINITY,INFINITY,INFINITY},upper{-INFINITY,-INFINITY,-INFINITY};
    for(const auto& line:m_assemblyBoundaryLines)for(int i=0;i<6;++i) {
        lower[i%3]=std::min(lower[i%3],line[i]);upper[i%3]=std::max(upper[i%3],line[i]);
    }
    const double precision=std::max(1e-12,std::hypot(std::hypot(upper[0]-lower[0],upper[1]-lower[1]),upper[2]-lower[2])*1e-10);
    std::map<std::array<long long,6>,std::pair<std::array<double,6>,bool>> unique;
    for(size_t i=0;i<m_assemblyBoundaryLines.size();++i) {
        const bool closed=i<m_assemblyBoundaryClosed.size()&&m_assemblyBoundaryClosed[i];
        if(!m_showAllAssemblyBoundaries && !closed && !m_assemblyBoundaryRegions.empty() &&
            (i>=m_assemblyBoundaryRegions.size() || !selectedRegions.count(m_assemblyBoundaryRegions[i])))continue;
        auto line=m_assemblyBoundaryLines[i];
        std::array<long long,3> a{},b{};
        for(int axis=0;axis<3;++axis) {
            a[axis]=std::llround((line[axis]-lower[axis])/precision);
            b[axis]=std::llround((line[axis+3]-lower[axis])/precision);
        }
        if(a==b)continue;
        if(b<a){std::swap(a,b);line={line[3],line[4],line[5],line[0],line[1],line[2]};}
        const std::array<long long,6> key{a[0],a[1],a[2],b[0],b[1],b[2]};
        auto inserted=unique.emplace(key,std::make_pair(line,closed));
        if(!inserted.second)inserted.first->second.second|=closed;
    }
    vtkNew<vtkPoints> points;vtkNew<vtkCellArray> cells;vtkNew<vtkUnsignedCharArray> colors;colors->SetNumberOfComponents(3);
    for(const auto& entry:unique){const auto& line=entry.second.first;const bool closed=entry.second.second;const vtkIdType ids[]{points->InsertNextPoint(line[0],line[1],line[2]),points->InsertNextPoint(line[3],line[4],line[5])};cells->InsertNextCell(2,ids);
        const unsigned char color[]{static_cast<unsigned char>(closed?50:255),static_cast<unsigned char>(closed?220:170),80};colors->InsertNextTypedTuple(color);}
    vtkNew<vtkPolyData> polyData;polyData->SetPoints(points);polyData->SetLines(cells);polyData->GetCellData()->SetScalars(colors);
    vtkNew<vtkPolyDataMapper> mapper;mapper->SetInputData(polyData);mapper->SetScalarModeToUseCellData();mapper->SetColorModeToDirectScalars();
    m_assemblyBoundaryActor->SetMapper(mapper);m_assemblyBoundarySegments=unique.size();m_assemblyBoundaryActor->SetVisibility(m_pointCloudVisible&&!unique.empty());renderNow();
}

double VTKViewer::pointCloudDiagonal() const
{
    double bounds[6];
    m_pointCloudActor.bounds(bounds);
    return std::hypot(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]);
}

void VTKViewer::refreshPointCloudSelection()
{
    m_pointCloudActor.setAnnotations(&pointCloudSelection());
    emit pointCloudSurfaceDisplayChanged();
    if(!m_assemblyBoundaryLines.empty())refreshAssemblyBoundaries();
    requestRender();
}

void VTKViewer::setAutomaticPointCloudSurfaces(bool enabled)
{
    m_pointCloudActor.setAutomaticSurfaceDisplay(enabled);
    refreshPointCloudSelection();
}

void VTKViewer::refreshPointCloudSurfaceSettings()
{
    closeBoundaryInspection();
    m_pointCloudActor.invalidateSurfaceMeshes();
    refreshPointCloudSelection();
}

void VTKViewer::setPointCloudRegionSurface(int region, bool enabled)
{
    m_pointCloudActor.setRegionSurfaceDisplay(region, enabled);
    refreshPointCloudSelection();
}

void VTKViewer::setPointCloudPointSize(double size)
{
    m_pointCloudActor.setPointSize(size);
    renderNow();
}

void VTKViewer::rotateCamera(double azimuthDegrees, double elevationDegrees)
{
    auto* camera = m_renderer->GetActiveCamera();
    // Orbit around the existing focal point, including off-origin clouds.
    // Camera-local axes also allow rotation through the top and bottom views.
    camera->Azimuth(azimuthDegrees);
    camera->Elevation(elevationDegrees);
    camera->OrthogonalizeViewUp();
    m_renderer->ResetCameraClippingRange();
    requestRender();
    publishCameraChange();
}

void VTKViewer::resetCamera()
{
    // The datum planes are large fixed-size reference geometry, so they are left
    // out of the fit: framing them would shrink the model itself by half.
    double bounds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (sceneBounds(bounds)) {
        m_renderer->ResetCamera(bounds);
    } else {
        m_renderer->ResetCamera();
    }
    m_renderer->ResetCameraClippingRange();
    renderNow();
    publishCameraChange();
}

bool VTKViewer::sceneBounds(double bounds[6])
{
    bool hasBounds = false;
    if (m_bodyActor.hasShape() && m_bodyActor.surfaceActor()->GetVisibility() != 0) {
        m_bodyActor.surfaceActor()->GetBounds(bounds);
        hasBounds = true;
    }
    if (m_pointCloudActor.hasCloud()) {
        double cloud[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        m_pointCloudActor.bounds(cloud);
        if (!hasBounds) {
            std::copy(cloud, cloud + 6, bounds);
            return true;
        }
        for (int axis = 0; axis < 3; ++axis) {
            bounds[axis * 2] = std::min(bounds[axis * 2], cloud[axis * 2]);
            bounds[axis * 2 + 1] = std::max(bounds[axis * 2 + 1], cloud[axis * 2 + 1]);
        }
    }
    return hasBounds;
}

void VTKViewer::setStandardView(CameraController::StandardView view)
{
    closeBoundaryInspection();
    CameraController::applyStandardView(m_renderer->GetActiveCamera(), view);
    double bounds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    if (sceneBounds(bounds)) {
        m_renderer->ResetCamera(bounds);
    } else {
        m_renderer->ResetCamera();
    }
    m_renderer->ResetCameraClippingRange();
    renderNow();
    publishCameraChange();
}

void VTKViewer::applyCamera(const CameraParameters& parameters)
{
    closeBoundaryInspection();
    CameraController::apply(m_renderer->GetActiveCamera(), parameters);
    m_renderer->ResetCameraClippingRange();
    renderNow();
    publishCameraChange();
}

void VTKViewer::setParallelProjection(bool enabled)
{
    if (m_boundaryInspectionRenderer) return;
    m_renderer->GetActiveCamera()->SetParallelProjection(enabled ? 1 : 0);
    renderNow();
    publishCameraChange();
}

void VTKViewer::setObjectTransform(double px, double py, double pz,
                                   double rx, double ry, double rz,
                                   double sx, double sy, double sz)
{
    m_bodyActor.setTransform(px, py, pz, rx, ry, rz, sx, sy, sz);
    m_datumPlanes.setTransform(m_bodyActor.surfaceActor()->GetMatrix());
    m_boxReferenceActor.setTransform(m_bodyActor.surfaceActor()->GetMatrix());
    m_sketchActor.actor()->SetUserMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    m_sketchActor.draftActor()->SetUserMatrix(m_bodyActor.surfaceActor()->GetMatrix());
    m_renderer->ResetCameraClippingRange();
    renderNow();
}

void VTKViewer::setObjectVisible(bool visible)
{
    if (!visible) endBoxReferenceSelection();
    m_bodyActor.setVisible(visible);
    emit objectVisibilityChanged(visible);
    renderNow();
}

void VTKViewer::setObjectRepresentation(int representation)
{
    m_bodyActor.setRepresentation(representation);
    renderNow();
}

void VTKViewer::setObjectOpacity(double opacity)
{
    m_bodyActor.setOpacity(std::clamp(opacity, 0.0, 1.0));
    renderNow();
}

void VTKViewer::setObjectColor(const QColor& color)
{
    m_bodyActor.setColor(color);
    renderNow();
}

void VTKViewer::setObjectMetallic(double metallic)
{
    m_bodyActor.setMetallic(metallic);
    renderNow();
}

void VTKViewer::setObjectRoughness(double roughness)
{
    m_bodyActor.setRoughness(roughness);
    renderNow();
}

void VTKViewer::setObjectMaterialPreview(bool enabled)
{
    m_bodyActor.setMaterialPreviewEnabled(enabled);
    renderNow();
}

QColor VTKViewer::backgroundColor() const
{
    const auto* c = m_renderer->GetBackground();
    return QColor::fromRgbF(c[0], c[1], c[2]);
}

QColor VTKViewer::backgroundColor2() const
{
    const auto* c = m_renderer->GetBackground2();
    return QColor::fromRgbF(c[0], c[1], c[2]);
}

bool VTKViewer::gradientBackground() const { return m_renderer->GetGradientBackground(); }

void VTKViewer::setBackgroundGradient(bool enabled, const QColor& secondary)
{
    m_renderer->SetBackground2(secondary.redF(), secondary.greenF(), secondary.blueF());
    m_renderer->SetGradientBackground(enabled);
    renderNow();
}

void VTKViewer::setBackgroundColor(const QColor& color)
{
    m_renderer->GradientBackgroundOff();
    m_renderer->SetBackground(color.redF(), color.greenF(), color.blueF());
    renderNow();
}

void VTKViewer::setLightingEnabled(bool enabled)
{
    m_bodyActor.setLightingEnabled(enabled);
    renderNow();
}

void VTKViewer::onCameraModified(vtkObject*, unsigned long, void* clientData, void*)
{
    auto* viewer = static_cast<VTKViewer*>(clientData);
    if (viewer) {
        viewer->m_cameraDirty = true;
        // A VTK camera gesture changes position, up and clipping separately.
        // Publish the completed pose once on the event loop, or at 20 Hz while
        // dragging, instead of updating Qt widgets for every intermediate edit.
        if (!viewer->m_cameraNotifyTimer.isActive()) {
            viewer->m_cameraNotifyTimer.start(
                viewer->m_cameraInteracting ? kCameraPanelIntervalMs : 0);
        }
    }
}

void VTKViewer::createOrientationAxes()
{
    m_axesActor = vtkSmartPointer<vtkAxesActor>::New();
    m_axesActor->SetTotalLength(0.9, 0.9, 0.9);
    m_axesActor->SetShaftTypeToCylinder();

    m_orientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    m_orientationWidget->SetOrientationMarker(m_axesActor);
    m_orientationWidget->SetInteractor(m_renderWindow->GetInteractor());
    m_orientationWidget->SetViewport(0.0, 0.0, 0.18, 0.25);
    m_orientationWidget->SetEnabled(1);
    m_orientationWidget->InteractiveOff();
}
