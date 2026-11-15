#include "rendering/BodyActor.h"
#include "rendering/ProjectedMaterialShader.h"
#include <QCoreApplication>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDataArray.h>
#include <vtkFeatureEdges.h>
#include <vtkFloatArray.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkOpenGLPolyDataMapper.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkShaderProperty.h>
#include <vtkTexture.h>
#include <vtkUniforms.h>
#include <vtkWindowToImageFilter.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <string>
#include <vector>
#include <QImage>

namespace {

// Projection direction of a facing direction: the axis it is most aligned with.
int dominantAxis(const double normal[3])
{
    int axis = 0;
    double best = std::fabs(normal[0]);
    for (int i = 1; i < 3; ++i) {
        const double value = std::fabs(normal[i]);
        if (value > best) {
            best = value;
            axis = i;
        }
    }
    return axis;
}

// Geometric normal of a cell, used for the faces the tessellator cannot describe
// by a plane (a curved or otherwise non-planar face).
bool cellNormal(vtkPolyData* mesh, vtkIdType cell, double normal[3])
{
    vtkIdType pointCount = 0;
    const vtkIdType* pointIds = nullptr;
    mesh->GetCellPoints(cell, pointCount, pointIds);
    if (pointCount < 3) {
        return false;
    }
    const double* a = mesh->GetPoint(pointIds[0]);
    const double* b = mesh->GetPoint(pointIds[1]);
    const double* c = mesh->GetPoint(pointIds[2]);
    const double ab[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    const double ac[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    normal[0] = ab[1] * ac[2] - ab[2] * ac[1];
    normal[1] = ab[2] * ac[0] - ab[0] * ac[2];
    normal[2] = ab[0] * ac[1] - ab[1] * ac[0];
    const double length =
        std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
    if (length <= 1.0e-12) {
        return false;
    }
    normal[0] /= length;
    normal[1] /= length;
    normal[2] /= length;
    return true;
}

}  // namespace

namespace {

constexpr int kPreviewShadowSize = 1024;

double clampValue(double value, double low, double high)
{
    return std::min(std::max(value, low), high);
}

double length3(const double v[3])
{
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

bool normalize3(double v[3])
{
    const double length = length3(v);
    if (length <= 1.0e-12) {
        return false;
    }
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
    return true;
}

void cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

// Frame of the capture rig: X right, Y down the image, Z forward. The preview
// light uses it so the same shader sees the same projector conventions as the
// structured-light backend.
void lookAtBasis(const double position[3], const double target[3], const double upHint[3],
                 double x[3], double y[3], double z[3])
{
    for (int i = 0; i < 3; ++i) {
        z[i] = target[i] - position[i];
    }
    if (!normalize3(z)) {
        z[0] = 0.0;
        z[1] = 0.0;
        z[2] = 1.0;
    }
    cross3(z, upHint, x);
    if (!normalize3(x)) {
        const double fallback[3] = {0.0, 1.0, 0.0};
        cross3(z, fallback, x);
        normalize3(x);
    }
    cross3(z, x, y);
}

vtkSmartPointer<vtkMatrix4x4> poseMatrix(const double position[3], const double x[3],
                                         const double y[3], const double z[3])
{
    auto matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    for (int row = 0; row < 3; ++row) {
        matrix->SetElement(row, 0, x[row]);
        matrix->SetElement(row, 1, y[row]);
        matrix->SetElement(row, 2, z[row]);
        matrix->SetElement(row, 3, position[row]);
    }
    return matrix;
}

void applyProjectorCamera(vtkCamera* camera, const double position[3], const double y[3],
                          const double z[3], double focalLength, double nearPlane,
                          double farPlane, int shadowSize)
{
    camera->SetPosition(position[0], position[1], position[2]);
    camera->SetFocalPoint(position[0] + z[0], position[1] + z[1], position[2] + z[2]);
    // The rig's Y axis runs down the image, so the camera up vector is -Y.
    camera->SetViewUp(-y[0], -y[1], -y[2]);
    camera->SetClippingRange(nearPlane, farPlane);
    const double width = shadowSize;
    const double height = shadowSize;
    const double center = 0.5 * width;
    vtkNew<vtkMatrix4x4> projection;
    projection->Zero();
    projection->SetElement(0, 0, 2.0 * focalLength / width);
    projection->SetElement(0, 2, 1.0 - 2.0 * center / width);
    projection->SetElement(1, 1, 2.0 * focalLength / height);
    projection->SetElement(1, 2, 2.0 * center / height - 1.0);
    projection->SetElement(2, 2, -(farPlane + nearPlane) / (farPlane - nearPlane));
    projection->SetElement(2, 3, -2.0 * farPlane * nearPlane / (farPlane - nearPlane));
    projection->SetElement(3, 2, -1.0);
    camera->SetExplicitProjectionTransformMatrix(projection);
    camera->SetUseExplicitProjectionTransformMatrix(true);
}

vtkSmartPointer<vtkTexture> makeWhiteTexture()
{
    auto data = vtkSmartPointer<vtkImageData>::New();
    data->SetDimensions(1, 1, 1);
    data->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
    auto* pixel = static_cast<unsigned char*>(data->GetScalarPointer());
    pixel[0] = 255;
    pixel[1] = 255;
    pixel[2] = 255;
    pixel[3] = 255;
    auto texture = vtkSmartPointer<vtkTexture>::New();
    texture->SetInputData(data);
    texture->SetColorModeToDirectScalars();
    texture->InterpolateOff();
    texture->RepeatOff();
    texture->EdgeClampOn();
    return texture;
}

double srgbToLinear(double channel)
{
    return channel <= 0.04045 ? channel / 12.92
                              : std::pow((channel + 0.055) / 1.055, 2.4);
}

}  // namespace

BodyActor::BodyActor()
{
    m_surfaceMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_surfaceMapper->ScalarVisibilityOff();

    m_surfaceActor = vtkSmartPointer<vtkActor>::New();
    m_surfaceActor->SetMapper(m_surfaceMapper);
    m_surfaceActor->GetProperty()->SetInterpolationToPhong();
    m_surfaceActor->SetVisibility(0);
    applyActorColor();

    // Every modeled face gets its own set of points during tessellation, so the
    // borders between faces are boundary edges rather than feature edges.
    m_featureEdges = vtkSmartPointer<vtkFeatureEdges>::New();
    m_featureEdges->FeatureEdgesOff();
    m_featureEdges->BoundaryEdgesOn();
    m_featureEdges->NonManifoldEdgesOff();
    m_featureEdges->ManifoldEdgesOff();
    m_featureEdges->ColoringOff();

    m_edgeMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_edgeMapper->SetInputConnection(m_featureEdges->GetOutputPort());
    m_edgeMapper->ScalarVisibilityOff();

    m_edgeActor = vtkSmartPointer<vtkActor>::New();
    m_edgeActor->SetMapper(m_edgeMapper);
    m_edgeActor->GetProperty()->SetColor(0.06, 0.07, 0.09);
    m_edgeActor->GetProperty()->SetLineWidth(1.5);
    m_edgeActor->SetVisibility(0);

    m_highlightMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    m_highlightMapper->ScalarVisibilityOff();
    m_highlightMapper->SetResolveCoincidentTopologyToPolygonOffset();
    m_highlightMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(-2.0, -2.0);

    m_highlightActor = vtkSmartPointer<vtkActor>::New();
    m_highlightActor->SetMapper(m_highlightMapper);
    m_highlightActor->GetProperty()->SetColor(1.0, 0.65, 0.10);
    m_highlightActor->GetProperty()->SetOpacity(0.55);
    m_highlightActor->GetProperty()->SetInterpolationToFlat();
    m_highlightActor->SetVisibility(0);
}

void BodyActor::setShape(const TopoDS_Shape& shape)
{
    m_tessellation = OcctShapeTessellator::tessellate(shape);
    m_hasShape = !m_tessellation.empty();

    // Only the mapper input is refreshed so the camera stays where the user
    // left it; the tessellator returns a brand new data object every time. A
    // texture outlives a rebuild, so its projection is recomputed from the new
    // mesh here as well.
    m_featureEdges->SetInputData(m_tessellation.mesh);
    m_featureEdges->Update();
    refreshSurfaceInput();

    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    clearHighlight();
    updateEdgeVisibility();
}

void BodyActor::clear()
{
    setShape(TopoDS_Shape());
}

bool BodyActor::hasShape() const noexcept
{
    return m_hasShape;
}

void BodyActor::setVisible(bool visible)
{
    m_visible = visible;
    m_surfaceActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
    m_highlightActor->SetVisibility(
        m_visible && m_hasShape && m_highlightedFace >= 0 ? 1 : 0);
    updateEdgeVisibility();
}

void BodyActor::setRepresentation(int representation)
{
    m_representation = representation;
    switch (representation) {
    case 1:
        m_surfaceActor->GetProperty()->SetRepresentationToWireframe();
        break;
    case 2:
        m_surfaceActor->GetProperty()->SetRepresentationToPoints();
        m_surfaceActor->GetProperty()->SetPointSize(4.0F);
        break;
    default:
        m_surfaceActor->GetProperty()->SetRepresentationToSurface();
        break;
    }
    updateEdgeVisibility();
}

void BodyActor::setOpacity(double opacity)
{
    m_surfaceActor->GetProperty()->SetOpacity(std::clamp(opacity, 0.0, 1.0));
    updatePreviewColors();
}

void BodyActor::setColor(const QColor& color)
{
    m_color = color;
    applyActorColor();
    updatePreviewColors();
}

void BodyActor::setLightingEnabled(bool enabled)
{
    m_lightingEnabled = enabled;
    m_surfaceActor->GetProperty()->SetLighting(enabled ? 1 : 0);
    applyMaterialPreviewShader();
}

void BodyActor::setMetallic(double metallic)
{
    m_metallic = std::clamp(metallic, 0.0, 1.0);
    updatePreviewColors();
}

void BodyActor::setRoughness(double roughness)
{
    m_roughness = std::clamp(roughness, 0.04, 1.0);
    updatePreviewColors();
}

void BodyActor::setMaterialPreviewEnabled(bool enabled)
{
    if (m_materialPreview == enabled) {
        return;
    }
    m_materialPreview = enabled;
    applyMaterialPreviewShader();
}

bool BodyActor::materialPreviewActive() const
{
    return m_materialPreview && m_lightingEnabled && m_hasShape;
}

void BodyActor::applyMaterialPreviewShader()
{
    auto* shader = m_surfaceActor->GetShaderProperty();
    shader->ClearAllShaderReplacements();
    m_previewShaderInstalled = false;
    m_shadowDirty = true;
    if (!materialPreviewActive()) {
        return;
    }

    // A body texture is the albedo map. Without one the vertex stage has no
    // texture coordinates, so a constant stands in and the samplers stay bound.
    const bool textured = m_textureEnabled && m_texture != nullptr && m_texturedMesh != nullptr;
    shader->AddVertexShaderReplacement(
        "//VTK::PositionVC::Dec", true,
        "//VTK::PositionVC::Dec\nout vec3 slWorldPosition;\nout vec3 slWorldNormal;\nout vec2 slSurfaceUV;\n",
        false);
    std::string vertexImplementation =
        "//VTK::PositionVC::Impl\n"
        "slWorldPosition=(slModelMatrix*vec4(vertexMC.xyz,1.0)).xyz;\n"
        "slWorldNormal=normalize((slNormalMatrix*vec4(normalMC,0.0)).xyz);\n";
    vertexImplementation += textured ? "slSurfaceUV=tcoord;\n" : "slSurfaceUV=vec2(0.5);\n";
    shader->AddVertexShaderReplacement("//VTK::PositionVC::Impl", true, vertexImplementation,
                                       false);
    shader->AddFragmentShaderReplacement("//VTK::PositionVC::Dec", true,
                                         shading::fragmentDeclarations, false);
    shader->AddFragmentShaderReplacement("//VTK::Light::Impl", true, shading::illumination,
                                         false);
    shader->AddFragmentShaderReplacement("//VTK::TCoord::Impl", true, "", false);

    if (m_whiteTexture == nullptr) {
        m_whiteTexture = makeWhiteTexture();
    }
    if (m_shadowData == nullptr) {
        m_shadowData = vtkSmartPointer<vtkImageData>::New();
        m_shadowData->SetDimensions(kPreviewShadowSize, kPreviewShadowSize, 1);
        m_shadowData->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
        std::fill_n(static_cast<unsigned char*>(m_shadowData->GetScalarPointer()),
                    kPreviewShadowSize * kPreviewShadowSize * 3,
                    static_cast<unsigned char>(255));
        m_shadowTexture = vtkSmartPointer<vtkTexture>::New();
        m_shadowTexture->SetInputData(m_shadowData);
        m_shadowTexture->SetColorModeToDirectScalars();
        m_shadowTexture->InterpolateOff();
        m_shadowTexture->RepeatOff();
        m_shadowTexture->EdgeClampOn();
    }

    // Material maps other than the body texture are a capture-only feature; the
    // preview binds white so the constants describe the surface on their own.
    auto* property = m_surfaceActor->GetProperty();
    property->SetTexture("slBaseMap", textured ? m_texture : m_whiteTexture);
    property->SetTexture("slMetalMap", m_whiteTexture);
    property->SetTexture("slRoughMap", m_whiteTexture);
    property->SetTexture("slPatternMap", m_whiteTexture);
    property->SetTexture("slShadowMap", m_shadowTexture);

    // Safe defaults so a render that slips in before the first camera update
    // shows an unlit-looking but valid surface instead of undefined uniforms.
    auto* uniforms = shader->GetFragmentCustomUniforms();
    const float neutral[3]{0.5f, 0.5f, 0.5f};
    uniforms->SetUniform3f("slBaseColor", neutral);
    uniforms->SetUniformf("slMetallic", static_cast<float>(m_metallic));
    uniforms->SetUniformf("slRoughness", static_cast<float>(m_roughness));
    // The viewport has one point light, so a purely metallic surface would be
    // black except at the highlight. A studio-like ambient fill stands in for
    // the environment the capture backend leaves out.
    uniforms->SetUniformf("slAmbient", 0.35f);
    uniforms->SetUniformf("slIntensity", 2.0f);
    uniforms->SetUniformf("slDisplaySpace", 1.0f);
    uniforms->SetUniformf("slExposure", 1.0f);
    uniforms->SetUniformf("slOpacity", static_cast<float>(property->GetOpacity()));
    uniforms->SetUniformf("slFx", kPreviewShadowSize * 0.5f);
    uniforms->SetUniformf("slFy", kPreviewShadowSize * 0.5f);
    uniforms->SetUniformf("slShadowFy", kPreviewShadowSize * 0.5f);
    uniforms->SetUniformf("slCx", kPreviewShadowSize * 0.5f);
    uniforms->SetUniformf("slCy", kPreviewShadowSize * 0.5f);
    uniforms->SetUniformf("slWidth", static_cast<float>(kPreviewShadowSize));
    uniforms->SetUniformf("slHeight", static_cast<float>(kPreviewShadowSize));
    uniforms->SetUniformf("slNear", 0.1f);
    uniforms->SetUniformf("slFar", 1000.0f);
    uniforms->SetUniformf("slReference2", 1.0f);
    vtkNew<vtkMatrix4x4> identity;
    uniforms->SetUniformMatrix("slWorldToProjector", identity);
    const float origin[3]{0.0f, 0.0f, 0.0f};
    uniforms->SetUniform3f("slProjectorCenter", origin);
    uniforms->SetUniform3f("slCameraCenter", origin);
    shader->GetVertexCustomUniforms()->SetUniformMatrix("slModelMatrix", identity);
    shader->GetVertexCustomUniforms()->SetUniformMatrix("slNormalMatrix", identity);

    m_previewShaderInstalled = true;
    updatePreviewColors();
}

void BodyActor::updatePreviewColors()
{
    if (!m_previewShaderInstalled) {
        return;
    }
    auto* uniforms = m_surfaceActor->GetShaderProperty()->GetFragmentCustomUniforms();
    const float base[3]{static_cast<float>(srgbToLinear(m_color.redF())),
                        static_cast<float>(srgbToLinear(m_color.greenF())),
                        static_cast<float>(srgbToLinear(m_color.blueF()))};
    uniforms->SetUniform3f("slBaseColor", base);
    uniforms->SetUniformf("slMetallic", static_cast<float>(m_metallic));
    uniforms->SetUniformf("slRoughness", static_cast<float>(m_roughness));
    uniforms->SetUniformf("slOpacity",
                          static_cast<float>(m_surfaceActor->GetProperty()->GetOpacity()));
}

void BodyActor::updatePreviewShadowMap(const double position[3], const double y[3],
                                       const double z[3], double focalLength,
                                       double nearPlane, double farPlane, int shadowSize)
{
    if (m_shadowRenderer == nullptr) {
        m_shadowRenderer = vtkSmartPointer<vtkRenderer>::New();
        m_shadowActor = vtkSmartPointer<vtkActor>::New();
        auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        mapper->ScalarVisibilityOff();
        m_shadowActor->SetMapper(mapper);
        m_shadowRenderer->AddActor(m_shadowActor);
        m_shadowWindow = vtkSmartPointer<vtkRenderWindow>::New();
        m_shadowWindow->SetOffScreenRendering(1);
        m_shadowWindow->SetMultiSamples(0);
        m_shadowWindow->SetSize(shadowSize, shadowSize);
        m_shadowWindow->AddRenderer(m_shadowRenderer);
    }
    m_shadowWindow->SetSize(shadowSize, shadowSize);
    if (m_shadowData->GetDimensions()[0] != shadowSize) {
        m_shadowData->SetDimensions(shadowSize, shadowSize, 1);
        m_shadowData->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    }
    auto* mapper = vtkPolyDataMapper::SafeDownCast(m_shadowActor->GetMapper());
    if (mapper == nullptr || m_surfaceMapper->GetInput() == nullptr) {
        return;
    }
    mapper->SetInputData(m_surfaceMapper->GetInput());
    // The display mesh lives in body coordinates, so the shadow pass carries the
    // same placement the surface actor has.
    vtkNew<vtkMatrix4x4> transform;
    transform->DeepCopy(m_surfaceActor->GetMatrix());
    m_shadowActor->SetUserMatrix(transform);

    applyProjectorCamera(m_shadowRenderer->GetActiveCamera(), position, y, z, focalLength,
                         nearPlane, farPlane, shadowSize);
    m_shadowWindow->Render();
    vtkNew<vtkWindowToImageFilter> depth;
    depth->SetInput(m_shadowWindow);
    depth->SetInputBufferTypeToZBuffer();
    depth->ReadFrontBufferOff();
    depth->ShouldRerenderOff();
    depth->Update();
    const auto* source = static_cast<float*>(depth->GetOutput()->GetScalarPointer());
    auto* destination = static_cast<unsigned char*>(m_shadowData->GetScalarPointer());
    for (int i = 0; i < shadowSize * shadowSize; ++i) {
        // Three bytes hold the depth exactly; a normalized texture would lose
        // precision in the driver and the bias could no longer be trusted.
        const auto packed =
            static_cast<unsigned int>(std::llround(std::clamp(source[i], 0.f, 1.f) * 16777215.0));
        destination[3 * i] = static_cast<unsigned char>(packed >> 16);
        destination[3 * i + 1] = static_cast<unsigned char>((packed >> 8) & 255);
        destination[3 * i + 2] = static_cast<unsigned char>(packed & 255);
    }
    m_shadowData->Modified();
}

void BodyActor::updateMaterialPreview(vtkCamera* camera, bool interacting)
{
    if (!materialPreviewActive() || !m_previewShaderInstalled || camera == nullptr) {
        return;
    }
    const int shadowSize = interacting ? 512 : kPreviewShadowSize;
    double cameraPosition[3];
    double cameraTarget[3];
    double cameraUp[3];
    camera->GetPosition(cameraPosition);
    camera->GetFocalPoint(cameraTarget);
    camera->GetViewUp(cameraUp);
    double forward[3]{cameraTarget[0] - cameraPosition[0], cameraTarget[1] - cameraPosition[1],
                      cameraTarget[2] - cameraPosition[2]};
    if (!normalize3(forward)) {
        return;
    }
    double right[3];
    cross3(forward, cameraUp, right);
    if (!normalize3(right)) {
        return;
    }
    double up[3];
    cross3(right, forward, up);
    normalize3(up);
    const double viewDistance = camera->GetDistance();

    double bounds[6];
    m_surfaceActor->GetBounds(bounds);
    const double center[3]{(bounds[0] + bounds[1]) * 0.5, (bounds[2] + bounds[3]) * 0.5,
                           (bounds[4] + bounds[5]) * 0.5};
    const double diagonal[3]{bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]};
    const double radius = 0.5 * length3(diagonal);
    if (!(radius > 1.0e-9) || !(viewDistance > 1.0e-9)) {
        return;
    }

    // The projector sits where the scan rig puts it: rigidly attached to the
    // camera, offset up and to the side, so the visible side is always lit.
    const double lightPosition[3]{
        cameraPosition[0] + right[0] * viewDistance * 0.35 + up[0] * viewDistance * 0.45,
        cameraPosition[1] + right[1] * viewDistance * 0.35 + up[1] * viewDistance * 0.45,
        cameraPosition[2] + right[2] * viewDistance * 0.35 + up[2] * viewDistance * 0.45};
    double toCenter[3]{center[0] - lightPosition[0], center[1] - lightPosition[1],
                       center[2] - lightPosition[2]};
    const double lightDistance = std::max(length3(toCenter), radius * 1.01);
    double halfAngle = std::asin(clampValue(radius / lightDistance, 0.0, 0.95));
    // The frustum has to hug the body: the fixed 1024 shadow map would otherwise
    // spend almost every texel on empty space and the remaining few would
    // self-shadow the surface.
    halfAngle = clampValue(halfAngle * 1.25, 0.002, 0.9);
    const double focalLength = (shadowSize * 0.5) / std::tan(halfAngle);
    const double nearPlane = std::max(1.0e-3, lightDistance - radius * 1.6);
    const double farPlane = lightDistance + radius * 1.6;

    double lightX[3];
    double lightY[3];
    double lightZ[3];
    lookAtBasis(lightPosition, center, cameraUp, lightX, lightY, lightZ);

    double actorMatrix[16];
    vtkMatrix4x4* modelMatrix = m_surfaceActor->GetMatrix();
    for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
            actorMatrix[row * 4 + column] = modelMatrix->GetElement(row, column);
        }
    }
    const bool stale =
        m_shadowDirty || m_shadowData->GetDimensions()[0] != shadowSize ||
        std::memcmp(lightPosition, m_shadowViewPosition, sizeof(m_shadowViewPosition)) != 0 ||
        std::memcmp(center, m_shadowViewTarget, sizeof(m_shadowViewTarget)) != 0 ||
        std::memcmp(lightY, m_shadowViewUp, sizeof(m_shadowViewUp)) != 0 ||
        std::memcmp(actorMatrix, m_previewActorMatrix, sizeof(m_previewActorMatrix)) != 0;
    if (stale) {
        updatePreviewShadowMap(lightPosition, lightY, lightZ, focalLength, nearPlane,
                               farPlane, shadowSize);
        std::copy(lightPosition, lightPosition + 3, m_shadowViewPosition);
        std::copy(center, center + 3, m_shadowViewTarget);
        std::copy(lightY, lightY + 3, m_shadowViewUp);
        std::copy(actorMatrix, actorMatrix + 16, m_previewActorMatrix);
        m_shadowDirty = false;
    }

    auto* shader = m_surfaceActor->GetShaderProperty();
    auto* vertexUniforms = shader->GetVertexCustomUniforms();
    vertexUniforms->SetUniformMatrix("slModelMatrix", modelMatrix);
    vtkNew<vtkMatrix4x4> normalMatrix;
    normalMatrix->DeepCopy(modelMatrix);
    normalMatrix->Invert();
    normalMatrix->Transpose();
    vertexUniforms->SetUniformMatrix("slNormalMatrix", normalMatrix);

    auto pose = poseMatrix(lightPosition, lightX, lightY, lightZ);
    vtkNew<vtkMatrix4x4> worldToProjector;
    vtkMatrix4x4::Invert(pose, worldToProjector);
    // vtkUniforms' flat matrix upload uses OpenGL column-major ordering.
    worldToProjector->Transpose();
    auto* uniforms = shader->GetFragmentCustomUniforms();
    uniforms->SetUniformMatrix("slWorldToProjector", worldToProjector);
    uniforms->SetUniform3f("slProjectorCenter", lightPosition);
    uniforms->SetUniform3f("slCameraCenter", cameraPosition);
    uniforms->SetUniformf("slReference2", viewDistance * viewDistance);
    uniforms->SetUniformf("slFx", static_cast<float>(focalLength));
    uniforms->SetUniformf("slFy", static_cast<float>(focalLength));
    uniforms->SetUniformf("slShadowFy", static_cast<float>(focalLength));
    uniforms->SetUniformf("slCx", shadowSize * 0.5f);
    uniforms->SetUniformf("slCy", shadowSize * 0.5f);
    uniforms->SetUniformf("slWidth", static_cast<float>(shadowSize));
    uniforms->SetUniformf("slHeight", static_cast<float>(shadowSize));
    uniforms->SetUniformf("slNear", static_cast<float>(nearPlane));
    uniforms->SetUniformf("slFar", static_cast<float>(farPlane));
}

bool BodyActor::setTextureFromFile(const QString& imagePath, QString& error)
{
    const QByteArray fileName = imagePath.toUtf8();
    vtkSmartPointer<vtkImageReader2> reader =
        vtkImageReader2Factory::CreateImageReader2(fileName.constData());
    if (reader == nullptr) {
        error = QCoreApplication::translate("BodyActor", "不支持的纹理格式：%1").arg(imagePath);
        return false;
    }
    reader->SetFileName(fileName.constData());
    reader->Update();
    vtkImageData* image = reader->GetOutput();
    if (image == nullptr || image->GetNumberOfPoints() == 0) {
        error = QCoreApplication::translate("BodyActor", "无法读取纹理图像：%1").arg(imagePath);
        return false;
    }

    m_textureReader = reader;
    m_texture = vtkSmartPointer<vtkTexture>::New();
    m_texture->SetInputConnection(m_textureReader->GetOutputPort());
    m_texture->InterpolateOn();
    // The coordinates are normalised to the bounding box, so the picture covers
    // the body exactly once and never tiles across it.
    m_texture->RepeatOff();
    m_textureEnabled = true;
    refreshSurfaceInput();
    return true;
}

vtkSmartPointer<vtkImageData> BodyActor::prepareTexture(const QImage& source)
{
    if (source.isNull()) return nullptr;
    const auto image = source.convertToFormat(QImage::Format_RGBA8888);
    auto data = vtkSmartPointer<vtkImageData>::New();
    data->SetDimensions(image.width(), image.height(), 1);
    data->AllocateScalars(VTK_UNSIGNED_CHAR, 4);
    for (int y = 0; y < image.height(); ++y) {
        // Match the bottom-up convention of VTK's image readers.
        std::memcpy(data->GetScalarPointer(0, y, 0), image.constScanLine(image.height() - y - 1),
                    static_cast<size_t>(image.width()) * 4);
    }
    return data;
}

void BodyActor::setTextureData(vtkImageData* image)
{
    m_textureReader = nullptr;
    m_texture = vtkSmartPointer<vtkTexture>::New();
    m_texture->SetInputData(image);
    m_texture->InterpolateOn();
    m_texture->RepeatOff();
    m_textureEnabled = true;
    refreshSurfaceInput();
}

void BodyActor::setTextureEnabled(bool enabled)
{
    if (m_textureEnabled == enabled) {
        return;
    }
    m_textureEnabled = enabled;
    refreshSurfaceInput();
}

void BodyActor::setTextureProjection(int axis)
{
    const int clamped = std::clamp(axis, 0, 4);
    if (m_textureAxis == clamped) {
        return;
    }
    m_textureAxis = clamped;
    if (m_textureEnabled && m_texture != nullptr) {
        refreshSurfaceInput();
    }
}

void BodyActor::clearTexture()
{
    m_textureEnabled = false;
    m_texture = nullptr;
    m_textureReader = nullptr;
    refreshSurfaceInput();
    m_texturedMesh = nullptr;
}

bool BodyActor::hasTexture() const noexcept
{
    return m_texture != nullptr;
}

int BodyActor::textureProjection() const noexcept
{
    return m_textureAxis;
}

void BodyActor::refreshSurfaceInput()
{
    const bool textured = m_textureEnabled && m_texture != nullptr && m_hasShape;
    if (textured) {
        m_texturedMesh = projectTextureCoordinates(m_tessellation.mesh, m_textureAxis);
        m_surfaceMapper->SetInputData(m_texturedMesh);
        m_surfaceActor->SetTexture(m_texture);
    } else {
        m_surfaceActor->SetTexture(nullptr);
        m_surfaceMapper->SetInputData(m_tessellation.mesh);
    }
    applyActorColor();
    m_surfaceMapper->Update();
    // The preview shader is tied to the mapper input and its texture
    // coordinates, so it is rebuilt whenever the surface input changes.
    applyMaterialPreviewShader();
}

void BodyActor::applyActorColor()
{
    if (m_textureEnabled && m_texture != nullptr && m_hasShape) {
        // The picture is the colour of the surface, so the object tint drops to
        // white; anything else would multiply the texture and wash it out.
        m_surfaceActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
        return;
    }
    m_surfaceActor->GetProperty()->SetColor(m_color.redF(), m_color.greenF(), m_color.blueF());
}

vtkSmartPointer<vtkPolyData> BodyActor::projectTextureCoordinates(vtkPolyData* mesh,
                                                                 int axis) const
{
    auto projected = vtkSmartPointer<vtkPolyData>::New();
    if (mesh == nullptr) {
        return projected;
    }
    // The points and polygons are shared, but the point data is copied into a
    // fresh container so the texture coordinates never leak into the mesh the
    // face index and the outline are read from.
    projected->SetPoints(mesh->GetPoints());
    projected->SetPolys(mesh->GetPolys());
    projected->GetPointData()->ShallowCopy(mesh->GetPointData());

    vtkPoints* points = mesh->GetPoints();
    if (points == nullptr || points->GetNumberOfPoints() == 0) {
        return projected;
    }

    double bounds[6];
    mesh->GetBounds(bounds);
    double extent[3] = {bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]};

    // A perfectly flat body would divide by zero; a zero-sized side keeps its
    // coordinates at the lower bound instead.
    const double size[3] = {extent[0] > 1.0e-9 ? extent[0] : 1.0,
                            extent[1] > 1.0e-9 ? extent[1] : 1.0,
                            extent[2] > 1.0e-9 ? extent[2] : 1.0};

    // Axis 0 means "let the body decide": the picture is thrown at it from the
    // side it is widest, which keeps the projection from smearing on a thin
    // direction. 1/2/3 pin the direction to X/Y/Z. 4 projects every face from
    // the direction that face looks at.
    int autoAxis = 0;
    for (int i = 1; i < 3; ++i) {
        if (extent[i] > extent[autoAxis]) {
            autoAxis = i;
        }
    }

    // Projects one point from the given direction. Every direction normalises
    // against the whole body's bounds, so the faces of a box stay in register
    // with each other instead of each stretching the picture across itself.
    const auto project = [&](const double* point, int normalAxis, double& u, double& v) {
        const int uAxis = (normalAxis + 1) % 3;
        const int vAxis = (normalAxis + 2) % 3;
        u = (point[uAxis] - bounds[2 * uAxis]) / size[uAxis];
        v = (point[vAxis] - bounds[2 * vAxis]) / size[vAxis];
    };

    vtkNew<vtkFloatArray> coordinates;
    coordinates->SetName("TextureCoordinates");
    coordinates->SetNumberOfComponents(2);
    coordinates->SetNumberOfTuples(points->GetNumberOfPoints());

    if (axis != 4) {
        const int normalAxis = axis >= 1 ? axis - 1 : autoAxis;
        for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
            double u = 0.0;
            double v = 0.0;
            project(points->GetPoint(i), normalAxis, u, v);
            coordinates->SetTuple2(i, u, v);
        }
        projected->GetPointData()->SetTCoords(coordinates);
        return projected;
    }

    // Per-face (tri-planar) mode. A single direction cannot face all six sides
    // of a box, so the sides it misses are smeared; picking the direction from
    // each face's own normal gives every side a square-on projection. The
    // tessellator gives every face its own points, so one direction per point is
    // enough and no vertex has to be split.
    vtkDataArray* faceIds = mesh->GetCellData()->GetArray("FaceIds");
    std::vector<int> pointAxis(static_cast<std::size_t>(points->GetNumberOfPoints()), -1);
    for (vtkIdType cell = 0; cell < mesh->GetNumberOfCells(); ++cell) {
        double normal[3] = {0.0, 0.0, 1.0};
        const int faceIndex =
            faceIds != nullptr && cell < faceIds->GetNumberOfTuples()
                ? static_cast<int>(faceIds->GetComponent(cell, 0))
                : -1;
        const BodyFaceInfo* info = faceInfo(faceIndex);
        if (info != nullptr && info->planar) {
            std::copy(info->normal, info->normal + 3, normal);
        } else if (!cellNormal(mesh, cell, normal)) {
            continue;
        }

        const int faceAxis = dominantAxis(normal);
        vtkIdType pointCount = 0;
        const vtkIdType* pointIds = nullptr;
        mesh->GetCellPoints(cell, pointCount, pointIds);
        for (vtkIdType k = 0; k < pointCount; ++k) {
            if (pointAxis[static_cast<std::size_t>(pointIds[k])] < 0) {
                pointAxis[static_cast<std::size_t>(pointIds[k])] = faceAxis;
            }
        }
    }

    for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
        const int pointDirection = pointAxis[static_cast<std::size_t>(i)];
        double u = 0.0;
        double v = 0.0;
        project(points->GetPoint(i), pointDirection >= 0 ? pointDirection : autoAxis, u, v);
        coordinates->SetTuple2(i, u, v);
    }
    projected->GetPointData()->SetTCoords(coordinates);
    return projected;
}

void BodyActor::setTransform(double px, double py, double pz,
                             double rx, double ry, double rz,
                             double sx, double sy, double sz)
{
    vtkActor* actors[] = {m_surfaceActor, m_edgeActor, m_highlightActor};
    for (vtkActor* actor : actors) {
        actor->SetPosition(px, py, pz);
        actor->SetOrientation(rx, ry, rz);
        actor->SetScale(sx, sy, sz);
    }
}

ObjectPresentation BodyActor::presentation() const
{
    ObjectPresentation state;
    m_surfaceActor->GetPosition(state.position.data()); m_surfaceActor->GetOrientation(state.rotation.data());
    m_surfaceActor->GetScale(state.scale.data()); state.visible=m_visible; state.representation=m_representation;
    state.opacity=m_surfaceActor->GetProperty()->GetOpacity(); state.color=m_color;
    state.metallic=m_metallic; state.roughness=m_roughness;
    state.materialPreview=m_materialPreview;
    return state;
}

vtkActor* BodyActor::surfaceActor() const noexcept
{
    return m_surfaceActor;
}

vtkActor* BodyActor::edgeActor() const noexcept
{
    return m_edgeActor;
}

int BodyActor::faceCount() const noexcept
{
    return static_cast<int>(m_tessellation.faces.size());
}

const BodyFaceInfo* BodyActor::faceInfo(int faceIndex) const noexcept
{
    if (faceIndex < 0 || faceIndex >= faceCount()) {
        return nullptr;
    }
    return &m_tessellation.faces[static_cast<std::size_t>(faceIndex)];
}

int BodyActor::faceIdOfCell(vtkIdType cellId) const noexcept
{
    if (m_tessellation.mesh == nullptr || cellId < 0) {
        return -1;
    }
    vtkDataArray* ids = m_tessellation.mesh->GetCellData()->GetArray("FaceIds");
    if (ids == nullptr || cellId >= ids->GetNumberOfTuples()) {
        return -1;
    }
    return static_cast<int>(ids->GetComponent(cellId, 0));
}

void BodyActor::highlightFace(int faceIndex)
{
    const BodyFaceInfo* info = faceInfo(faceIndex);
    if (info == nullptr || info->cellCount <= 0) {
        clearHighlight();
        return;
    }

    vtkPolyData* source = m_tessellation.mesh;
    vtkNew<vtkCellArray> selection;
    const vtkIdType lastCell = info->firstCell + info->cellCount;
    for (vtkIdType cell = info->firstCell; cell < lastCell; ++cell) {
        vtkIdType pointCount = 0;
        const vtkIdType* pointIds = nullptr;
        source->GetCellPoints(cell, pointCount, pointIds);
        selection->InsertNextCell(pointCount, pointIds);
    }

    vtkNew<vtkPolyData> overlay;
    overlay->SetPoints(source->GetPoints());
    overlay->SetPolys(selection);
    m_highlightMapper->SetInputData(overlay);

    m_highlightedFace = faceIndex;
    m_highlightActor->SetVisibility(m_visible && m_hasShape ? 1 : 0);
}

void BodyActor::clearHighlight()
{
    m_highlightedFace = -1;
    m_highlightMapper->SetInputData(vtkSmartPointer<vtkPolyData>::New());
    m_highlightActor->SetVisibility(0);
}

vtkActor* BodyActor::highlightActor() const noexcept
{
    return m_highlightActor;
}

void BodyActor::updateEdgeVisibility()
{
    const bool showEdges = m_visible && m_hasShape && m_representation == 0;
    m_edgeActor->SetVisibility(showEdges ? 1 : 0);
}
