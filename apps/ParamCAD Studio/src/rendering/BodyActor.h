#pragma once

#include "rendering/OcctShapeTessellator.h"
#include "core/PresentationState.h"

#include <vtkSmartPointer.h>

#include <QColor>
#include <QString>

class TopoDS_Shape;
class vtkActor;
class vtkCamera;
class vtkFeatureEdges;
class vtkImageReader2;
class vtkImageData;
class QImage;
class vtkPolyData;
class vtkPolyDataMapper;
class vtkRenderer;
class vtkRenderWindow;
class vtkTexture;

// Owns the VTK pipeline that draws one modeled body: a shaded surface plus a
// black outline of the sharp edges. It is a display cache only - the real
// model state lives in modeling::ModelingCore.
//
// Besides the display mesh it keeps the per-face index produced by the
// tessellator so the viewport can map a picked display cell back to an
// OpenCASCADE face and highlight it.
class BodyActor
{
public:
    BodyActor();

    void setShape(const TopoDS_Shape& shape);
    void clear();

    bool hasShape() const noexcept;
    ObjectPresentation presentation() const;
    bool textureEnabled() const { return m_textureEnabled; }

    void setVisible(bool visible);
    void setRepresentation(int representation);
    void setOpacity(double opacity);
    void setColor(const QColor& color);
    void setLightingEnabled(bool enabled);

    // Physical surface shown by the material preview. The same values feed the
    // structured-light capture, so the preview is what the scan will see.
    void setMetallic(double metallic);
    void setRoughness(double roughness);
    void setMaterialPreviewEnabled(bool enabled);

    // Refreshes the preview light and its projector shadow map from the live
    // camera. A no-op while the preview is off, unlit, or has no shape.
    void updateMaterialPreview(vtkCamera* camera, bool interacting = false);

    // Casts an image onto the body without unwrapping anything: the texture
    // coordinates are a straight planar projection of the display mesh, so the
    // picture is simply thrown onto the model from one side. Axis 0 lets the
    // widest side of the body choose the direction, 1/2/3 pin it to X/Y/Z and
    // 4 projects every face from the direction that face looks at, which is
    // what keeps the picture square on every side of a box.
    bool setTextureFromFile(const QString& imagePath, QString& error);
    static vtkSmartPointer<vtkImageData> prepareTexture(const QImage& image);
    void setTextureData(vtkImageData* image);
    void setTextureEnabled(bool enabled);
    void setTextureProjection(int axis);
    void clearTexture();
    bool hasTexture() const noexcept;
    int textureProjection() const noexcept;
    void setTransform(double px, double py, double pz,
                      double rx, double ry, double rz,
                      double sx, double sy, double sz);

    vtkActor* surfaceActor() const noexcept;
    vtkActor* edgeActor() const noexcept;

    // Face index lookups. A cell that carries no face index returns -1.
    int faceCount() const noexcept;
    const BodyFaceInfo* faceInfo(int faceIndex) const noexcept;
    int faceIdOfCell(vtkIdType cellId) const noexcept;

    // Draws a translucent overlay on the given face; pass a negative index to
    // clear it. The overlay follows the body transform and is polygon-offset so
    // it never z-fights with the shaded surface.
    void highlightFace(int faceIndex);
    void clearHighlight();
    vtkActor* highlightActor() const noexcept;

private:
    void updateEdgeVisibility();

    // Rebuilds the mapper input for the current texture state and repaints the
    // actor colour that goes with it. Called whenever the mesh, the texture or
    // the projection direction changes.
    void refreshSurfaceInput();
    void applyActorColor();
    // Installs or removes the projected-light GGX shader used by the material
    // preview. Reinstalled whenever the display mesh or its texture changes.
    void applyMaterialPreviewShader();
    bool materialPreviewActive() const;
    void updatePreviewColors();
    void updatePreviewShadowMap(const double position[3], const double y[3],
                                const double z[3], double focalLength, double nearPlane,
                                double farPlane, int shadowSize);
    // A copy of the display mesh carrying 2D texture coordinates obtained by
    // projecting every point onto the plane chosen by the given axis.
    vtkSmartPointer<vtkPolyData> projectTextureCoordinates(vtkPolyData* mesh,
                                                           int axis) const;

    BodyTessellation m_tessellation;
    vtkSmartPointer<vtkPolyDataMapper> m_surfaceMapper;
    vtkSmartPointer<vtkActor> m_surfaceActor;
    vtkSmartPointer<vtkFeatureEdges> m_featureEdges;
    vtkSmartPointer<vtkPolyDataMapper> m_edgeMapper;
    vtkSmartPointer<vtkActor> m_edgeActor;
    vtkSmartPointer<vtkPolyDataMapper> m_highlightMapper;
    vtkSmartPointer<vtkActor> m_highlightActor;
    vtkSmartPointer<vtkImageReader2> m_textureReader;
    vtkSmartPointer<vtkTexture> m_texture;
    vtkSmartPointer<vtkPolyData> m_texturedMesh;
    // Material preview: a projector light rigidly attached to the viewport
    // camera plus its depth map, mirroring the structured-light rig.
    vtkSmartPointer<vtkTexture> m_whiteTexture;
    vtkSmartPointer<vtkTexture> m_shadowTexture;
    vtkSmartPointer<vtkImageData> m_shadowData;
    vtkSmartPointer<vtkRenderer> m_shadowRenderer;
    vtkSmartPointer<vtkRenderWindow> m_shadowWindow;
    vtkSmartPointer<vtkActor> m_shadowActor;
    bool m_previewShaderInstalled = false;
    bool m_shadowDirty = true;
    bool m_materialPreview = true;
    bool m_lightingEnabled = true;
    double m_metallic = 0.9;
    double m_roughness = 0.4;
    double m_shadowViewPosition[3]{0, 0, 0};
    double m_shadowViewTarget[3]{0, 0, 0};
    double m_shadowViewUp[3]{0, 0, 1};
    double m_previewActorMatrix[16]{};
    QColor m_color{71, 145, 214};
    bool m_textureEnabled = false;
    int m_textureAxis = 0;
    bool m_hasShape = false;
    bool m_visible = true;
    int m_representation = 0;
    int m_highlightedFace = -1;
};
