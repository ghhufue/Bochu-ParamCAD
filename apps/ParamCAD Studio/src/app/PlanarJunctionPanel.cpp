#include "app/PlanarJunctionPanel.h"

#include "core/PointCloudDocument.h"
#include "io/SurfaceEvidenceAdapter.h"
#include "ui/viewport/VTKViewer.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QThread>
#include <QVBoxLayout>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>

#include <algorithm>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace {

using reconstruction::PlanarSurfaceInput;

void copyMesh(PlanarSurfaceInput& destination, const PointCloudSurfaceMesh& mesh)
{
    destination.vertices.clear();
    destination.triangles.clear();
    destination.vertices.reserve(mesh.positions.size());
    destination.triangles.reserve(mesh.triangles.size());
    for (const auto& [x, y, z] : mesh.positions) destination.vertices.push_back({x, y, z});
    for (const auto& [a, b, c] : mesh.triangles)
        destination.triangles.push_back({std::size_t(a), std::size_t(b), std::size_t(c)});
}

void restoreMesh(AnalysisBundlePlane& plane)
{
    if (!plane.surface.triangles.empty()) return;
    PointCloudRegion region;
    region.id = plane.surface.id;
    region.geometry = QStringLiteral("plane");
    const auto [n, offset] = plane.surface.plane;
    region.parameters = {{"normal", QJsonArray{n.x, n.y, n.z}}, {"offset", offset}};
    PlyCloud cloud;
    cloud.positions.reserve(plane.observations.size());
    for (const auto point : plane.observations)
        cloud.positions.push_back({float(point.x), float(point.y), float(point.z)});
    std::vector<vtkIdType> rows(cloud.positions.size());
    std::iota(rows.begin(), rows.end(), vtkIdType{0});
    copyMesh(plane.surface, meshPointCloudSurface(region, cloud, rows));
}

vtkRenderer* renderer(VTKViewer* viewer)
{
    return viewer->renderWindow()->GetRenderers()->GetFirstRenderer();
}

vtkSmartPointer<vtkActor> actor(vtkPolyData* data, const std::array<double, 3>& color, double lineWidth = 0)
{
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(data);
    auto result = vtkSmartPointer<vtkActor>::New();
    result->SetMapper(mapper);
    result->PickableOff();
    result->GetProperty()->SetColor(color[0], color[1], color[2]);
    if (lineWidth > 0) {
        result->GetProperty()->LightingOff();
        result->GetProperty()->SetLineWidth(float(lineWidth));
        mapper->SetResolveCoincidentTopologyToPolygonOffset();
        mapper->SetRelativeCoincidentTopologyLineOffsetParameters(0, lineWidth > 3 ? -8 : -4);
    }
    return result;
}

vtkSmartPointer<vtkActor> surfaceActor(const reconstruction::FittedPlanarSurface& surface,
                                      const std::array<double, 3>& color)
{
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    vtkNew<vtkCellArray> cells;
    for (const auto p : surface.vertices) points->InsertNextPoint(p.x, p.y, p.z);
    for (const auto& [a, b, c] : surface.triangles) {
        const vtkIdType ids[]{vtkIdType(a), vtkIdType(b), vtkIdType(c)};
        cells->InsertNextCell(3, ids);
    }
    vtkNew<vtkPolyData> data;
    data->SetPoints(points);
    data->SetPolys(cells);
    return actor(data, color);
}

void addLine(vtkPoints* points, vtkCellArray* cells, const std::vector<modeling::Vec3>& vertices, bool closed)
{
    std::vector<vtkIdType> ids;
    ids.reserve(vertices.size() + 1);
    for (const auto p : vertices) ids.push_back(points->InsertNextPoint(p.x, p.y, p.z));
    if (closed) ids.push_back(ids.front());
    cells->InsertNextCell(static_cast<vtkIdType>(ids.size()), ids.data());
}

vtkSmartPointer<vtkActor> lineActor(vtkPoints* points, vtkCellArray* cells,
                                  const std::array<double, 3>& color, double width)
{
    vtkNew<vtkPolyData> data;
    data->SetPoints(points);
    data->SetLines(cells);
    return actor(data, color, width);
}

} // namespace

PlanarJunctionPanel::PlanarJunctionPanel(PointCloudDocument* document, VTKViewer* documentViewer, QWidget* parent)
    : QWidget(parent), m_document(document), m_documentViewer(documentViewer)
{
    setObjectName(QStringLiteral("planarJunctionPanel"));
    auto* controls = new QVBoxLayout(this);
    controls->setContentsMargins(4, 6, 4, 4);
    auto* sources = new QHBoxLayout;
    m_openBundle = new QPushButton(tr("打开分析包…"), this);
    m_useDocument = new QPushButton(tr("使用当前点云"), this);
    sources->addWidget(m_openBundle);
    sources->addWidget(m_useDocument);
    controls->addLayout(sources);
    connect(m_openBundle, &QPushButton::clicked, this, [this] {
        const auto path = QFileDialog::getExistingDirectory(this, tr("选择 .pcad-analysis 目录"), m_bundle);
        if (path.isEmpty()) return;
        QString error;
        if (!openBundle(path, error)) m_status->setText(error);
    });
    connect(m_useDocument, &QPushButton::clicked, this, &PlanarJunctionPanel::useDocument);
    m_filter = new QLineEdit(this);
    m_filter->setPlaceholderText(tr("按名称或 ID 筛选平面"));
    m_planes = new QListWidget(this);
    m_planes->setObjectName(QStringLiteral("planarJunctionFaces"));
    m_fit = new QPushButton(tr("拟合所选面片"), this);
    m_fit->setObjectName(QStringLiteral("fitPlanarJunctionsButton"));
    m_inspectBoundary = new QPushButton(tr("查看平面边界"), this);
    m_inspectBoundary->setObjectName(QStringLiteral("inspectPlanarJunctionBoundaryButton"));
    auto* fitActions = new QHBoxLayout;
    fitActions->addWidget(m_fit);
    fitActions->addWidget(m_inspectBoundary);
    m_openPreview = new QPushButton(tr("显示交线视图"), this);
    m_openPreview->setObjectName(QStringLiteral("showPlanarJunctionViewportButton"));
    m_fitCamera = new QPushButton(tr("适应视图"), this);
    auto* viewActions = new QHBoxLayout;
    viewActions->addWidget(m_openPreview);
    viewActions->addWidget(m_fitCamera);
    m_visibleFace = new QComboBox(this);
    m_visibleFace->setObjectName(QStringLiteral("planarJunctionVisibleFace"));
    m_boundaries = new QCheckBox(tr("显示边界"), this);
    m_boundaries->setChecked(true);
    m_intersections = new QCheckBox(tr("显示交线"), this);
    m_intersections->setChecked(true);
    m_report = new QPlainTextEdit(this);
    m_report->setObjectName(QStringLiteral("planarJunctionReport"));
    m_report->setReadOnly(true);

    controls->addWidget(m_filter);
    controls->addWidget(m_planes, 2);
    controls->addLayout(fitActions);
    controls->addLayout(viewActions);
    controls->addWidget(m_visibleFace);
    controls->addWidget(m_boundaries);
    controls->addWidget(m_intersections);
    controls->addWidget(m_report, 1);
    m_status = new QLabel(this);
    m_status->setWordWrap(true);
    controls->addWidget(m_status);
    connect(m_filter, &QLineEdit::textChanged, this, [this](const QString& value) {
        const auto query = value.trimmed();
        for (int i = 0; i < m_planes->count(); ++i) {
            auto* item = m_planes->item(i);
            item->setHidden(!item->text().contains(query, Qt::CaseInsensitive));
        }
    });
    connect(m_planes, &QListWidget::itemChanged, this, &PlanarJunctionPanel::updateFitAction);
    connect(m_planes, &QListWidget::currentItemChanged, this, &PlanarJunctionPanel::updateFitAction);
    connect(m_fit, &QPushButton::clicked, this, &PlanarJunctionPanel::fitSelected);
    connect(m_inspectBoundary, &QPushButton::clicked, this, [this] {
        if (const auto* item = m_planes->currentItem(); item && m_bundle.isEmpty())
            emit boundaryInspectionRequested(item->data(Qt::UserRole).toInt());
    });
    connect(m_openPreview, &QPushButton::clicked, this, &PlanarJunctionPanel::showPreview);
    connect(m_fitCamera, &QPushButton::clicked, this, [this] {
        showPreview();
        m_viewer->resetCamera();
    });
    connect(m_visibleFace, &QComboBox::currentIndexChanged, this, [this] {
        updateVisibility();
        if (m_visibleFace->count() > 1) m_viewer->resetCamera();
    });
    connect(m_boundaries, &QCheckBox::toggled, this, &PlanarJunctionPanel::updateVisibility);
    connect(m_intersections, &QCheckBox::toggled, this, &PlanarJunctionPanel::updateVisibility);
    connect(m_document, &PointCloudDocument::documentChanged, this, [this] {
        if (m_bundle.isEmpty() && m_surfaceRevision != m_document->selection().surfaceRevision())
            useDocument();
    });

    useDocument();
}

PlanarJunctionPanel::~PlanarJunctionPanel()
{
    if (m_worker) {
        m_worker->requestInterruption();
        m_worker->wait();
    }
    delete m_viewer.data();
}

VTKViewer* PlanarJunctionPanel::viewport() const
{
    return m_viewer.data();
}

void PlanarJunctionPanel::showPreview()
{
    if (!m_viewer) {
        m_viewer = new VTKViewer(this, nullptr, false);
        m_viewer->setObjectName(QStringLiteral("planarJunctionViewport"));
        m_viewer->setDatumPlanesVisible(false);
    }
    emit viewportRequested(m_viewer.data());
}

bool PlanarJunctionPanel::openBundle(const QString& directory, QString& error)
{
    if (m_worker) return false;
    try {
        auto planes = listAnalysisBundlePlanes(directory);
        m_bundle = directory;
        listPlanes(planes);
        m_status->setText(tr("分析包：%1").arg(QDir::toNativeSeparators(directory)));
        return true;
    } catch (const std::exception& failure) {
        error = QString::fromUtf8(failure.what());
        return false;
    }
}

void PlanarJunctionPanel::useDocument()
{
    if (m_worker) m_worker->requestInterruption();
    m_bundle.clear();
    const auto& selection = m_document->selection();
    m_surfaceRevision = selection.surfaceRevision();
    std::vector<AnalysisBundlePlaneEntry> entries;
    for (const auto& region : selection.regions()) {
        if (region.geometry == "plane" && regionEvidence(region))
            entries.push_back({region.id, region.name});
    }
    listPlanes(entries);
    m_status->setText(entries.empty() ? tr("请打开分析包，或先在主窗口识别平面。")
                                    : tr("当前点云：%1").arg(m_document->sourcePath()));
}

void PlanarJunctionPanel::listPlanes(const std::vector<AnalysisBundlePlaneEntry>& entries)
{
    const QSignalBlocker blocker(m_planes);
    m_planes->clear();
    m_filter->clear();
    for (const auto& entry : entries) {
        auto* item = new QListWidgetItem(QStringLiteral("%1 (#%2)").arg(entry.name).arg(entry.id), m_planes);
        item->setData(Qt::UserRole, entry.id);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
    }
    clearFit();
    updateFitAction();
}

QString PlanarJunctionPanel::planeName(int id) const
{
    for (int i = 0; i < m_planes->count(); ++i) {
        const auto* item = m_planes->item(i);
        if (item->data(Qt::UserRole).toInt() == id) return item->text();
    }
    return QString::number(id);
}

void PlanarJunctionPanel::updateFitAction()
{
    m_openBundle->setEnabled(!m_worker);
    m_useDocument->setEnabled(!m_worker);
    int checked = 0;
    for (int i = 0; i < m_planes->count(); ++i)
        checked += m_planes->item(i)->checkState() == Qt::Checked;
    m_fit->setEnabled(!m_worker && checked > 0);
    m_inspectBoundary->setEnabled(m_bundle.isEmpty() && m_planes->currentItem());
    m_openPreview->setEnabled(!m_surfaces.empty());
    m_fitCamera->setEnabled(!m_surfaces.empty());
}

void PlanarJunctionPanel::fitSelected()
{
    if (m_worker) return;
    std::vector<int> ids;
    for (int i = 0; i < m_planes->count(); ++i) {
        const auto* item = m_planes->item(i);
        if (item->checkState() == Qt::Checked) ids.push_back(item->data(Qt::UserRole).toInt());
    }
    if (ids.empty()) return;
    const auto bundle = m_bundle;
    const auto revision = m_surfaceRevision;
    std::vector<AnalysisBundlePlane> inputs;
    if (bundle.isEmpty()) {
        const auto& selection = m_document->selection();
        for (const auto id : ids) {
            const auto* region = selection.region(id);
            const auto evidence = region ? regionEvidence(*region) : std::nullopt;
            const auto* plane = evidence ? std::get_if<reconstruction::PlaneEvidence>(&*evidence) : nullptr;
            if (!plane) {
                m_status->setText(tr("平面已更改，请重新选择。"));
                return;
            }
            auto& input = inputs.emplace_back();
            input.surface.id = id;
            input.surface.plane = plane->plane;
            input.surface.residual = region->residual;
            if (const auto* mesh = m_documentViewer->pointCloudSurfaceMesh(id)) copyMesh(input.surface, *mesh);
            if (input.surface.triangles.empty()) {
                const auto& positions = selection.sourceCloud().positions;
                for (const auto row : selection.regionPoints(id)) {
                    const auto& [x, y, z] = positions[std::size_t(row)];
                    input.observations.push_back({x, y, z});
                }
            }
        }
    }

    showPreview();
    struct Result {
        reconstruction::PlanarSurfaceJunctionResult geometry;
        QString error;
    };
    const auto result = std::make_shared<Result>();
    m_worker = QThread::create([bundle, ids = std::move(ids), inputs = std::move(inputs), result]() mutable {
        const auto cancelled = [] { return QThread::currentThread()->isInterruptionRequested(); };
        try {
            if (!bundle.isEmpty()) inputs = loadAnalysisBundlePlanes(bundle, ids);
            std::vector<PlanarSurfaceInput> ready;
            for (auto& input : inputs) {
                if (cancelled()) return;
                restoreMesh(input);
                if (input.surface.triangles.empty()) {
                    result->error = tr("平面 #%1 缺少可拟合的有限网格。").arg(input.surface.id);
                    return;
                }
                ready.push_back(std::move(input.surface));
            }
            result->geometry = reconstruction::fitPlanarSurfaceJunctions(std::move(ready), cancelled);
        } catch (const std::exception& failure) {
            result->error = QString::fromUtf8(failure.what());
        }
    });
    m_worker->setParent(this);
    connect(m_worker, &QThread::finished, this, [this, result, bundle, revision] {
        std::exchange(m_worker, nullptr)->deleteLater();
        updateFitAction();
        if (bundle.isEmpty() && revision != m_surfaceRevision) return;
        if (!result->error.isEmpty()) m_status->setText(result->error);
        else showFit(result->geometry);
    });
    updateFitAction();
    m_status->setText(tr("正在拟合面片与交线…"));
    m_worker->start();
}

void PlanarJunctionPanel::clearFit()
{
    if (m_viewer) {
        auto* scene = renderer(m_viewer);
        for (const auto& face : m_surfaces) {
            scene->RemoveActor(face.surface);
            scene->RemoveActor(face.boundary);
        }
        for (const auto& seam : m_seams) scene->RemoveActor(seam.actor);
        m_viewer->requestRender();
    }
    m_surfaces.clear();
    m_seams.clear();
    const QSignalBlocker blocker(m_visibleFace);
    m_visibleFace->clear();
    m_visibleFace->addItem(tr("全部面片"), 0);
    m_visibleFace->setEnabled(false);
    m_report->clear();
}

void PlanarJunctionPanel::showFit(const reconstruction::PlanarSurfaceJunctionResult& result)
{
    clearFit();
    auto* scene = renderer(m_viewer);
    QStringList report;
    for (const auto& face : result.surfaces) {
        const auto rgb = PointCloudActor::regionColor(face.id);
        const std::array<double, 3> color{rgb[0] / 255., rgb[1] / 255., rgb[2] / 255.};
        auto fill = surfaceActor(face, color);

        vtkNew<vtkPoints> points;
        points->SetDataTypeToDouble();
        vtkNew<vtkCellArray> lines;
        for (const auto& boundary : face.boundaries) addLine(points, lines, boundary.vertices, true);
        auto outline = lineActor(points, lines, {.82, .93, 1.}, 1.8);
        scene->AddActor(fill);
        scene->AddActor(outline);
        m_surfaces.push_back({face.id, std::move(fill), std::move(outline)});
        m_visibleFace->addItem(planeName(face.id), face.id);
        report.push_back(tr("%1：%2").arg(planeName(face.id), face.boundaryAdjusted
            ? tr("边界已贴合交线") : tr("保留观测边界")));
    }

    std::size_t count = 0;
    for (const auto& junction : result.junctions) {
        QString summary;
        using Kind = reconstruction::PlanarJunctionKind;
        if (junction.kind == Kind::NearlyParallel) {
            summary = tr("平行或接近平行");
        } else if (junction.segments.empty()) {
            summary = tr("有限面域无交线");
        } else {
            vtkNew<vtkPoints> points;
            points->SetDataTypeToDouble();
            vtkNew<vtkCellArray> lines;
            for (const auto& [a, b] : junction.segments) addLine(points, lines, {a, b}, false);
            auto seam = lineActor(points, lines, {1., .63, .07}, 3.6);
            scene->AddActor(seam);
            m_seams.push_back({junction.first, junction.second, std::move(seam)});
            count += junction.segments.size();
            summary = tr("%1 段交线").arg(junction.segments.size());
        }
        report.push_back(tr("%1 × %2：%3").arg(planeName(junction.first), planeName(junction.second), summary));
    }
    m_report->setPlainText(report.join(QLatin1Char('\n')));
    m_visibleFace->setEnabled(!m_surfaces.empty());
    updateFitAction();
    updateVisibility();
    m_viewer->resetCamera();
    m_status->setText(tr("已拟合 %1 个面片，找到 %2 段交线。").arg(m_surfaces.size()).arg(count));
}

void PlanarJunctionPanel::updateVisibility()
{
    const int selected = m_visibleFace->currentData().toInt();
    for (const auto& face : m_surfaces) {
        const bool visible = !selected || face.id == selected;
        face.surface->SetVisibility(visible);
        face.boundary->SetVisibility(visible && m_boundaries->isChecked());
    }
    for (const auto& seam : m_seams)
        seam.actor->SetVisibility(m_intersections->isChecked() &&
            (!selected || seam.first == selected || seam.second == selected));
    if (m_viewer) m_viewer->requestRender();
}
