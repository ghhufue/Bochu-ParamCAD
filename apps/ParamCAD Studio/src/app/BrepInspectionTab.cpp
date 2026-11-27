#include "app/BrepInspectionTab.h"
#include "rendering/OcctShapeTessellator.h"
#include "ui/viewport/VTKViewer.h"

#include <BRepBuilderAPI_Copy.hxx>
#include <BRep_Builder.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Compound.hxx>

#include <QCheckBox>
#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QSlider>
#include <QStandardItemModel>
#include <QVBoxLayout>

#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPlane.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>

#include <algorithm>

namespace {
using Label = reconstruction::BrepInspectionCell::Label;
enum Layer { CuttingFaces, Cells, FinalSolid, HoleCuts };
enum HoleStage { BeforeCuts, AfterCuts };

vtkSmartPointer<vtkActor> meshActor(vtkPolyData* mesh)
{
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputData(mesh);
    mapper->ScalarVisibilityOff();
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->PickableOff();
    return actor;
}

vtkSmartPointer<vtkActor> shapeActor(const TopoDS_Shape& shape, double deflection)
{
    try {
        // Tessellation writes to OCCT's triangulation cache. Use view-owned
        // topology so preview meshing leaves the published CAD intact.
        BRepBuilderAPI_Copy copy(shape, false, false);
        return meshActor(OcctShapeTessellator::tessellate(copy.Shape(), deflection).mesh);
    } catch (const Standard_Failure&) {
        vtkNew<vtkPolyData> empty;
        return meshActor(empty);
    }
}

vtkSmartPointer<vtkActor> observationActor(const std::vector<modeling::Vec3>& observations)
{
    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    vtkNew<vtkCellArray> vertices;
    for (const auto& point : observations) {
        const auto id = points->InsertNextPoint(point.x, point.y, point.z);
        vertices->InsertNextCell(1, &id);
    }
    vtkNew<vtkPolyData> mesh;
    mesh->SetPoints(points);
    mesh->SetVerts(vertices);
    auto actor = meshActor(mesh);
    actor->GetProperty()->SetPointSize(2.);
    actor->GetProperty()->SetColor(.86, .88, .90);
    actor->GetProperty()->LightingOff();
    return actor;
}

void styleCell(vtkActor* actor, Label label)
{
    auto* property = actor->GetProperty();
    property->EdgeVisibilityOn();
    switch (label) {
    case Label::Inside:
        property->SetColor(.24, .57, .89);
        property->SetEdgeColor(.12, .29, .45);
        property->SetOpacity(.85);
        break;
    case Label::Outside:
        property->SetColor(.67, .70, .74);
        property->SetEdgeColor(.61, .64, .68);
        property->SetOpacity(.26);
        break;
    case Label::ExteriorAnchor:
        property->SetColor(.92, .57, .28);
        property->SetRepresentationToWireframe();
        property->SetOpacity(.63);
        break;
    case Label::Unclassified:
        property->SetColor(.91, .77, .36);
        property->SetEdgeColor(.64, .48, .22);
        property->SetOpacity(.42);
        break;
    }
}

bool isOutside(Label label)
{
    return label == Label::Outside || label == Label::ExteriorAnchor;
}
} // namespace

BrepInspectionTab::BrepInspectionTab(QWidget* parent)
    : QWidget(parent),
      m_viewer(new VTKViewer(this, nullptr, false)),
      m_layer(new QComboBox(this)),
      m_cell(new QComboBox(this)),
      m_holeStage(new QComboBox(this)),
      m_showOutside(new QCheckBox(tr("显示外部 Cell"), this)),
      m_showPoints(new QCheckBox(tr("工作点云"), this)),
      m_clipAxis(new QComboBox(this)),
      m_clipPosition(new QSlider(Qt::Horizontal, this)),
      m_clipValue(new QLabel(this)),
      m_summary(new QLabel(this)),
      m_details(new QLabel(this)),
      m_clip(vtkSmartPointer<vtkPlane>::New())
{
    setObjectName(QStringLiteral("brepInspectionTab"));
    m_viewer->setDatumPlanesVisible(false);
    m_layer->setObjectName(QStringLiteral("brepInspectionLayer"));
    m_cell->setObjectName(QStringLiteral("brepInspectionCell"));
    m_holeStage->setObjectName(QStringLiteral("brepInspectionHoleStage"));
    m_showOutside->setObjectName(QStringLiteral("brepInspectionShowOutside"));
    m_clipAxis->setObjectName(QStringLiteral("brepInspectionClipAxis"));
    m_clipPosition->setObjectName(QStringLiteral("brepInspectionClipPosition"));
    m_summary->setObjectName(QStringLiteral("brepInspectionSummary"));
    m_details->setObjectName(QStringLiteral("brepInspectionDetails"));
    m_layer->addItems({tr("切分面"), tr("Cell 内外"), tr("最终实体"), tr("孔切除")});
    m_holeStage->addItems({tr("切孔前与刀具"), tr("切孔后")});
    m_holeStage->hide();
    m_clipAxis->addItems({tr("关闭剖切"), QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z")});
    m_clipPosition->setRange(0, 1000);
    m_clipPosition->setValue(500);
    m_clipPosition->setEnabled(false);
    m_clipPosition->setAccessibleName(tr("剖切位置"));
    m_layer->setAccessibleName(tr("检查阶段"));
    m_cell->setAccessibleName(tr("选中的 Cell"));
    m_holeStage->setAccessibleName(tr("孔切除阶段"));
    m_clipAxis->setAccessibleName(tr("剖切轴"));
    m_summary->setTextFormat(Qt::PlainText);
    m_details->setTextFormat(Qt::PlainText);
    m_summary->setWordWrap(true);
    m_details->setWordWrap(true);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(6, 6, 6, 6);
    layout->addWidget(m_summary);
    auto* controls = new QHBoxLayout;
    controls->addWidget(m_layer);
    controls->addWidget(m_cell);
    controls->addWidget(m_holeStage);
    controls->addWidget(m_showOutside);
    controls->addWidget(m_showPoints);
    controls->addStretch();
    controls->addWidget(m_clipAxis);
    controls->addWidget(m_clipPosition, 1);
    controls->addWidget(m_clipValue);
    layout->addLayout(controls);
    layout->addWidget(m_details);
    layout->addWidget(m_viewer, 1);

    connect(m_layer, &QComboBox::currentIndexChanged, this, &BrepInspectionTab::updateDisplay);
    connect(m_cell, &QComboBox::currentIndexChanged, this, &BrepInspectionTab::updateDisplay);
    connect(m_holeStage, &QComboBox::currentIndexChanged, this, &BrepInspectionTab::updateDisplay);
    connect(m_showOutside, &QCheckBox::toggled, this, &BrepInspectionTab::updateDisplay);
    connect(m_showPoints, &QCheckBox::toggled, this, &BrepInspectionTab::updateDisplay);
    connect(m_clipAxis, &QComboBox::currentIndexChanged, this, &BrepInspectionTab::updateClipping);
    connect(m_clipPosition, &QSlider::valueChanged, this, &BrepInspectionTab::updateClipping);
}

BrepInspectionTab::~BrepInspectionTab() = default;

void BrepInspectionTab::showEvent(QShowEvent* event)
{
    QWidget::showEvent(event);
    if (m_framePending) frameResult();
}

QString BrepInspectionTab::cellLabel(Label label) const
{
    switch (label) {
    case Label::Inside: return tr("内部");
    case Label::Outside: return tr("外部");
    case Label::ExteriorAnchor: return tr("包围盒判外");
    case Label::Unclassified: return tr("未分类");
    }
    return {};
}

void BrepInspectionTab::setResult(const reconstruction::ClosedBrepResult& result, const QString& fileName,
                                  const QString& error)
{
    m_inspection = result.inspection;
    m_finalSolid = result.solid;
    m_cuttingActor = nullptr;
    m_finalActor = nullptr;
    m_beforeHolesActor = nullptr;
    m_holeToolsActor = nullptr;
    m_pointsActor = nullptr;
    m_cells.clear();
    m_visible.clear();
    m_viewer->setInspectionActors({});
    const auto& attempt = m_inspection->attempt;
    m_cells.resize(attempt.cells.size());

    const QSignalBlocker layerSignals(m_layer), cellSignals(m_cell), holeStageSignals(m_holeStage), clipSignals(m_clipAxis),
        outsideSignals(m_showOutside);
    auto* layers = qobject_cast<QStandardItemModel*>(m_layer->model());
    layers->item(CuttingFaces)->setEnabled(!attempt.cuttingFaces.empty());
    layers->item(Cells)->setEnabled(!attempt.cells.empty());
    layers->item(FinalSolid)->setEnabled(!m_finalSolid.IsNull());
    layers->item(HoleCuts)->setEnabled(!attempt.holeTools.empty());
    auto* holeStages = qobject_cast<QStandardItemModel*>(m_holeStage->model());
    holeStages->item(AfterCuts)->setEnabled(!m_finalSolid.IsNull());
    m_holeStage->setCurrentIndex(BeforeCuts);
    m_cell->clear();
    m_cell->addItem(tr("全部 Cell"));
    for (std::size_t i = 0; i < attempt.cells.size(); ++i)
        m_cell->addItem(tr("Cell %1 · %2").arg(i + 1).arg(cellLabel(attempt.cells[i].label)));
    if (!m_finalSolid.IsNull()) m_layer->setCurrentIndex(FinalSolid);
    else if (!attempt.cells.empty()) m_layer->setCurrentIndex(Cells);
    else if (!attempt.cuttingFaces.empty() || attempt.holeTools.empty()) m_layer->setCurrentIndex(CuttingFaces);
    else m_layer->setCurrentIndex(HoleCuts);
    m_clipAxis->setCurrentIndex(0);
    m_showOutside->setChecked(std::none_of(attempt.cells.begin(), attempt.cells.end(),
        [](const auto& cell) { return !isOutside(cell.label); }));
    auto summary = fileName;
    if (attempt.number)
        summary += tr(" · 第 %1 次切分 · 容差 %2 mm").arg(attempt.number).arg(attempt.fuzzyToleranceMm, 0, 'g', 5);
    if (!attempt.error.empty()) summary += QStringLiteral("\n") + QString::fromStdString(attempt.error).trimmed();
    if (!error.isEmpty() && error.trimmed() != QString::fromStdString(attempt.error).trimmed())
        summary += QStringLiteral("\n") + error.trimmed();
    m_summary->setText(summary);
    m_framePending = true;
    if (isVisible()) frameResult();
}

void BrepInspectionTab::frameResult()
{
    m_framePending = false;
    updateDisplay();
    m_viewer->resetCamera();
}

void BrepInspectionTab::updateDisplay()
{
    if (!m_inspection) return;
    const auto& attempt = m_inspection->attempt;
    const auto& bounds = m_inspection->boundsMm;
    const auto span = std::max({bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]});
    const auto deflection = std::max(1e-5, span * .001);
    const auto layer = m_layer->currentIndex();
    const int cellIndex = m_cell->currentIndex() - 1;
    const bool showHoleCuts = layer == HoleCuts;
    const bool showAfterCuts = showHoleCuts && m_holeStage->currentIndex() == AfterCuts;
    m_cell->setVisible(!showHoleCuts);
    m_showOutside->setVisible(!showHoleCuts);
    m_holeStage->setVisible(showHoleCuts);
    m_cell->setEnabled(layer == Cells);
    m_showOutside->setEnabled(layer == Cells && cellIndex < 0);
    m_visible.clear();
    if (layer == CuttingFaces && !attempt.cuttingFaces.empty()) {
        if (!m_cuttingActor) {
            BRep_Builder builder;
            TopoDS_Compound faces;
            builder.MakeCompound(faces);
            for (const auto& face : attempt.cuttingFaces) builder.Add(faces, face);
            m_cuttingActor = shapeActor(faces, deflection);
            m_cuttingActor->GetProperty()->SetColor(.32, .72, .78);
            m_cuttingActor->GetProperty()->SetOpacity(.38);
            m_cuttingActor->GetProperty()->EdgeVisibilityOn();
            m_cuttingActor->GetProperty()->SetEdgeColor(.18, .47, .54);
        }
        m_visible.push_back(m_cuttingActor);
        m_details->setText(tr("%1 张切分面").arg(attempt.cuttingFaces.size()));
    } else if (layer == Cells) {
        std::size_t inside = 0, outside = 0, anchors = 0, unknown = 0;
        for (std::size_t i = 0; i < attempt.cells.size(); ++i) {
            const auto& cell = attempt.cells[i];
            inside += cell.label == Label::Inside;
            outside += cell.label == Label::Outside;
            anchors += cell.label == Label::ExteriorAnchor;
            unknown += cell.label == Label::Unclassified;
            if (cellIndex >= 0 ? i != static_cast<std::size_t>(cellIndex) :
                                 isOutside(cell.label) && !m_showOutside->isChecked()) continue;
            if (!m_cells[i]) {
                m_cells[i] = shapeActor(cell.solid, deflection);
                styleCell(m_cells[i], cell.label);
            }
            m_visible.push_back(m_cells[i]);
        }
        if (cellIndex >= 0) {
            const auto& selected = attempt.cells[static_cast<std::size_t>(cellIndex)];
            m_details->setText(tr("Cell %1 · %2 · %3 mm³").arg(cellIndex + 1)
                .arg(cellLabel(selected.label)).arg(selected.volumeMm3, 0, 'g', 7));
        } else {
            m_details->setText(tr("蓝色内部 %1 · 灰色外部 %2 · 橙色包围盒判外 %3 · 黄色未分类 %4")
                .arg(inside).arg(outside).arg(anchors).arg(unknown));
        }
        if (!attempt.holeTools.empty()) {
            m_details->setText(m_details->text() + QStringLiteral("\n") +
                tr("合并 Cell 后应用 %1 个孔刀具，可在「孔切除」查看。").arg(attempt.holeTools.size()));
        }
    } else if ((layer == FinalSolid || showAfterCuts) && !m_finalSolid.IsNull()) {
        if (!m_finalActor) {
            m_finalActor = shapeActor(m_finalSolid, deflection);
            m_finalActor->GetProperty()->SetColor(.81, .83, .86);
        }
        m_visible.push_back(m_finalActor);
        m_details->setText(attempt.holeTools.empty()
            ? tr("通过质检的闭合实体")
            : tr("通过质检的闭合实体 · 已应用 %1 个孔刀具").arg(attempt.holeTools.size()));
    } else if (showHoleCuts && !attempt.holeTools.empty()) {
        if (!attempt.solidBeforeHoleCuts.IsNull()) {
            if (!m_beforeHolesActor) {
                m_beforeHolesActor = shapeActor(attempt.solidBeforeHoleCuts, deflection);
                m_beforeHolesActor->GetProperty()->SetColor(.76, .81, .87);
                m_beforeHolesActor->GetProperty()->SetOpacity(.28);
            }
            m_visible.push_back(m_beforeHolesActor);
        }
        if (!m_holeToolsActor) {
            BRep_Builder builder;
            TopoDS_Compound tools;
            builder.MakeCompound(tools);
            for (const auto& tool : attempt.holeTools) builder.Add(tools, tool);
            m_holeToolsActor = shapeActor(tools, deflection);
            m_holeToolsActor->GetProperty()->SetColor(.82, .38, .83);
        }
        m_visible.push_back(m_holeToolsActor);
        m_details->setText(tr("半透明灰色为切孔前基底 · 紫色为 %1 个孔刀具").arg(attempt.holeTools.size()));
        if (attempt.solidBeforeHoleCuts.IsNull())
            m_details->setText(tr("紫色为 %1 个孔刀具，本次切分尚未得到基底。").arg(attempt.holeTools.size()));
    } else {
        m_details->setText(tr("本次重建没有生成切分面"));
    }
    if (m_showPoints->isChecked()) {
        if (!m_pointsActor) m_pointsActor = observationActor(m_inspection->observationsMm);
        m_visible.push_back(m_pointsActor);
    }
    m_viewer->setInspectionActors(m_visible);
    updateClipping();
}

void BrepInspectionTab::updateClipping()
{
    if (!m_inspection) return;
    const auto axis = m_clipAxis->currentIndex() - 1;
    m_clipPosition->setEnabled(axis >= 0);
    if (axis >= 0) {
        const auto& bounds = m_inspection->boundsMm;
        const auto fraction = m_clipPosition->value() / 1000.;
        const auto value = bounds[2 * axis] + fraction * (bounds[2 * axis + 1] - bounds[2 * axis]);
        double origin[3]{}, normal[3]{};
        origin[axis] = value;
        normal[axis] = 1.;
        m_clip->SetOrigin(origin);
        m_clip->SetNormal(normal);
        m_clipValue->setText(tr("%1 mm").arg(value, 0, 'g', 6));
    } else {
        m_clipValue->clear();
    }
    for (const auto& actor : m_visible) {
        auto* mapper = actor->GetMapper();
        mapper->RemoveAllClippingPlanes();
        if (axis >= 0) mapper->AddClippingPlane(m_clip);
    }
    m_viewer->requestRender();
}
