#include "app/PointCloudWorkspaceController.h"

#include "app/ModelingController.h"
#include "core/PointCloudDocument.h"
#include "rendering/PointCloudContour.h"
#include "ui/panels/PointCloudRelationsPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <contour_fitting/ContourParameterization.h>
#include <modeling/SketchValidation.h>

#include <QThread>
#include <exception>

namespace {
using contour_fitting::FittedRing;

void appendRing(modeling::SketchFeatureParams& sketch, const FittedRing& ring)
{
    const auto append = [&](modeling::SketchGeometry geometry) {
        const auto id = static_cast<modeling::SketchEntityId>(sketch.entities.size()+1);
        sketch.entities.push_back({id, std::move(geometry)});
    };
    if (ring.circle) {
        const auto& [center, radius] = *ring.circle;
        append(modeling::Circle2D{center.x, center.y, radius});
        return;
    }
    for (std::size_t i = 0; i < ring.polyline.size(); ++i) {
        const auto a = ring.polyline[i], b = ring.polyline[(i+1)%ring.polyline.size()];
        if (!ring.arcs.empty() && ring.arcs[i]) {
            const auto middle = ring.arcs[i]->pointAt(.5);
            append(modeling::Arc2D{{a.x,a.y}, {middle.x,middle.y}, {b.x,b.y}});
        } else {
            append(modeling::Line2D{a.x, a.y, b.x, b.y});
        }
    }
}

QString contourFailure(contour_fitting::FitFailure failure)
{
    using contour_fitting::FitFailure;
    switch (failure) {
    case FitFailure::InvalidBoundary:
        return workspaceTr("边界无法形成有效闭环。请检查相交、接触或缺失的边界。");
    case FitFailure::EdgeBudget:
        return workspaceTr("当前容差下超出草图边数上限。请提高边数上限或调整容差。");
    case FitFailure::Cancelled:
        return workspaceTr("轮廓拟合已取消。");
    case FitFailure::TouchingHole:
        return workspaceTr("内孔与外环在顶点处接触，当前拟合容差不足以分开它们。");
    case FitFailure::None:
        return {};
    }
    return {};
}
} // namespace

struct PointCloudWorkspaceController::ContourPreview {
    contour_fitting::ParameterizationResult fit;
    std::vector<modeling::SketchFeatureParams> sketches;
    QString error;
};

void PointCloudWorkspaceController::inspectCloudBoundary(int regionId)
{
    const auto* region = m_document->selection().region(regionId);
    if (!region) return;
    auto input = std::make_shared<PointCloudContourInput>();
    const auto counts = m_ui.viewer->inspectPointCloudBoundary(regionId, input.get());
    if (!counts) {
        m_ui.relationsPanel->setStatus(workspaceTr("当前平面无法进入边界检查。"));
        return;
    }
    const auto& unit = m_document->sourceUnit();
    m_contourUnitScale = unit == "mm" ? std::optional<double>(1.)
        : unit == "m" ? std::optional<double>(1000.) : std::nullopt;
    m_contourInput = input;
    m_ui.relationsPanel->showBoundaryInspection(region->name, counts->points, counts->triangles,
                                                counts->outerRings, counts->innerRings);
    m_ui.relationsPanel->configureContourFit(input->spacing*m_contourUnitScale.value_or(1.),
                                             m_contourUnitScale.has_value());
    if (m_contourFitThread) m_ui.relationsPanel->setContourFitState(true, false);
}

void PointCloudWorkspaceController::invalidateContourPreview()
{
    ++m_contourFitRevision;
    m_contourPreview.reset();
    if (m_contourFitThread) m_contourFitThread->requestInterruption();
    m_ui.viewer->clearBoundaryContourPreview();
    m_ui.relationsPanel->setContourFitState(m_contourFitThread != nullptr, false);
}

void PointCloudWorkspaceController::closeContourInspection()
{
    m_contourInput.reset();
    invalidateContourPreview();
    m_ui.relationsPanel->clearBoundaryInspection();
}

void PointCloudWorkspaceController::startContourFit(double tolerance, int maximumEdges, bool recognizeCircles,
                                                    bool engineering, double maximumLineTolerance, double circleTolerance)
{
    if (!m_contourInput || m_contourInput->contours.empty() || m_contourFitThread) return;
    invalidateContourPreview();
    const auto revision = m_contourFitRevision;
    auto input = m_contourInput;
    const double scale = m_contourUnitScale.value_or(1.);
    contour_fitting::ParameterizationOptions options{tolerance,
        static_cast<std::size_t>(maximumEdges), recognizeCircles};
    if (engineering) options.engineering = contour_fitting::ParameterizationOptions::Engineering{
        input->spacing*scale, maximumLineTolerance, circleTolerance};
    auto result = std::make_shared<ContourPreview>();
    auto* thread = QThread::create([input, scale, options, result] {
        const auto cancelled = [] { return QThread::currentThread()->isInterruptionRequested(); };
        try {
            auto contours = input->contours;
            if (scale != 1.) {
                const auto convert = [scale](auto& ring) {
                    for (auto& point : ring) { point.x *= scale; point.y *= scale; }
                };
                for (auto& contour : contours) {
                    convert(contour.outer);
                    for (auto& hole : contour.holes) convert(hole);
                }
            }
            result->fit = contour_fitting::parameterizeContours(contours, options, cancelled);
            result->error = contourFailure(result->fit.failure);
            if (!result->error.isEmpty()) return;
            auto plane = input->plane;
            plane.origin.x *= scale;
            plane.origin.y *= scale;
            plane.origin.z *= scale;
            for (const auto& contour : result->fit.contours) {
                if (cancelled()) return;
                modeling::SketchFeatureParams sketch;
                sketch.plane = plane;
                sketch.keepCoincidentEndpoints = true;
                appendRing(sketch, contour.outer);
                for (const auto& hole : contour.holes) appendRing(sketch, hole);
                std::string error;
                if (!modeling::validateSketch(sketch, error)) {
                    result->error = workspaceTr("草图构面校验失败：%1").arg(QString::fromStdString(error));
                    return;
                }
                result->sketches.push_back(std::move(sketch));
            }
        } catch (const std::exception& error) {
            result->error = QString::fromUtf8(error.what());
        }
    });
    m_contourFitThread = thread;
    thread->setParent(this);
    connect(thread, &QThread::finished, this, [this, input, scale, revision, result, engineering] {
        m_contourFitThread = nullptr;
        if (revision != m_contourFitRevision) {
            if (m_contourInput) m_ui.relationsPanel->setContourFitState(false, false);
            return;
        }
        if (!result->error.isEmpty() || result->sketches.empty()) {
            m_ui.relationsPanel->setContourFitState(false, false, result->error);
            return;
        }
        m_contourPreview = result;
        m_ui.viewer->setBoundaryContourPreview(*input, result->fit, 1./scale);
        const auto unit = m_contourUnitScale ? QStringLiteral("mm") : workspaceTr("源单位");
        auto message = workspaceTr("青色为拟合轮廓。%1 张草图，%2 条边\n最大偏差 ≤ %3 %5，P95 偏差 %4 %5")
                .arg(result->sketches.size()).arg(result->fit.edgeCount)
                .arg(result->fit.maximumDeviation, 0, 'g', 5).arg(result->fit.p95Deviation, 0, 'g', 5).arg(unit);
        if (result->fit.separatedHoles)
            message += QStringLiteral("\n") + workspaceTr("已在拟合容差内分开 %1 个接触外环的内孔。")
                .arg(result->fit.separatedHoles);
        if (engineering)
            message += QStringLiteral("\n") + workspaceTr("直线 %1 条，整圆 %2 个，圆弧 %3 段。橙色点表示较大残差。")
                .arg(result->fit.edgeCount-result->fit.circleCount-result->fit.arcCount)
                .arg(result->fit.circleCount).arg(result->fit.arcCount);
        m_ui.relationsPanel->setContourFitState(false, true, message);
    });
    connect(thread, &QThread::finished, thread, &QObject::deleteLater);
    m_ui.relationsPanel->setContourFitState(true, false, workspaceTr("正在后台拟合轮廓…"));
    thread->start();
}

void PointCloudWorkspaceController::createContourSketch()
{
    const auto preview = m_contourPreview;
    if (!preview) return;
    if (!m_contourUnitScale) {
        const auto unit = confirmCloudUnit();
        if (!unit || m_contourPreview != preview) return;
        if (*unit != reconstruction::LengthUnit::Millimeter) {
            m_ui.viewer->closeBoundaryInspection();
            m_ui.relationsPanel->setStatus(workspaceTr("点云尺度已变化。请重新载入点云后拟合。"));
            return;
        }
    }
    modeling::FeatureId firstId = modeling::kInvalidFeatureId;
    if (!m_modelingController->addContourSketches(preview->sketches, firstId)) {
        m_ui.relationsPanel->setContourFitState(false, true, m_modelingController->lastError());
        return;
    }
    m_ui.viewer->closeBoundaryInspection();
    const auto message = workspaceTr("已生成 %1 张轮廓草图。").arg(preview->sketches.size());
    emit logInfo(message);
    emit statusMessage(message, 5000);
    emit contourSketchCreated(firstId);
}
