#include "ui/panels/SketchEntityPanel.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QVBoxLayout>

namespace {

QString kindLabel(modeling::SketchEntityType kind)
{
    switch (kind) {
    case modeling::SketchEntityType::Point:
        return QStringLiteral("Point");
    case modeling::SketchEntityType::Line:
        return QStringLiteral("Line");
    case modeling::SketchEntityType::Rectangle:
        return QStringLiteral("Rectangle");
    case modeling::SketchEntityType::Circle:
        return QStringLiteral("Circle");
    case modeling::SketchEntityType::Arc:
        return QStringLiteral("Arc");
    }
    return QStringLiteral("Unknown");
}

}  // namespace

SketchEntityPanel::SketchEntityPanel(QWidget* parent)
    : QWidget(parent)
    , m_stack(new QStackedWidget(this))
{
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(8, 8, 8, 8);
    outer->setSpacing(8);

    m_title = new QLabel(tr("未选择图元"), this);
    m_title->setWordWrap(true);
    outer->addWidget(m_title);

    // One page per entity kind: a point has no width, a circle no corner, and
    // showing the fields of every kind at once would only invite mis-edits.
    auto* emptyPage = new QWidget(m_stack);
    auto* emptyLayout = new QVBoxLayout(emptyPage);
    emptyLayout->addWidget(new QLabel(tr("请在草图列表中选择一个图元。"),
                                      emptyPage));
    emptyLayout->addStretch();

    auto* pointPage = new QWidget(m_stack);
    auto* pointForm = new QFormLayout(pointPage);
    m_pointX = addField(pointPage, pointForm, tr("X"), 0.0, -1000.0,
                        QStringLiteral("sketchPointX"));
    m_pointY = addField(pointPage, pointForm, tr("Y"), 0.0, -1000.0,
                        QStringLiteral("sketchPointY"));

    auto* linePage = new QWidget(m_stack);
    auto* lineForm = new QFormLayout(linePage);
    m_lineX1 = addField(linePage, lineForm, tr("起点 X"), 0.0, -1000.0,
                        QStringLiteral("sketchLineX1"));
    m_lineY1 = addField(linePage, lineForm, tr("起点 Y"), 0.0, -1000.0,
                        QStringLiteral("sketchLineY1"));
    m_lineX2 = addField(linePage, lineForm, tr("终点 X"), 10.0, -1000.0,
                        QStringLiteral("sketchLineX2"));
    m_lineY2 = addField(linePage, lineForm, tr("终点 Y"), 0.0, -1000.0,
                        QStringLiteral("sketchLineY2"));

    auto* rectPage = new QWidget(m_stack);
    auto* rectForm = new QFormLayout(rectPage);
    m_rectX = addField(rectPage, rectForm, tr("X"), 0.0, -1000.0,
                       QStringLiteral("sketchRectX"));
    m_rectY = addField(rectPage, rectForm, tr("Y"), 0.0, -1000.0,
                       QStringLiteral("sketchRectY"));
    m_rectWidth = addField(rectPage, rectForm, tr("宽度"), 20.0, 0.001,
                           QStringLiteral("sketchRectWidth"));
    m_rectHeight = addField(rectPage, rectForm, tr("高度"), 20.0, 0.001,
                            QStringLiteral("sketchRectHeight"));

    auto* circlePage = new QWidget(m_stack);
    auto* circleForm = new QFormLayout(circlePage);
    m_circleX = addField(circlePage, circleForm, tr("中心 X"), 0.0, -1000.0,
                         QStringLiteral("sketchCircleX"));
    m_circleY = addField(circlePage, circleForm, tr("中心 Y"), 0.0, -1000.0,
                         QStringLiteral("sketchCircleY"));
    m_circleRadius = addField(circlePage, circleForm, tr("半径"), 5.0, 0.001,
                              QStringLiteral("sketchCircleRadius"));
    auto* arcPage = new QWidget(m_stack);
    auto* arcForm = new QFormLayout(arcPage);
    m_arcRadius = addField(arcPage, arcForm, tr("半径"), 5.0, 0.001, QStringLiteral("sketchArcRadius"));
    m_arcRadius->setKeyboardTracking(false);

    m_stack->addWidget(emptyPage);
    m_stack->addWidget(pointPage);
    m_stack->addWidget(linePage);
    m_stack->addWidget(rectPage);
    m_stack->addWidget(circlePage);
    m_stack->addWidget(arcPage);
    outer->addWidget(m_stack);
    outer->addStretch();

    setEntity(nullptr);
}

QDoubleSpinBox* SketchEntityPanel::addField(QWidget* page, QFormLayout* layout,
                                           const QString& label, double value,
                                           double minimum, const QString& objectName)
{
    auto* field = new QDoubleSpinBox(page);
    field->setObjectName(objectName);
    field->setRange(minimum, 1000.0);
    field->setDecimals(3);
    field->setSingleStep(1.0);
    field->setValue(value);
    layout->addRow(label, field);
    connectField(field);
    return field;
}

void SketchEntityPanel::connectField(QDoubleSpinBox* field)
{
    connect(field, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (m_currentId != modeling::kInvalidSketchEntityId) {
            emit entityEdited();
        }
    });
}

void SketchEntityPanel::setEntity(const modeling::SketchEntity* entity, bool fittedContour)
{
    if (entity == nullptr) {
        m_currentId = modeling::kInvalidSketchEntityId;
        m_title->setText(tr("未选择图元"));
        m_stack->setCurrentIndex(0);
        return;
    }

    m_currentId = entity->id;
    m_kind = modeling::sketchEntityType(*entity);
    m_title->setText(QStringLiteral("%1 #%2").arg(kindLabel(m_kind)).arg(entity->id));

    // Filling the fields must not look like a user edit, so every field is
    // blocked while it is being seeded.
    const auto seed = [fittedContour](QDoubleSpinBox* field, double value, bool positive = false) {
        const QSignalBlocker blocker(field);
        field->setDecimals(fittedContour ? 9 : 3);
        const double limit = fittedContour ? 1e9 : 1000.;
        const double minimum = fittedContour ? 1e-9 : .001;
        field->setRange(positive ? minimum : -limit, limit);
        field->setValue(value);
    };

    switch (m_kind) {
    case modeling::SketchEntityType::Point: {
        const auto& point = std::get<modeling::Point2D>(entity->geometry);
        seed(m_pointX, point.x);
        seed(m_pointY, point.y);
        m_stack->setCurrentIndex(1);
        break;
    }
    case modeling::SketchEntityType::Line: {
        const auto& line = std::get<modeling::Line2D>(entity->geometry);
        seed(m_lineX1, line.x1);
        seed(m_lineY1, line.y1);
        seed(m_lineX2, line.x2);
        seed(m_lineY2, line.y2);
        m_stack->setCurrentIndex(2);
        break;
    }
    case modeling::SketchEntityType::Rectangle: {
        const auto& rectangle = std::get<modeling::Rectangle2D>(entity->geometry);
        seed(m_rectX, rectangle.x);
        seed(m_rectY, rectangle.y);
        seed(m_rectWidth, rectangle.width, true);
        seed(m_rectHeight, rectangle.height, true);
        m_stack->setCurrentIndex(3);
        break;
    }
    case modeling::SketchEntityType::Circle: {
        const auto& circle = std::get<modeling::Circle2D>(entity->geometry);
        seed(m_circleX, circle.x);
        seed(m_circleY, circle.y);
        seed(m_circleRadius, circle.radius, true);
        m_stack->setCurrentIndex(4);
        break;
    }
    case modeling::SketchEntityType::Arc: {
        m_arc = std::get<modeling::Arc2D>(entity->geometry);
        const auto parameters = modeling::arcParameters(m_arc);
        if (parameters) seed(m_arcRadius, parameters->radius, true);
        m_arcRadius->setEnabled(parameters && std::abs(parameters->sweep) < 3.13);
        m_stack->setCurrentIndex(5);
        break;
    }
    }
}

modeling::SketchGeometry SketchEntityPanel::geometry() const
{
    switch (m_kind) {
    case modeling::SketchEntityType::Point:
        return modeling::Point2D{m_pointX->value(), m_pointY->value()};
    case modeling::SketchEntityType::Line:
        return modeling::Line2D{m_lineX1->value(), m_lineY1->value(),
                                m_lineX2->value(), m_lineY2->value()};
    case modeling::SketchEntityType::Rectangle:
        return modeling::Rectangle2D{m_rectX->value(), m_rectY->value(),
                                     m_rectWidth->value(), m_rectHeight->value()};
    case modeling::SketchEntityType::Circle:
        return modeling::Circle2D{m_circleX->value(), m_circleY->value(),
                                  m_circleRadius->value()};
    case modeling::SketchEntityType::Arc: {
        const auto parameters = modeling::arcParameters(m_arc);
        if (!parameters || !m_arcRadius->isEnabled()) return m_arc;
        const double scale = m_arcRadius->value() / parameters->radius;
        const double angle = parameters->startAngle + parameters->sweep*.5;
        const double offset = parameters->radius / std::cos(parameters->sweep*.5);
        const modeling::Point2D corner{parameters->center.x + offset*std::cos(angle),
                                        parameters->center.y + offset*std::sin(angle)};
        // Scale about the intersection of the tangents. Connected line
        // endpoints follow their moved tangent points through the document.
        const auto move = [&](modeling::Point2D p) {
            return modeling::Point2D{corner.x + scale*(p.x-corner.x), corner.y + scale*(p.y-corner.y)};
        };
        return modeling::Arc2D{move(m_arc.start), move(m_arc.middle), move(m_arc.end)};
    }
    }
    return modeling::Point2D{};
}
