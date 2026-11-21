#include "app/ParametricModelingTab.h"

#include "app/FeatureEditingController.h"
#include "app/ModelingController.h"
#include "ui/panels/FeatureDetailsPanel.h"
#include "ui/panels/ModelingPanel.h"
#include "ui/viewport/VTKViewer.h"

#include <QAction>
#include <QSplitter>
#include <QToolBar>
#include <QVBoxLayout>

ParametricModelingTab::ParametricModelingTab(ModelingController* model, VTKViewer* documentViewer,
                                           QWidget* parent)
    : QWidget(parent),
      m_model(model),
      m_documentViewer(documentViewer),
      m_viewer(new VTKViewer(this, nullptr, false)),
      m_features(new ModelingPanel(this, ModelingPanel::Commands::None)),
      m_details(new FeatureDetailsPanel(this))
{
    setObjectName(QStringLiteral("parametricModelingTab"));

    m_viewer->setObjectName(QStringLiteral("modelingViewport"));

    auto* splitter = new QSplitter(Qt::Horizontal, this);
    splitter->addWidget(m_features);
    splitter->addWidget(m_viewer);
    splitter->addWidget(m_details);
    splitter->setChildrenCollapsible(false);
    splitter->setSizes({215, 650, 315});
    splitter->setStretchFactor(1, 1);

    auto* toolbar = new QToolBar(workspaceTr("参数化建模"), this);
    toolbar->setMovable(false);
    auto* fit = toolbar->addAction(workspaceTr("重置相机"));
    connect(fit, &QAction::triggered, m_viewer, &VTKViewer::resetCamera);
    auto* undo = toolbar->addAction(workspaceTr("撤销 CAD 编辑"));
    undo->setObjectName(QStringLiteral("modelingUndoAction"));
    auto* redo = toolbar->addAction(workspaceTr("重做 CAD 编辑"));
    redo->setObjectName(QStringLiteral("modelingRedoAction"));
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(toolbar);
    layout->addWidget(splitter, 1);

    connect(m_features, &ModelingPanel::featureSelected, this, [this](modeling::FeatureId id) {
        m_details->setFeature(m_model->features(), id);
    });
    connect(m_details, &FeatureDetailsPanel::featureRequested, m_features, &ModelingPanel::selectFeature);
    connect(m_documentViewer, &VTKViewer::bodyShapeChanged, this, &ParametricModelingTab::syncDocument);
    connect(m_model, &ModelingController::modelRebuilt, this, [this] {
        // With only sketches in the history, the document viewer may not
        // publish a body change. The feature list still needs an update.
        if (m_model->bodyShape().IsNull()) syncDocument();
    });

    syncDocument();
    auto* editing = new FeatureEditingController(model,
        {m_viewer, m_features, m_details, undo, redo, m_documentViewer}, this, this);
    editing->connectUi();
}

void ParametricModelingTab::syncDocument()
{
    const auto& body = m_model->bodyShape();
    const auto& documentBody = m_documentViewer->displayedBody();
    const bool hasBody = !body.IsNull() && body.IsSame(documentBody);
    const bool isParametricDocument = hasBody || (body.IsNull() && documentBody.IsNull());
    const auto& displayed = m_viewer->displayedBody();
    if (hasBody && (displayed.IsNull() || !body.IsSame(displayed))) {
        const bool needsFit = displayed.IsNull();
        m_viewer->setBodyShape(body);
        if (needsFit) m_viewer->resetCamera();
        else m_viewer->requestRender();
    } else if (!hasBody && !displayed.IsNull()) {
        m_viewer->clearBody();
        m_viewer->requestRender();
    }

    if (isParametricDocument) {
        const auto& features = m_model->features();
        m_features->setFeatures(features);
        if (!features.empty() && m_features->selectedFeatureId() == modeling::kInvalidFeatureId)
            m_features->selectFeature(features.back().id);
        m_details->setFeature(features, m_features->selectedFeatureId());
    } else {
        m_features->setFeatures({});
        m_details->setFeature({}, modeling::kInvalidFeatureId);
    }
    m_features->setStatusText(hasBody ? QString{} : workspaceTr("当前项目没有可建模的实体。"));
}
