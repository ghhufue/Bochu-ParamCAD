#pragma once

#include <QWidget>

class FeatureDetailsPanel;
class ModelingController;
class ModelingPanel;
class VTKViewer;

class ParametricModelingTab final : public QWidget
{
    Q_OBJECT

public:
    ParametricModelingTab(ModelingController* model, VTKViewer* documentViewer, QWidget* parent);

private:
    void syncDocument();

    ModelingController* const m_model;
    VTKViewer* const m_documentViewer;
    VTKViewer* const m_viewer;
    ModelingPanel* const m_features;
    FeatureDetailsPanel* const m_details;
};
