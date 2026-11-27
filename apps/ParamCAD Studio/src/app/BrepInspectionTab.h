#pragma once

#include <reconstruction/brep_closure/ClosedBrepReconstructor.h>

#include <QWidget>
#include <vtkSmartPointer.h>

class QCheckBox;
class QComboBox;
class QLabel;
class QSlider;
class VTKViewer;
class vtkActor;
class vtkPlane;

class BrepInspectionTab final : public QWidget
{
    Q_OBJECT
public:
    explicit BrepInspectionTab(QWidget* parent = nullptr);
    ~BrepInspectionTab() override;
    void setResult(const reconstruction::ClosedBrepResult& result, const QString& fileName,
                   const QString& error);

protected:
    void showEvent(QShowEvent* event) override;

private:
    QString cellLabel(reconstruction::BrepInspectionCell::Label label) const;
    void frameResult();
    void updateDisplay();
    void updateClipping();

    VTKViewer* m_viewer;
    QComboBox* m_layer;
    QComboBox* m_cell;
    QComboBox* m_holeStage;
    QCheckBox* m_showOutside;
    QCheckBox* m_showPoints;
    QComboBox* m_clipAxis;
    QSlider* m_clipPosition;
    QLabel* m_clipValue;
    QLabel* m_summary;
    QLabel* m_details;
    std::shared_ptr<const reconstruction::BrepInspection> m_inspection;
    TopoDS_Solid m_finalSolid;
    vtkSmartPointer<vtkPlane> m_clip;
    vtkSmartPointer<vtkActor> m_cuttingActor;
    vtkSmartPointer<vtkActor> m_finalActor;
    vtkSmartPointer<vtkActor> m_beforeHolesActor;
    vtkSmartPointer<vtkActor> m_holeToolsActor;
    vtkSmartPointer<vtkActor> m_pointsActor;
    std::vector<vtkSmartPointer<vtkActor>> m_cells;
    std::vector<vtkSmartPointer<vtkActor>> m_visible;
    bool m_framePending = false;
};
