#pragma once
#include "core/PointCloudSelection.h"
#include <QStringList>
#include <QWidget>
#include <array>
#include <map>
#include <set>

class QAction;
class QCheckBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QComboBox;
class QSpinBox;
class QStackedWidget;
class QTabWidget;
class QToolButton;
class QTreeWidget;
class QTreeWidgetItem;

class PointCloudRelationsPanel final : public QWidget
{
    Q_OBJECT
public:
    explicit PointCloudRelationsPanel(QWidget* parent = nullptr);
    void setJunctionPanel(QWidget* panel);
    void showJunctionPage();
    void setSourceInfo(const QString& path, const QString& unit);
    void setDocument(const PointCloudSelection& document, bool available);
    void updateSelection();
    void setPickedFace(int region);
    void setBusy(bool busy);
    void setAnalysisExportBusy(bool busy);
    void setCadReconstructionBusy(bool busy, bool cancellable);
    void setStatus(const QString& text);
    void setDefaultTolerances(double diagonal);
    double surfaceTolerance() const;
    double relationTolerance() const;
    double angleTolerance() const;
    bool automaticSurfaceDisplay() const;
    void setSurfaceDisplayState(const std::set<int>& available, const std::set<int>& displayed);
    void showBoundaryInspection(const QString& name, std::size_t points, std::size_t triangles,
                                std::size_t outerRings, std::size_t innerRings);
    void clearBoundaryInspection();
    void configureContourFit(double spacing, bool knownUnits);
    void setContourFitState(bool busy, bool ready, const QString& message = {});

signals:
    void showAllBoundariesChanged(bool enabled);
    void automaticSurfaceDisplayChanged(bool enabled);
    void regionSurfaceDisplayRequested(int region, bool enabled);
    void boundaryInspectionRequested(int region);
    void boundaryInspectionLayersChanged(bool points, bool surface, bool outerRings, bool innerRings);
    void boundaryInspectionCloseRequested();
    void contourFitSettingsChanged();
    void contourFitRequested(double tolerance, int maximumEdges, bool recognizeCircles,
                             bool engineering, double maximumLineTolerance, double circleTolerance);
    void contourSketchRequested();
    void detectRequested();
    void reconstructRequested();
    void cancelCadRequested();
    void inferRequested();
    void refitRequested(int id);
    void regionSelected(int id);
    void relationSelected(int id);
    void relationGroupSelected(const std::vector<int>& ids);
    void assignRequested(const QString& geometry, const QString& name, int id);
    void unassignRequested();
    void editRegionRequested(int id, const QString& name, const QString& geometry);
    void appendSelectionRequested(int id);
    void replaceSelectionRequested(int id);
    void removeRegionRequested(int id);
    void deleteRegionPointsRequested(int id);
    void deleteSelectedPointsRequested();
    void setRelationRequested(int id, int first, int second, const QString& type);
    void removeRelationRequested(int id);
    void undoRequested();
    void saveRequested();
    void loadRequested();
    void exportRequested();
    void analysisExportRequested();

private:
    int selectedRegion() const;
    int selectedRelation() const;
    void updateActions();
    void setCloudAvailable(bool available);
    void createRegion();
    void editRegion();
    void editRelation(bool create);
    void showContextMenu(QTreeWidget* tree, const QPoint& position);
    void loadPointPage(QTreeWidgetItem* item, int offset);
    void appendRelationNode(QTreeWidgetItem* parent, const PointCloudRelation& relation);
    QTreeWidgetItem* appendRelationGroup(QTreeWidgetItem* parent, std::size_t index);
    void loadRelationPage(QTreeWidgetItem* item, int offset, int count);
    void openRelationGroup(std::size_t index);
    void applyFilter();
    const PointCloudSelection* m_document = nullptr;
    std::map<int, std::vector<vtkIdType>> m_points;
    std::map<int, QTreeWidgetItem*> m_displayNodes;
    PointCloudRelationGrouping m_relationGrouping;
    std::map<int, QString> m_regionNames;
    std::map<int, QStringList> m_directionNames;
    std::vector<QTreeWidgetItem*> m_relationGroupItems;
    std::set<int> m_availableSurfaces, m_displayedSurfaces;
    int m_pickedRegion = 0;
    bool m_busy = false;
    bool m_analysisExportBusy = false;
    bool m_boundaryActive = false;
    QTabWidget* m_pages;
    QStackedWidget* m_boundaryPage;
    QWidget* m_boundaryInspection;
    QWidget* m_junctionPanel = nullptr;
    QWidget* m_cloudActions;
    QLabel* m_boundarySummary;
    QPushButton* m_closeBoundary;
    std::array<QCheckBox*, 4> m_boundaryLayers;
    QWidget* m_contourControls;
    QWidget* m_contourOptions;
    QComboBox* m_contourPrecision;
    QDoubleSpinBox* m_contourTolerance;
    QSpinBox* m_contourEdgeBudget;
    QCheckBox* m_contourCircles;
    QCheckBox* m_engineeringContour;
    QDoubleSpinBox* m_contourLineTolerance;
    QDoubleSpinBox* m_contourCircleTolerance;
    QLabel* m_contourStatus;
    QPushButton* m_fitContour;
    QPushButton* m_createContourSketch;
    double m_contourSpacing = 0;
    QTreeWidget* m_tree;
    QTreeWidget* m_relationTree;
    QTreeWidget* m_activeTree;
    QLabel* m_source;
    QLabel* m_summary;
    QLabel* m_selection;
    QLabel* m_status;
    QLineEdit* m_filter;
    QLineEdit* m_relationFilter;
    QDoubleSpinBox* m_surfaceTolerance;
    QCheckBox* m_automaticSurfaces;
    QDoubleSpinBox* m_relationTolerance;
    QDoubleSpinBox* m_angleTolerance;
    QPushButton* m_detect;
    QPushButton* m_cancelCad;
    QPushButton* m_infer;
    QToolButton* m_reconstruct;
    QAction* m_reconstructAction;
    QAction* m_analysisExportAction;
    QAction* m_newRegion;
    QAction* m_unassign;
    QAction* m_refit;
    QAction* m_editRegion;
    QAction* m_showSurface;
    QAction* m_showPoints;
    QAction* m_inspectBoundary;
    QAction* m_append;
    QAction* m_replace;
    QAction* m_removeRegion;
    QAction* m_deleteRegionPoints;
    QAction* m_deleteSelected;
    QAction* m_addRelation;
    QAction* m_editRelation;
    QAction* m_removeRelation;
    QAction* m_undo;
};
