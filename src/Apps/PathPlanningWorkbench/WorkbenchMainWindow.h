#pragma once

#include "ModelLoader.h"
#include "OcctViewWidget.h"

#include "GeometryQueryPathPlanner.h"
#include "VoxelPathPlanner.h"

#include <QElapsedTimer>
#include <QFutureWatcher>
#include <QMainWindow>

#include <atomic>
#include <memory>
#include <vector>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QPlainTextEdit;
class QSpinBox;
class QVBoxLayout;

namespace path_planning_workbench
{

struct VtkExportSettings
{
    bool exportEnabled = false;
    QString shapeMeshPath = "D:/shape_mesh.vtk";
    QString astarFailedPath = "D:/astar_failed.vtk";
    QString astarPath = "D:/astar_path.vtk";
    QString optimizedPathVoxelsPath = "D:/optimized_path_voxels.vtk";
    QString optimizedPathPolylinePath = "D:/optimized_path_polyline.vtk";
};

enum class PlannerMethod
{
    VoxelFullBounds,
    VoxelLazy,
    GeometryQuery
};

enum class PointPickMode
{
    None,
    StartPoint,
    GoalPoint,
    RestrictedRegionPoint
};

struct RestrictedRegion
{
    Vec point;
    Vec normal = Vec(0.0, 0.0, 1.0);
};

class WorkbenchMainWindow final : public QMainWindow
{
public:
    explicit WorkbenchMainWindow(QWidget* parent = nullptr);

private:
    void BuildUi();
    void ImportModel();
    void ApplyDiscretization();
    void RefreshModelDisplay();
    void ComputePath();
    void StopPathComputation();
    void OpenVtkExportSettings();
    void OnPathComputationFinished();
    void SetPlanningUiBusy(bool busy);
    void UpdateEndpointOverlay();
    void BeginPick(PointPickMode mode);
    void BeginRestrictedRegionPointPick(std::size_t index);
    void ApplyPickedPoint(const gp_Pnt& point);
    void CycleStartDirection();
    void CycleGoalDirection();
    void AppendLog(const QString& line);
    void NotifyPlanningInputChanged();
    void AddRestrictedRegion();
    void EditRestrictedRegion(std::size_t index);
    void RemoveRestrictedRegion(std::size_t index);
    void RefreshRestrictedRegionList();
    void SetDirectionSpins(
        QDoubleSpinBox* x,
        QDoubleSpinBox* y,
        QDoubleSpinBox* z,
        const gp_Vec& direction);

protected:
    void keyPressEvent(QKeyEvent* event) override;

    gp_Pnt ReadPoint(
        QDoubleSpinBox* x,
        QDoubleSpinBox* y,
        QDoubleSpinBox* z) const;

    gp_Vec ReadDirection(
        QDoubleSpinBox* x,
        QDoubleSpinBox* y,
        QDoubleSpinBox* z) const;

    ModelImportOptions ReadImportOptions() const;
    ViewDisplayMode ReadDisplayMode() const;
    PlannerMethod ReadPlannerMethod() const;

    VoxelPathPlannerOptions MakeVoxelOptions(
        PlannerMethod method) const;
    std::vector<VoxelRestrictedHalfSpace> ReadRestrictedHalfSpaces() const;

    OcctViewWidget* m_view = nullptr;
    QPlainTextEdit* m_log = nullptr;
    QPushButton* m_computeButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QLabel* m_modelLabel = nullptr;
    QComboBox* m_displayModeCombo = nullptr;
    QComboBox* m_plannerCombo = nullptr;
    QCheckBox* m_showPathVoxelsCheck = nullptr;
    QCheckBox* m_realtimeCheck = nullptr;
    QComboBox* m_searchModeCombo = nullptr;
    QComboBox* m_neighborTypeCombo = nullptr;
    QCheckBox* m_smoothPathCheck = nullptr;
    QCheckBox* m_enableRestrictedRegionsCheck = nullptr;
    QVBoxLayout* m_restrictedRegionListLayout = nullptr;

    QDoubleSpinBox* m_linearDeflectionSpin = nullptr;
    QDoubleSpinBox* m_angularDeflectionSpin = nullptr;
    QDoubleSpinBox* m_voxelSizeSpin = nullptr;
    QDoubleSpinBox* m_clearanceSpin = nullptr;
    QDoubleSpinBox* m_lazyTimeBudgetSpin = nullptr;
    QSpinBox* m_snapRadiusSpin = nullptr;

    QDoubleSpinBox* m_startX = nullptr;
    QDoubleSpinBox* m_startY = nullptr;
    QDoubleSpinBox* m_startZ = nullptr;
    QDoubleSpinBox* m_startDirX = nullptr;
    QDoubleSpinBox* m_startDirY = nullptr;
    QDoubleSpinBox* m_startDirZ = nullptr;

    QDoubleSpinBox* m_goalX = nullptr;
    QDoubleSpinBox* m_goalY = nullptr;
    QDoubleSpinBox* m_goalZ = nullptr;
    QDoubleSpinBox* m_goalDirX = nullptr;
    QDoubleSpinBox* m_goalDirY = nullptr;
    QDoubleSpinBox* m_goalDirZ = nullptr;
    std::vector<RestrictedRegion> m_restrictedRegions;

    ImportedModel m_model;
    PointPickMode m_pickMode = PointPickMode::None;
    std::size_t m_pickRestrictedRegionIndex = 0;
    QFutureWatcher<VoxelPathPlannerResult>* m_planWatcher = nullptr;
    std::shared_ptr<std::atomic_bool> m_cancelRequested;
    QElapsedTimer m_planTimer;
    VtkExportSettings m_vtkExportSettings;
    double m_runningVoxelSize = 1.0;
};

} // namespace path_planning_workbench
