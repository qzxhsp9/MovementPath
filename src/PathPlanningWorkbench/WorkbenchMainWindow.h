#pragma once

#include "ModelLoader.h"
#include "OcctViewWidget.h"

#include "GeometryQueryPathPlanner.h"
#include "VoxelPathPlanner.h"

#include <QFutureWatcher>
#include <QMainWindow>

#include <atomic>
#include <memory>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QPlainTextEdit;

namespace path_planning_workbench
{

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
    GoalPoint
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
    void OnPathComputationFinished();
    void SetPlanningUiBusy(bool busy);
    void UpdateEndpointOverlay();
    void BeginPick(PointPickMode mode);
    void ApplyPickedPoint(const gp_Pnt& point);
    void CycleStartDirection();
    void CycleGoalDirection();
    void AppendLog(const QString& line);
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

    OcctViewWidget* m_view = nullptr;
    QPlainTextEdit* m_log = nullptr;
    QPushButton* m_computeButton = nullptr;
    QPushButton* m_stopButton = nullptr;
    QLabel* m_modelLabel = nullptr;
    QComboBox* m_displayModeCombo = nullptr;
    QComboBox* m_plannerCombo = nullptr;
    QCheckBox* m_showKeyVoxelsCheck = nullptr;
    QCheckBox* m_realtimeCheck = nullptr;

    QDoubleSpinBox* m_linearDeflectionSpin = nullptr;
    QDoubleSpinBox* m_angularDeflectionSpin = nullptr;
    QDoubleSpinBox* m_voxelSizeSpin = nullptr;

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

    ImportedModel m_model;
    PointPickMode m_pickMode = PointPickMode::None;
    QFutureWatcher<VoxelPathPlannerResult>* m_planWatcher = nullptr;
    std::shared_ptr<std::atomic_bool> m_cancelRequested;
    double m_runningVoxelSize = 1.0;
};

} // namespace path_planning_workbench
