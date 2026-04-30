#include "WorkbenchMainWindow.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSplitter>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QApplication>
#include <QtConcurrent/QtConcurrentRun>

#include <iomanip>
#include <sstream>
#include <utility>

namespace path_planning_workbench
{
namespace
{
QDoubleSpinBox* MakeCoordinateSpin(double value = 0.0)
{
    QDoubleSpinBox* spin = new QDoubleSpinBox();
    spin->setRange(-1000000.0, 1000000.0);
    spin->setDecimals(4);
    spin->setSingleStep(1.0);
    spin->setValue(value);
    return spin;
}

QWidget* MakeVectorEditor(
    QDoubleSpinBox*& x,
    QDoubleSpinBox*& y,
    QDoubleSpinBox*& z,
    double ix = 0.0,
    double iy = 0.0,
    double iz = 0.0)
{
    QWidget* widget = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);
    layout->setContentsMargins(0, 0, 0, 0);

    x = MakeCoordinateSpin(ix);
    y = MakeCoordinateSpin(iy);
    z = MakeCoordinateSpin(iz);

    layout->addWidget(x);
    layout->addWidget(y);
    layout->addWidget(z);
    return widget;
}

std::vector<Vec> ToPathPoints(const std::vector<Vec>& points)
{
    return points;
}

std::vector<Vec> ToVoxelCenters(
    const std::vector<Vec>& pathPoints)
{
    return pathPoints;
}

QString ToText(VoxelAStarFailReason reason)
{
    switch (reason)
    {
    case VoxelAStarFailReason::None:
        return "None";
    case VoxelAStarFailReason::InvalidVoxelSpace:
        return "InvalidVoxelSpace";
    case VoxelAStarFailReason::StartOrGoalOutsideBounds:
        return "StartOrGoalOutsideBounds";
    case VoxelAStarFailReason::SnapStartFailed:
        return "SnapStartFailed";
    case VoxelAStarFailReason::SnapGoalFailed:
        return "SnapGoalFailed";
    case VoxelAStarFailReason::StartNotWalkable:
        return "StartNotWalkable";
    case VoxelAStarFailReason::GoalNotWalkable:
        return "GoalNotWalkable";
    case VoxelAStarFailReason::MaxVisitedExceeded:
        return "MaxVisitedExceeded";
    case VoxelAStarFailReason::OpenSetEmpty:
        return "OpenSetEmpty";
    case VoxelAStarFailReason::Cancelled:
        return "Cancelled";
    }

    return "Unknown";
}

QString ToText(const VoxelIndex& index)
{
    return QString("(%1, %2, %3)")
        .arg(index.x)
        .arg(index.y)
        .arg(index.z);
}

QString ToText(const VoxelBounds& bounds)
{
    if (!bounds.IsValid())
    {
        return "invalid";
    }

    return QString("%1 -> %2")
        .arg(ToText(bounds.minIndex))
        .arg(ToText(bounds.maxIndex));
}

std::vector<MeshTriangle> ToPlannerTriangles(
    const TriangleMeshData& mesh)
{
    std::vector<MeshTriangle> triangles;
    triangles.reserve(mesh.triangles.size());

    for (const std::array<int, 3>& tri : mesh.triangles)
    {
        if (tri[0] < 0 ||
            tri[1] < 0 ||
            tri[2] < 0 ||
            static_cast<std::size_t>(tri[0]) >= mesh.points.size() ||
            static_cast<std::size_t>(tri[1]) >= mesh.points.size() ||
            static_cast<std::size_t>(tri[2]) >= mesh.points.size())
        {
            continue;
        }

        const gp_Pnt& p0 = mesh.points[static_cast<std::size_t>(tri[0])];
        const gp_Pnt& p1 = mesh.points[static_cast<std::size_t>(tri[1])];
        const gp_Pnt& p2 = mesh.points[static_cast<std::size_t>(tri[2])];

        triangles.push_back({
            Vec(p0.X(), p0.Y(), p0.Z()),
            Vec(p1.X(), p1.Y(), p1.Z()),
            Vec(p2.X(), p2.Y(), p2.Z())
        });
    }

    return triangles;
}

gp_Vec NextAxisDirection(const gp_Vec& current)
{
    const gp_Vec directions[] = {
        gp_Vec(1.0, 0.0, 0.0),
        gp_Vec(0.0, 1.0, 0.0),
        gp_Vec(0.0, 0.0, 1.0),
        gp_Vec(-1.0, 0.0, 0.0),
        gp_Vec(0.0, -1.0, 0.0),
        gp_Vec(0.0, 0.0, -1.0)
    };

    gp_Vec normalized = current;
    if (normalized.SquareMagnitude() > 1.0e-12)
    {
        normalized.Normalize();
    }
    else
    {
        normalized = directions[5];
    }

    int bestIndex = 0;
    double bestDot = -2.0;
    for (int i = 0; i < 6; ++i)
    {
        const double dot = normalized.Dot(directions[i]);
        if (dot > bestDot)
        {
            bestDot = dot;
            bestIndex = i;
        }
    }

    return directions[(bestIndex + 1) % 6];
}

void AddPathEditorRow(
    QWidget* parent,
    QFormLayout* layout,
    const QString& label,
    QLineEdit*& lineEdit,
    const QString& initialPath)
{
    QWidget* row = new QWidget(parent);
    QHBoxLayout* rowLayout = new QHBoxLayout(row);
    rowLayout->setContentsMargins(0, 0, 0, 0);

    lineEdit = new QLineEdit(initialPath, row);
    QPushButton* browseButton = new QPushButton("Browse", row);
    rowLayout->addWidget(lineEdit, 1);
    rowLayout->addWidget(browseButton);

    QObject::connect(browseButton, &QPushButton::clicked, row, [lineEdit]() {
        const QString path = QFileDialog::getSaveFileName(
            lineEdit,
            "Select VTK output",
            lineEdit->text(),
            "VTK files (*.vtk)");
        if (!path.isEmpty())
        {
            lineEdit->setText(path);
        }
    });

    layout->addRow(label, row);
}
}

WorkbenchMainWindow::WorkbenchMainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    BuildUi();
    m_planWatcher = new QFutureWatcher<VoxelPathPlannerResult>(this);
    connect(m_planWatcher, &QFutureWatcher<VoxelPathPlannerResult>::finished,
        this, [this]() {
            OnPathComputationFinished();
        });
}

void WorkbenchMainWindow::BuildUi()
{
    setWindowTitle("MovementPath Workbench");
    resize(1400, 900);

    QSplitter* splitter = new QSplitter(this);
    m_view = new OcctViewWidget(splitter);

    QWidget* panel = new QWidget(splitter);
    panel->setMinimumWidth(360);
    QVBoxLayout* panelLayout = new QVBoxLayout(panel);

    QPushButton* importButton = new QPushButton("Import Model");
    QPushButton* applyDiscretizationButton =
        new QPushButton("Apply Discretization");
    m_computeButton = new QPushButton("Compute Path");
    m_stopButton = new QPushButton("Stop Computation");
    QPushButton* vtkExportSettingsButton =
        new QPushButton("VTK Export Settings");
    m_stopButton->setEnabled(false);
    m_modelLabel = new QLabel("No model loaded");

    panelLayout->addWidget(importButton);
    panelLayout->addWidget(m_modelLabel);

    QGroupBox* displayGroup = new QGroupBox("Display");
    QFormLayout* displayLayout = new QFormLayout(displayGroup);
    m_displayModeCombo = new QComboBox();
    m_displayModeCombo->addItem("Shaded");
    m_displayModeCombo->addItem("Wireframe");
    m_displayModeCombo->addItem("Mesh");
    m_showKeyVoxelsCheck = new QCheckBox("Show key voxels");
    displayLayout->addRow("Mode", m_displayModeCombo);
    displayLayout->addRow(m_showKeyVoxelsCheck);
    panelLayout->addWidget(displayGroup);

    QGroupBox* importGroup = new QGroupBox("Discretization");
    QFormLayout* importLayout = new QFormLayout(importGroup);
    m_linearDeflectionSpin = new QDoubleSpinBox();
    m_linearDeflectionSpin->setRange(0.001, 1000.0);
    m_linearDeflectionSpin->setDecimals(4);
    m_linearDeflectionSpin->setValue(0.1);
    m_angularDeflectionSpin = new QDoubleSpinBox();
    m_angularDeflectionSpin->setRange(0.001, 3.1416);
    m_angularDeflectionSpin->setDecimals(4);
    m_angularDeflectionSpin->setValue(0.5);
    importLayout->addRow("Linear deflection", m_linearDeflectionSpin);
    importLayout->addRow("Angular deflection", m_angularDeflectionSpin);
    importLayout->addRow(applyDiscretizationButton);
    panelLayout->addWidget(importGroup);

    QGroupBox* endpointGroup = new QGroupBox("Endpoints");
    QFormLayout* endpointLayout = new QFormLayout(endpointGroup);
    endpointLayout->addRow(
        "Start",
        MakeVectorEditor(m_startX, m_startY, m_startZ));
    endpointLayout->addRow(
        "Start dir",
        MakeVectorEditor(m_startDirX, m_startDirY, m_startDirZ, 1.0, 0.0, 0.0));
    endpointLayout->addRow(
        "Goal",
        MakeVectorEditor(m_goalX, m_goalY, m_goalZ));
    endpointLayout->addRow(
        "Goal dir",
        MakeVectorEditor(m_goalDirX, m_goalDirY, m_goalDirZ, 1.0, 0.0, 0.0));

    QWidget* pickWidget = new QWidget();
    QGridLayout* pickLayout = new QGridLayout(pickWidget);
    pickLayout->setContentsMargins(0, 0, 0, 0);
    QPushButton* pickStartButton = new QPushButton("Pick start");
    QPushButton* pickGoalButton = new QPushButton("Pick goal");
    pickLayout->addWidget(pickStartButton, 0, 0);
    pickLayout->addWidget(pickGoalButton, 0, 1);
    endpointLayout->addRow("Pick", pickWidget);
    panelLayout->addWidget(endpointGroup);

    QGroupBox* plannerGroup = new QGroupBox("Planner");
    QFormLayout* plannerLayout = new QFormLayout(plannerGroup);
    m_plannerCombo = new QComboBox();
    m_plannerCombo->addItem("Voxel full bounds");
    m_plannerCombo->addItem("Voxel lazy");
    m_plannerCombo->addItem("Geometry query");
    m_voxelSizeSpin = new QDoubleSpinBox();
    m_voxelSizeSpin->setRange(0.01, 1000.0);
    m_voxelSizeSpin->setDecimals(4);
    m_voxelSizeSpin->setValue(1.0);
    m_clearanceSpin = new QDoubleSpinBox();
    m_clearanceSpin->setRange(0.0, 1000000.0);
    m_clearanceSpin->setDecimals(4);
    m_clearanceSpin->setSingleStep(1.0);
    m_clearanceSpin->setValue(3.0);
    m_snapRadiusSpin = new QSpinBox();
    m_snapRadiusSpin->setRange(0, 1000000);
    m_snapRadiusSpin->setValue(20);
    m_searchModeCombo = new QComboBox();
    m_searchModeCombo->addItem("Clearance band");
    m_searchModeCombo->addItem("Free space");
    m_neighborTypeCombo = new QComboBox();
    m_neighborTypeCombo->addItem("6-face");
    m_neighborTypeCombo->addItem("18-face-edge");
    m_neighborTypeCombo->addItem("26-face-edge-vertex");
    m_neighborTypeCombo->setCurrentIndex(2);
    m_realtimeCheck = new QCheckBox("Realtime after input changes");
    plannerLayout->addRow("Method", m_plannerCombo);
    plannerLayout->addRow("Search", m_searchModeCombo);
    plannerLayout->addRow("Neighbors", m_neighborTypeCombo);
    plannerLayout->addRow("Voxel size", m_voxelSizeSpin);
    plannerLayout->addRow("Clearance", m_clearanceSpin);
    plannerLayout->addRow("Snap radius", m_snapRadiusSpin);
    plannerLayout->addRow(m_realtimeCheck);
    plannerLayout->addRow(vtkExportSettingsButton);
    plannerLayout->addRow(m_computeButton);
    plannerLayout->addRow(m_stopButton);
    panelLayout->addWidget(plannerGroup);

    m_log = new QPlainTextEdit();
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(500);
    panelLayout->addWidget(m_log, 1);

    splitter->addWidget(m_view);
    splitter->addWidget(panel);
    splitter->setStretchFactor(0, 1);
    setCentralWidget(splitter);

    connect(importButton, &QPushButton::clicked, this, [this]() {
        ImportModel();
    });
    connect(applyDiscretizationButton, &QPushButton::clicked, this, [this]() {
        ApplyDiscretization();
    });
    connect(m_computeButton, &QPushButton::clicked, this, [this]() {
        ComputePath();
    });
    connect(m_stopButton, &QPushButton::clicked, this, [this]() {
        StopPathComputation();
    });
    connect(vtkExportSettingsButton, &QPushButton::clicked, this, [this]() {
        OpenVtkExportSettings();
    });
    connect(m_displayModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
        this, [this]() {
            if (m_model.kind != ImportedModelKind::Unknown)
            {
                m_view->SetDisplayMode(ReadDisplayMode());
            }
        });
    connect(pickStartButton, &QPushButton::clicked, this, [this]() {
        BeginPick(PointPickMode::StartPoint);
    });
    connect(pickGoalButton, &QPushButton::clicked, this, [this]() {
        BeginPick(PointPickMode::GoalPoint);
    });
    m_view->SetPointPickCallback([this](const gp_Pnt& point) {
        ApplyPickedPoint(point);
    });

    const auto endpointChanged = [this]() {
        UpdateEndpointOverlay();
        if (m_realtimeCheck->isChecked())
        {
            ComputePath();
        }
    };

    for (QDoubleSpinBox* spin : {
        m_startX, m_startY, m_startZ,
        m_startDirX, m_startDirY, m_startDirZ,
        m_goalX, m_goalY, m_goalZ,
        m_goalDirX, m_goalDirY, m_goalDirZ})
    {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, endpointChanged);
    }
}

void WorkbenchMainWindow::ImportModel()
{
    const QString path = QFileDialog::getOpenFileName(
        this,
        "Import model",
        QString(),
        "Models (*.step *.stp *.brep *.vtk)");

    if (path.isEmpty())
    {
        return;
    }

    ImportedModel model;
    QString error;
    if (!ModelLoader::Load(path, ReadImportOptions(), model, error))
    {
        AppendLog(error);
        return;
    }

    m_model = model;
    m_modelLabel->setText(path);
    AppendLog("Imported: " + path);
    RefreshModelDisplay();
    UpdateEndpointOverlay();
}

void WorkbenchMainWindow::ApplyDiscretization()
{
    if (m_model.kind == ImportedModelKind::Unknown)
    {
        AppendLog("No model loaded; discretization was not applied.");
        return;
    }

    if (m_model.kind == ImportedModelKind::Mesh)
    {
        AppendLog("VTK mesh is already discrete; discretization parameters apply only to STEP/BREP shapes.");
        return;
    }

    AppendLog("Applying discretization to current model...");
    QApplication::processEvents();

    QString error;
    if (!ModelLoader::RemeshShape(
            m_model.shape,
            ReadImportOptions(),
            error))
    {
        AppendLog(error);
        return;
    }

    AppendLog("Discretization applied. Display and future path planning use the updated mesh.");
    RefreshModelDisplay();
    UpdateEndpointOverlay();
}

void WorkbenchMainWindow::RefreshModelDisplay()
{
    if (m_model.kind == ImportedModelKind::Unknown)
    {
        return;
    }

    if (m_model.kind == ImportedModelKind::Mesh)
    {
        m_view->DisplayMesh(m_model.mesh, ReadDisplayMode());
    }
    else
    {
        m_view->DisplayShape(m_model.shape, ReadDisplayMode());
    }
}

void WorkbenchMainWindow::ComputePath()
{
    if (m_planWatcher != nullptr && m_planWatcher->isRunning())
    {
        AppendLog("A path computation is already running. Stop it before starting another one.");
        return;
    }

    if (m_model.kind == ImportedModelKind::Unknown)
    {
        AppendLog("No model loaded.");
        return;
    }

    m_view->ClearOverlays();
    UpdateEndpointOverlay();

    const PlannerMethod method = ReadPlannerMethod();

    AppendLog("Preparing path computation...");
    AppendLog(QString("Planner: %1, search=%2, neighbors=%3, voxelSize=%4, clearance=%5, snapRadius=%6, linearDeflection=%7, angularDeflection=%8")
        .arg(m_plannerCombo->currentText())
        .arg(m_searchModeCombo->currentText())
        .arg(m_neighborTypeCombo->currentText())
        .arg(m_voxelSizeSpin->value())
        .arg(m_clearanceSpin->value())
        .arg(m_snapRadiusSpin->value())
        .arg(m_linearDeflectionSpin->value())
        .arg(m_angularDeflectionSpin->value()));
    QApplication::processEvents();

    if (method == PlannerMethod::GeometryQuery)
    {
        AppendLog("Geometry-query planner is not connected to OCCT import yet.");
        return;
    }

    VoxelPlanningScenario scenario;
    scenario.name = "workbench";
    if (m_model.kind == ImportedModelKind::Mesh)
    {
        scenario.triangles = ToPlannerTriangles(m_model.mesh);
        AppendLog(QString("Using VTK triangle mesh directly: %1 triangles.")
            .arg(scenario.triangles.size()));
        if (scenario.triangles.empty())
        {
            AppendLog("VTK mesh does not contain valid triangles.");
            return;
        }
    }
    else
    {
        scenario.shape = m_model.shape;
    }
    scenario.startPoint = ReadPoint(m_startX, m_startY, m_startZ);
    scenario.startDir = ReadDirection(m_startDirX, m_startDirY, m_startDirZ);
    scenario.goalPoint = ReadPoint(m_goalX, m_goalY, m_goalZ);
    scenario.goalDir = ReadDirection(m_goalDirX, m_goalDirY, m_goalDirZ);

    VoxelPathPlannerOptions options = MakeVoxelOptions(method);
    m_cancelRequested = std::make_shared<std::atomic_bool>(false);
    const std::shared_ptr<std::atomic_bool> cancelRequested =
        m_cancelRequested;
    options.runOptions.shouldCancel = [cancelRequested]()
    {
        return cancelRequested != nullptr &&
            cancelRequested->load(std::memory_order_relaxed);
    };

    m_runningVoxelSize = options.meshBuildOptions.voxelSize;
    AppendLog("Voxel planner started...");
    QApplication::processEvents();

    SetPlanningUiBusy(true);

    m_planWatcher->setFuture(QtConcurrent::run(
        [scenario, options]()
        {
            return VoxelPathPlanner::Plan(scenario, options);
        }));
}

void WorkbenchMainWindow::StopPathComputation()
{
    if (m_cancelRequested != nullptr)
    {
        m_cancelRequested->store(true, std::memory_order_relaxed);
        AppendLog("Stop requested. Waiting for planner to reach a cancellation point...");
    }
}

void WorkbenchMainWindow::OpenVtkExportSettings()
{
    QDialog dialog(this);
    dialog.setWindowTitle("VTK Export Settings");

    QVBoxLayout* dialogLayout = new QVBoxLayout(&dialog);
    QCheckBox* exportCheck = new QCheckBox("Export VTK during path computation");
    exportCheck->setChecked(m_vtkExportSettings.exportEnabled);
    dialogLayout->addWidget(exportCheck);

    QGroupBox* pathGroup = new QGroupBox("Output paths", &dialog);
    QFormLayout* pathLayout = new QFormLayout(pathGroup);

    QLineEdit* shapeMeshPath = nullptr;
    QLineEdit* astarFailedPath = nullptr;
    QLineEdit* astarPath = nullptr;
    QLineEdit* optimizedPathVoxelsPath = nullptr;
    QLineEdit* optimizedPathPolylinePath = nullptr;
    QLineEdit* lazyChunkBoundsPath = nullptr;

    AddPathEditorRow(
        pathGroup,
        pathLayout,
        "Shape mesh",
        shapeMeshPath,
        m_vtkExportSettings.shapeMeshPath);
    AddPathEditorRow(
        pathGroup,
        pathLayout,
        "A* failed voxels",
        astarFailedPath,
        m_vtkExportSettings.astarFailedPath);
    AddPathEditorRow(
        pathGroup,
        pathLayout,
        "A* path voxels",
        astarPath,
        m_vtkExportSettings.astarPath);
    AddPathEditorRow(
        pathGroup,
        pathLayout,
        "Optimized voxels",
        optimizedPathVoxelsPath,
        m_vtkExportSettings.optimizedPathVoxelsPath);
    AddPathEditorRow(
        pathGroup,
        pathLayout,
        "Optimized polyline",
        optimizedPathPolylinePath,
        m_vtkExportSettings.optimizedPathPolylinePath);
    AddPathEditorRow(
        pathGroup,
        pathLayout,
        "Lazy chunk bounds",
        lazyChunkBoundsPath,
        m_vtkExportSettings.lazyChunkBoundsPath);

    dialogLayout->addWidget(pathGroup);

    QDialogButtonBox* buttons = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
        &dialog);
    dialogLayout->addWidget(buttons);

    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted)
    {
        return;
    }

    m_vtkExportSettings.exportEnabled = exportCheck->isChecked();
    m_vtkExportSettings.shapeMeshPath = shapeMeshPath->text();
    m_vtkExportSettings.astarFailedPath = astarFailedPath->text();
    m_vtkExportSettings.astarPath = astarPath->text();
    m_vtkExportSettings.optimizedPathVoxelsPath =
        optimizedPathVoxelsPath->text();
    m_vtkExportSettings.optimizedPathPolylinePath =
        optimizedPathPolylinePath->text();
    m_vtkExportSettings.lazyChunkBoundsPath = lazyChunkBoundsPath->text();

    AppendLog(QString("VTK export %1.")
        .arg(m_vtkExportSettings.exportEnabled ? "enabled" : "disabled"));
}

void WorkbenchMainWindow::OnPathComputationFinished()
{
    SetPlanningUiBusy(false);

    const bool wasCancelled =
        m_cancelRequested != nullptr &&
        m_cancelRequested->load(std::memory_order_relaxed);
    m_cancelRequested.reset();

    const VoxelPathPlannerResult result = m_planWatcher->result();
    AppendLog("Voxel planner finished.");

    if (wasCancelled)
    {
        AppendLog("Path computation was cancelled.");
        return;
    }

    std::ostringstream ss;
    ss << "Voxel plan "
        << (result.success ? "succeeded" : "failed")
        << ", mode=" << result.profile.buildRegionMode
        << ", triangles=" << result.profile.triangleCount
        << ", rawPath=" << result.profile.rawPathCount
        << ", optimizedPath=" << result.profile.optimizedPathCount
        << ", cost=" << std::fixed << std::setprecision(3)
        << result.profile.totalCost;
    AppendLog(QString::fromStdString(ss.str()));

    if (!result.success)
    {
        AppendLog(QString("A* failure reason: %1, visited=%2")
            .arg(ToText(result.astarResult.failReason))
            .arg(result.astarResult.visitedCount));
        AppendLog(QString("Input voxel start=%1, goal=%2")
            .arg(ToText(result.astarResult.inputStartIndex))
            .arg(ToText(result.astarResult.inputGoalIndex)));
        AppendLog(QString("Search voxel start=%1%2, goal=%3%4")
            .arg(ToText(result.astarResult.startIndex))
            .arg(result.astarResult.startSnapped ? " snapped" : "")
            .arg(ToText(result.astarResult.goalIndex))
            .arg(result.astarResult.goalSnapped ? " snapped" : ""));
        AppendLog(QString("Voxel states: stored=%1, occupied=%2, clearanceBand=%3")
            .arg(result.profile.storedCellCount)
            .arg(result.profile.occupiedCount)
            .arg(result.profile.clearanceBandCount));
        AppendLog(QString("Build/search: buildSucceeded=%1, astarSucceeded=%2, attempts=%3, bounds=%4")
            .arg(result.profile.buildSucceeded ? "true" : "false")
            .arg(result.profile.astarSucceeded ? "true" : "false")
            .arg(result.profile.buildAttemptCount)
            .arg(ToText(result.finalSearchBounds)));
    }

    const std::vector<Vec> pathPoints =
        result.optimizeResult.pointPath.empty() ?
            ToPathPoints(result.astarResult.pointPath) :
            ToPathPoints(result.optimizeResult.pointPath);
    m_view->DisplayPath(pathPoints);

    if (m_showKeyVoxelsCheck->isChecked())
    {
        m_view->DisplayKeyVoxels(
            ToVoxelCenters(pathPoints),
            m_runningVoxelSize);
    }
}

void WorkbenchMainWindow::SetPlanningUiBusy(bool busy)
{
    if (m_computeButton != nullptr)
    {
        m_computeButton->setEnabled(!busy);
    }
    if (m_stopButton != nullptr)
    {
        m_stopButton->setEnabled(busy);
    }
}

void WorkbenchMainWindow::UpdateEndpointOverlay()
{
    if (m_model.kind == ImportedModelKind::Unknown)
    {
        return;
    }

    m_view->DisplayEndpoints(
        ReadPoint(m_startX, m_startY, m_startZ),
        ReadDirection(m_startDirX, m_startDirY, m_startDirZ),
        ReadPoint(m_goalX, m_goalY, m_goalZ),
        ReadDirection(m_goalDirX, m_goalDirY, m_goalDirZ));
}

void WorkbenchMainWindow::BeginPick(PointPickMode mode)
{
    m_pickMode = mode;
    AppendLog("Double-click a model surface to set the selected point.");
}

void WorkbenchMainWindow::ApplyPickedPoint(const gp_Pnt& point)
{
    switch (m_pickMode)
    {
    case PointPickMode::StartPoint:
        m_startX->setValue(point.X());
        m_startY->setValue(point.Y());
        m_startZ->setValue(point.Z());
        break;
    case PointPickMode::GoalPoint:
        m_goalX->setValue(point.X());
        m_goalY->setValue(point.Y());
        m_goalZ->setValue(point.Z());
        break;
    case PointPickMode::None:
        return;
    }

    m_pickMode = PointPickMode::None;
    UpdateEndpointOverlay();
}

void WorkbenchMainWindow::CycleStartDirection()
{
    const gp_Vec next =
        NextAxisDirection(ReadDirection(m_startDirX, m_startDirY, m_startDirZ));
    SetDirectionSpins(m_startDirX, m_startDirY, m_startDirZ, next);
    AppendLog("Start direction changed by Q.");
    UpdateEndpointOverlay();
}

void WorkbenchMainWindow::CycleGoalDirection()
{
    const gp_Vec next =
        NextAxisDirection(ReadDirection(m_goalDirX, m_goalDirY, m_goalDirZ));
    SetDirectionSpins(m_goalDirX, m_goalDirY, m_goalDirZ, next);
    AppendLog("Goal direction changed by E.");
    UpdateEndpointOverlay();
}

void WorkbenchMainWindow::AppendLog(const QString& line)
{
    m_log->appendPlainText(line);
    m_log->ensureCursorVisible();
}

void WorkbenchMainWindow::SetDirectionSpins(
    QDoubleSpinBox* x,
    QDoubleSpinBox* y,
    QDoubleSpinBox* z,
    const gp_Vec& direction)
{
    const QSignalBlocker blockX(x);
    const QSignalBlocker blockY(y);
    const QSignalBlocker blockZ(z);
    x->setValue(direction.X());
    y->setValue(direction.Y());
    z->setValue(direction.Z());
}

void WorkbenchMainWindow::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_Q)
    {
        CycleStartDirection();
        event->accept();
        return;
    }
    if (event->key() == Qt::Key_E)
    {
        CycleGoalDirection();
        event->accept();
        return;
    }

    QMainWindow::keyPressEvent(event);
}

gp_Pnt WorkbenchMainWindow::ReadPoint(
    QDoubleSpinBox* x,
    QDoubleSpinBox* y,
    QDoubleSpinBox* z) const
{
    return gp_Pnt(x->value(), y->value(), z->value());
}

gp_Vec WorkbenchMainWindow::ReadDirection(
    QDoubleSpinBox* x,
    QDoubleSpinBox* y,
    QDoubleSpinBox* z) const
{
    return gp_Vec(x->value(), y->value(), z->value());
}

ModelImportOptions WorkbenchMainWindow::ReadImportOptions() const
{
    ModelImportOptions options;
    options.linearDeflection = m_linearDeflectionSpin->value();
    options.angularDeflection = m_angularDeflectionSpin->value();
    options.remeshShape = true;
    return options;
}

ViewDisplayMode WorkbenchMainWindow::ReadDisplayMode() const
{
    if (m_displayModeCombo->currentIndex() == 1)
    {
        return ViewDisplayMode::Wireframe;
    }
    if (m_displayModeCombo->currentIndex() == 2)
    {
        return ViewDisplayMode::Mesh;
    }
    return ViewDisplayMode::Shaded;
}

PlannerMethod WorkbenchMainWindow::ReadPlannerMethod() const
{
    if (m_plannerCombo->currentIndex() == 1)
    {
        return PlannerMethod::VoxelLazy;
    }
    if (m_plannerCombo->currentIndex() == 2)
    {
        return PlannerMethod::GeometryQuery;
    }
    return PlannerMethod::VoxelFullBounds;
}

VoxelPathPlannerOptions WorkbenchMainWindow::MakeVoxelOptions(
    PlannerMethod method) const
{
    VoxelPathPlannerOptions options =
        VoxelPathPlanner::MakeDefaultOptions();
    options.meshBuildOptions.voxelSize = m_voxelSizeSpin->value();
    options.meshBuildOptions.meshDeflection = m_linearDeflectionSpin->value();
    options.meshBuildOptions.angularDeflection =
        m_angularDeflectionSpin->value();
    options.meshBuildOptions.clearance = m_clearanceSpin->value();
    options.astarOptions.snapMaxRadius = m_snapRadiusSpin->value();
    options.astarOptions.searchMode =
        m_searchModeCombo->currentIndex() == 1 ?
            VoxelAStarSearchMode::FreeSpace :
            VoxelAStarSearchMode::ClearanceBand;

    if (m_neighborTypeCombo->currentIndex() == 1)
    {
        options.astarOptions.neighborType = VoxelNeighborType::FaceEdge18;
    }
    else if (m_neighborTypeCombo->currentIndex() == 2)
    {
        options.astarOptions.neighborType =
            VoxelNeighborType::FaceEdgeVertex26;
    }
    else
    {
        options.astarOptions.neighborType = VoxelNeighborType::Face6;
    }

    options.runOptions.exportVtk = m_vtkExportSettings.exportEnabled;
    options.runOptions.shapeMeshVtkPath =
        m_vtkExportSettings.shapeMeshPath.toStdString();
    options.runOptions.astarFailedVtkPath =
        m_vtkExportSettings.astarFailedPath.toStdString();
    options.runOptions.astarPathVtkPath =
        m_vtkExportSettings.astarPath.toStdString();
    options.runOptions.optimizedPathVoxelsVtkPath =
        m_vtkExportSettings.optimizedPathVoxelsPath.toStdString();
    options.runOptions.optimizedPathPolylineVtkPath =
        m_vtkExportSettings.optimizedPathPolylinePath.toStdString();
    options.runOptions.lazyChunkBoundsVtkPath =
        m_vtkExportSettings.lazyChunkBoundsPath.toStdString();
    options.runOptions.debugNeighborhood = false;
    options.runOptions.verbose = false;

    if (method == PlannerMethod::VoxelLazy)
    {
        options.lazyBuildOptions.enabled = true;
    }
    else
    {
        options.localBuildOptions.regionMode =
            VoxelBuildRegionMode::FullMeshBounds;
        options.lazyBuildOptions.enabled = false;
    }

    return options;
}

} // namespace path_planning_workbench
