#include "WorkbenchMainWindow.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLayoutItem>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QSettings>
#include <QSignalBlocker>
#include <QSplitter>
#include <QSpinBox>
#include <QStyle>
#include <QVBoxLayout>
#include <QToolButton>
#include <QApplication>
#include <QtConcurrent/QtConcurrentRun>

#include <algorithm>
#include <utility>

namespace path_planning_workbench
{
namespace
{
constexpr const char* kSettingsOrganization = "MovementPath";
constexpr const char* kSettingsApplication = "PathPlanningWorkbench";
constexpr const char* kLastModelDirectoryKey = "import/lastModelDirectory";

QDoubleSpinBox* MakeCoordinateSpin(double value = 0.0)
{
    QDoubleSpinBox* spin = new QDoubleSpinBox();
    spin->setRange(-1000000.0, 1000000.0);
    spin->setDecimals(4);
    spin->setSingleStep(1.0);
    spin->setValue(value);
    return spin;
}

QDoubleSpinBox* MakeTuningDoubleSpin(
    double value,
    double minimum,
    double maximum,
    int decimals = 4,
    double singleStep = 0.1)
{
    QDoubleSpinBox* spin = new QDoubleSpinBox();
    spin->setRange(minimum, maximum);
    spin->setDecimals(decimals);
    spin->setSingleStep(singleStep);
    spin->setValue(value);
    return spin;
}

void SetDefaultTooltip(
    QWidget* widget,
    const QString& variableName,
    const QString& description,
    double defaultValue)
{
    widget->setToolTip(QString("%1\n变量名: %2\n默认值: %3")
        .arg(description)
        .arg(variableName)
        .arg(defaultValue, 0, 'g', 8));
}

void SetDefaultTooltip(
    QWidget* widget,
    const QString& variableName,
    const QString& description,
    int defaultValue)
{
    widget->setToolTip(QString("%1\n变量名: %2\n默认值: %3")
        .arg(description)
        .arg(variableName)
        .arg(defaultValue));
}

QSpinBox* MakeTuningIntSpin(
    int value,
    int minimum,
    int maximum)
{
    QSpinBox* spin = new QSpinBox();
    spin->setRange(minimum, maximum);
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
    QToolButton* vtkExportSettingsButton = new QToolButton();
    vtkExportSettingsButton->setIcon(style()->standardIcon(QStyle::SP_FileDialogDetailedView));
    vtkExportSettingsButton->setToolTip("VTK Export Settings");
    QPushButton* pathTuningSettingsButton =
        new QPushButton("Path Settings");
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
    m_showPathVoxelsCheck = new QCheckBox("Path voxels");
    QWidget* voxelDisplayRow = new QWidget(displayGroup);
    QHBoxLayout* voxelDisplayLayout = new QHBoxLayout(voxelDisplayRow);
    voxelDisplayLayout->setContentsMargins(0, 0, 0, 0);
    voxelDisplayLayout->addWidget(m_showPathVoxelsCheck);
    voxelDisplayLayout->addStretch(1);
    voxelDisplayLayout->addWidget(vtkExportSettingsButton);
    displayLayout->addRow("Mode", m_displayModeCombo);
    displayLayout->addRow("Voxels", voxelDisplayRow);
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

    QWidget* deflectionWidget = new QWidget(importGroup);
    QGridLayout* deflectionLayout = new QGridLayout(deflectionWidget);
    deflectionLayout->setContentsMargins(0, 0, 0, 0);
    deflectionLayout->addWidget(new QLabel("Linear", deflectionWidget), 0, 0);
    deflectionLayout->addWidget(m_linearDeflectionSpin, 0, 1);
    deflectionLayout->addWidget(new QLabel("Angular", deflectionWidget), 0, 2);
    deflectionLayout->addWidget(m_angularDeflectionSpin, 0, 3);
    deflectionLayout->setColumnStretch(1, 1);
    deflectionLayout->setColumnStretch(3, 1);
    importLayout->addRow(deflectionWidget);
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

    QGroupBox* restrictedGroup = new QGroupBox("Restricted Half-spaces");
    QVBoxLayout* restrictedLayout = new QVBoxLayout(restrictedGroup);
    QWidget* restrictedHeader = new QWidget(restrictedGroup);
    QHBoxLayout* restrictedHeaderLayout = new QHBoxLayout(restrictedHeader);
    restrictedHeaderLayout->setContentsMargins(0, 0, 0, 0);
    m_enableRestrictedRegionsCheck = new QCheckBox("Enable regions", restrictedHeader);
    QPushButton* addRestrictedRegionButton = new QPushButton("+", restrictedHeader);
    addRestrictedRegionButton->setFixedWidth(32);
    addRestrictedRegionButton->setToolTip("Add region");
    restrictedHeaderLayout->addWidget(m_enableRestrictedRegionsCheck);
    restrictedHeaderLayout->addStretch(1);
    restrictedHeaderLayout->addWidget(addRestrictedRegionButton);
    restrictedLayout->addWidget(restrictedHeader);
    m_restrictedRegionListLayout = new QVBoxLayout();
    m_restrictedRegionListLayout->setContentsMargins(0, 0, 0, 0);
    restrictedLayout->addLayout(m_restrictedRegionListLayout);
    panelLayout->addWidget(restrictedGroup);

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
    m_lazyTimeBudgetSpin = new QDoubleSpinBox();
    m_lazyTimeBudgetSpin->setRange(0.0, 600000.0);
    m_lazyTimeBudgetSpin->setDecimals(0);
    m_lazyTimeBudgetSpin->setSingleStep(500.0);
    m_lazyTimeBudgetSpin->setValue(3000.0);
    m_searchModeCombo = new QComboBox();
    m_searchModeCombo->addItem("Clearance band");
    m_searchModeCombo->addItem("Free space");
    m_neighborTypeCombo = new QComboBox();
    m_neighborTypeCombo->addItem("6-face");
    m_neighborTypeCombo->addItem("18-face-edge");
    m_neighborTypeCombo->addItem("26-face-edge-vertex");
    m_neighborTypeCombo->setCurrentIndex(2);
    m_smoothPathCheck = new QCheckBox("Smooth path");
    m_smoothPathCheck->setChecked(true);
    QWidget* smoothPathRow = new QWidget(plannerGroup);
    QHBoxLayout* smoothPathLayout = new QHBoxLayout(smoothPathRow);
    smoothPathLayout->setContentsMargins(0, 0, 0, 0);
    smoothPathLayout->addWidget(m_smoothPathCheck);
    smoothPathLayout->addStretch(1);
    smoothPathLayout->addWidget(pathTuningSettingsButton);
    m_realtimeCheck = new QCheckBox("Realtime after input changes");
    plannerLayout->addRow("Method", m_plannerCombo);
    plannerLayout->addRow("Search", m_searchModeCombo);
    plannerLayout->addRow("Neighbors", m_neighborTypeCombo);
    plannerLayout->addRow("Voxel size", m_voxelSizeSpin);
    plannerLayout->addRow("Clearance", m_clearanceSpin);
    plannerLayout->addRow("Snap radius", m_snapRadiusSpin);
    plannerLayout->addRow("Lazy timeout ms", m_lazyTimeBudgetSpin);
    plannerLayout->addRow(smoothPathRow);
    plannerLayout->addRow(m_realtimeCheck);
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
    connect(pathTuningSettingsButton, &QPushButton::clicked, this, [this]() {
        OpenPathTuningSettings();
    });
    connect(addRestrictedRegionButton, &QPushButton::clicked, this, [this]() {
        AddRestrictedRegion();
    });
    connect(m_enableRestrictedRegionsCheck, &QCheckBox::toggled, this, [this]() {
        NotifyPlanningInputChanged();
    });
    QShortcut* resetViewShortcut =
        new QShortcut(QKeySequence(Qt::Key_Space), this);
    resetViewShortcut->setContext(Qt::WindowShortcut);
    connect(resetViewShortcut, &QShortcut::activated, this, [this]() {
        if (m_view != nullptr)
        {
            m_view->ResetToInitialView();
        }
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
    m_view->SetPointPickCallback([this](const gp_Pnt& point, const gp_Vec& normal) {
        ApplyPickedPoint(point, normal);
    });

    for (QDoubleSpinBox* spin : {
        m_startX, m_startY, m_startZ,
        m_startDirX, m_startDirY, m_startDirZ,
        m_goalX, m_goalY, m_goalZ,
        m_goalDirX, m_goalDirY, m_goalDirZ})
    {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, [this]() {
                NotifyPlanningInputChanged();
            });
    }

    RefreshRestrictedRegionList();
}

QString ToRestrictedRegionSummary(
    const RestrictedRegion& region,
    int index)
{
    return QString("Region %1  P(%2, %3, %4)  N(%5, %6, %7)")
        .arg(index + 1)
        .arg(region.point.x, 0, 'g', 4)
        .arg(region.point.y, 0, 'g', 4)
        .arg(region.point.z, 0, 'g', 4)
        .arg(region.normal.x, 0, 'g', 4)
        .arg(region.normal.y, 0, 'g', 4)
        .arg(region.normal.z, 0, 'g', 4);
}

QString FormatDuration(double milliseconds)
{
    if (milliseconds >= 1000.0)
    {
        return QString("%1 s").arg(milliseconds / 1000.0, 0, 'f', 3);
    }

    return QString("%1 ms").arg(milliseconds, 0, 'f', 2);
}

double TotalProfileDuration(const VoxelPlanningProfile& profile)
{
    return profile.shapeMeshExportMs +
        profile.triangulationMs +
        profile.spatialIndexBuildMs +
        profile.voxelBuildMs +
        profile.astarMs +
        profile.optimizeMs +
        profile.vtkExportMs;
}

void WorkbenchMainWindow::NotifyPlanningInputChanged()
{
    UpdateEndpointOverlay();
    if (m_realtimeCheck->isChecked())
    {
        ComputePath();
    }
}

void WorkbenchMainWindow::AddRestrictedRegion()
{
    RestrictedRegion region;
    m_restrictedRegions.push_back(region);
    if (m_enableRestrictedRegionsCheck != nullptr)
    {
        m_enableRestrictedRegionsCheck->setChecked(true);
    }
    RefreshRestrictedRegionList();
    NotifyPlanningInputChanged();
}

void WorkbenchMainWindow::EditRestrictedRegion(std::size_t index)
{
    if (index >= m_restrictedRegions.size())
    {
        return;
    }

    RestrictedRegion edited = m_restrictedRegions[index];
    QDialog dialog(this);
    dialog.setWindowTitle(QString("Edit Region %1").arg(index + 1));

    QVBoxLayout* dialogLayout = new QVBoxLayout(&dialog);
    QFormLayout* formLayout = new QFormLayout();
    QDoubleSpinBox* pointX = nullptr;
    QDoubleSpinBox* pointY = nullptr;
    QDoubleSpinBox* pointZ = nullptr;
    QDoubleSpinBox* normalX = nullptr;
    QDoubleSpinBox* normalY = nullptr;
    QDoubleSpinBox* normalZ = nullptr;

    formLayout->addRow(
        "Plane point",
        MakeVectorEditor(
            pointX,
            pointY,
            pointZ,
            edited.point.x,
            edited.point.y,
            edited.point.z));
    formLayout->addRow(
        "Blocked normal",
        MakeVectorEditor(
            normalX,
            normalY,
            normalZ,
            edited.normal.x,
            edited.normal.y,
            edited.normal.z));
    dialogLayout->addLayout(formLayout);

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

    edited.point = Vec(pointX->value(), pointY->value(), pointZ->value());
    edited.normal = Vec(normalX->value(), normalY->value(), normalZ->value());
    m_restrictedRegions[index] = edited;
    RefreshRestrictedRegionList();
    NotifyPlanningInputChanged();
}

void WorkbenchMainWindow::RemoveRestrictedRegion(std::size_t index)
{
    if (index >= m_restrictedRegions.size())
    {
        return;
    }

    m_restrictedRegions.erase(m_restrictedRegions.begin() + index);
    RefreshRestrictedRegionList();
    NotifyPlanningInputChanged();
}

void WorkbenchMainWindow::RefreshRestrictedRegionList()
{
    if (m_restrictedRegionListLayout == nullptr)
    {
        return;
    }

    if (m_enableRestrictedRegionsCheck != nullptr)
    {
        const bool hasRegions = !m_restrictedRegions.empty();
        m_enableRestrictedRegionsCheck->setVisible(hasRegions);
        if (!hasRegions)
        {
            QSignalBlocker blocker(m_enableRestrictedRegionsCheck);
            m_enableRestrictedRegionsCheck->setChecked(false);
        }
    }

    while (QLayoutItem* item = m_restrictedRegionListLayout->takeAt(0))
    {
        if (QWidget* widget = item->widget())
        {
            widget->deleteLater();
        }
        delete item;
    }

    for (std::size_t i = 0; i < m_restrictedRegions.size(); ++i)
    {
        QWidget* row = new QWidget();
        QHBoxLayout* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(0, 0, 0, 0);

        QLabel* summary = new QLabel(
            ToRestrictedRegionSummary(
                m_restrictedRegions[i],
                static_cast<int>(i)),
            row);
        summary->setTextInteractionFlags(Qt::TextSelectableByMouse);

        QPushButton* editButton = new QPushButton("Edit", row);
        QPushButton* pickButton = new QPushButton("Pick", row);
        QToolButton* removeButton = new QToolButton(row);
        removeButton->setIcon(style()->standardIcon(QStyle::SP_TrashIcon));
        removeButton->setToolTip("Remove region");

        rowLayout->addWidget(summary, 1);
        rowLayout->addWidget(editButton);
        rowLayout->addWidget(pickButton);
        rowLayout->addWidget(removeButton);
        m_restrictedRegionListLayout->addWidget(row);

        connect(editButton, &QPushButton::clicked, this, [this, i]() {
            EditRestrictedRegion(i);
        });
        connect(pickButton, &QPushButton::clicked, this, [this, i]() {
            BeginRestrictedRegionPointPick(i);
        });
        connect(removeButton, &QToolButton::clicked, this, [this, i]() {
            RemoveRestrictedRegion(i);
        });
    }
}

void WorkbenchMainWindow::ImportModel()
{
    QString startDir;
    QSettings settings(kSettingsOrganization, kSettingsApplication);
    const QString cachedModelDir =
        settings.value(kLastModelDirectoryKey).toString();
    if (QDir(cachedModelDir).exists())
    {
        startDir = QFileInfo(cachedModelDir).absoluteFilePath();
    }

#ifdef MOVEMENTPATH_WORKBENCH_DATA_DIR
    if (startDir.isEmpty())
    {
        const QString dataDir = QString::fromUtf8(MOVEMENTPATH_WORKBENCH_DATA_DIR);
        if (QDir(dataDir).exists())
        {
            startDir = QFileInfo(dataDir).absoluteFilePath();
        }
    }
#endif

    const QString path = QFileDialog::getOpenFileName(
        this,
        "Import model",
        startDir,
        "Models (*.step *.stp *.brep *.vtk)");

    if (path.isEmpty())
    {
        return;
    }

    settings.setValue(
        kLastModelDirectoryKey,
        QFileInfo(path).absolutePath());

    ImportedModel model;
    QString error;
    if (!ModelLoader::Load(path, ReadImportOptions(), model, error))
    {
        AppendLog(error);
        return;
    }

    m_model = model;
    ClearPlanningCaches();
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
    ClearPlanningCaches();
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

void WorkbenchMainWindow::ClearPlanningCaches()
{
    m_cachedPlannerTriangles.clear();
    m_hasCachedPlannerTriangles = false;
    m_cachedTriangleLinearDeflection = 0.0;
    m_cachedTriangleAngularDeflection = 0.0;

    m_cachedFullBoundsVoxelSpace.Clear();
    m_cachedFullBoundsBuildResult = VoxelMeshBuildResult();
    m_hasCachedFullBoundsVoxelSpace = false;
    m_cachedFullBoundsVoxelSize = 0.0;
    m_cachedFullBoundsClearance = 0.0;
}

bool WorkbenchMainWindow::EnsurePlannerTriangleCache()
{
    if (m_model.kind == ImportedModelKind::Unknown)
    {
        return false;
    }

    const double linearDeflection = m_linearDeflectionSpin->value();
    const double angularDeflection = m_angularDeflectionSpin->value();
    const bool cacheMatches =
        m_hasCachedPlannerTriangles &&
        m_cachedTriangleLinearDeflection == linearDeflection &&
        m_cachedTriangleAngularDeflection == angularDeflection;

    if (cacheMatches)
    {
        AppendLog(QString("Using cached model mesh: %1 triangles.")
            .arg(m_cachedPlannerTriangles.size()));
        return true;
    }

    m_cachedPlannerTriangles.clear();
    m_hasCachedPlannerTriangles = false;
    m_hasCachedFullBoundsVoxelSpace = false;
    m_cachedFullBoundsVoxelSpace.Clear();
    m_cachedFullBoundsBuildResult = VoxelMeshBuildResult();

    if (m_model.kind == ImportedModelKind::Mesh)
    {
        m_cachedPlannerTriangles = ToPlannerTriangles(m_model.mesh);
    }
    else
    {
        if (!VoxelMeshBuilder::BuildShapeTriangulation(
                m_model.shape,
                linearDeflection,
                angularDeflection,
                m_cachedPlannerTriangles))
        {
            AppendLog("Build cached model mesh failed.");
            return false;
        }
    }

    if (m_cachedPlannerTriangles.empty())
    {
        AppendLog("Cached model mesh does not contain valid triangles.");
        return false;
    }

    m_cachedTriangleLinearDeflection = linearDeflection;
    m_cachedTriangleAngularDeflection = angularDeflection;
    m_hasCachedPlannerTriangles = true;
    AppendLog(QString("Cached model mesh: %1 triangles.")
        .arg(m_cachedPlannerTriangles.size()));
    return true;
}

bool WorkbenchMainWindow::HasUsableFullBoundsVoxelCache(
    const VoxelPathPlannerOptions& options) const
{
    return m_hasCachedFullBoundsVoxelSpace &&
        m_cachedFullBoundsVoxelSpace.IsValid() &&
        m_cachedFullBoundsBuildResult.success &&
        m_cachedFullBoundsVoxelSize ==
            options.meshBuildOptions.voxelSize &&
        m_cachedFullBoundsClearance ==
            options.meshBuildOptions.clearance;
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

    AppendLog("Path computation preparing.");
    const std::vector<VoxelRestrictedHalfSpace> restrictedHalfSpaces =
        ReadRestrictedHalfSpaces();
    AppendLog(QString("Planner: %1.")
        .arg(m_plannerCombo->currentText())
    );
    AppendLog(QString("Search: %1, neighbors=%2.")
        .arg(m_searchModeCombo->currentText())
        .arg(m_neighborTypeCombo->currentText()));
    AppendLog(QString("Voxel: size=%1, clearance=%2, snap=%3.")
        .arg(m_voxelSizeSpin->value())
        .arg(m_clearanceSpin->value())
        .arg(m_snapRadiusSpin->value()));
    AppendLog(QString("Tuning: heuristic=%1, bounds=%2, turn=%3v, endpoint=%4v/%5, shortcut=%6.")
        .arg(m_pathTuningSettings.heuristicWeight)
        .arg(m_pathTuningSettings.searchBoundsExtraRadius)
        .arg(m_pathTuningSettings.turnPenaltyVoxelMultiplier)
        .arg(m_pathTuningSettings.endpointDirectionPenaltyVoxelMultiplier)
        .arg(m_pathTuningSettings.endpointDirectionRadius)
        .arg(m_pathTuningSettings.optimizerMaxShortcutLookAhead));
    AppendLog(QString("Applied cost: turn=%1, endpoint=%2.")
        .arg(m_voxelSizeSpin->value() *
            m_pathTuningSettings.turnPenaltyVoxelMultiplier)
        .arg(m_voxelSizeSpin->value() *
            m_pathTuningSettings.endpointDirectionPenaltyVoxelMultiplier));
    AppendLog(QString("Smoothing score: sigTurn=%1, totalTurn=%2, maxTurn=%3, detour=%4.")
        .arg(m_pathTuningSettings.smoothingSignificantTurnWeight)
        .arg(m_pathTuningSettings.smoothingTotalTurnWeight)
        .arg(m_pathTuningSettings.smoothingMaxTurnWeight)
        .arg(m_pathTuningSettings.smoothingDetourWeight));
    AppendLog(QString("Regions: %1.")
        .arg(restrictedHalfSpaces.size()));
    AppendLog(QString("Discretization: linear=%1, angular=%2.")
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
    if (!EnsurePlannerTriangleCache())
    {
        return;
    }
    scenario.triangles = m_cachedPlannerTriangles;
    scenario.startPoint = ReadPoint(m_startX, m_startY, m_startZ);
    scenario.startDir = ReadDirection(m_startDirX, m_startDirY, m_startDirZ);
    scenario.goalPoint = ReadPoint(m_goalX, m_goalY, m_goalZ);
    scenario.goalDir = ReadDirection(m_goalDirX, m_goalDirY, m_goalDirZ);

    VoxelPathPlannerOptions options = MakeVoxelOptions(method);
    if (method == PlannerMethod::VoxelFullBounds &&
        HasUsableFullBoundsVoxelCache(options))
    {
        options.useCachedFullBoundsVoxelSpace = true;
        options.cachedFullBoundsVoxelSpace = m_cachedFullBoundsVoxelSpace;
        options.cachedFullBoundsBuildResult = m_cachedFullBoundsBuildResult;
        AppendLog(QString("Using cached FullBounds voxel states: cells=%1.")
            .arg(m_cachedFullBoundsVoxelSpace.CellCount()));
    }
    else if (method == PlannerMethod::VoxelFullBounds)
    {
        AppendLog("FullBounds voxel cache miss; building voxel states.");
    }
    else if (method == PlannerMethod::VoxelLazy)
    {
        AppendLog("Voxel lazy mode does not use FullBounds voxel cache.");
    }
    m_cancelRequested = std::make_shared<std::atomic_bool>(false);
    const std::shared_ptr<std::atomic_bool> cancelRequested =
        m_cancelRequested;
    options.runOptions.shouldCancel = [cancelRequested]()
    {
        return cancelRequested != nullptr &&
            cancelRequested->load(std::memory_order_relaxed);
    };

    m_runningVoxelSize = options.meshBuildOptions.voxelSize;
    m_runningClearance = options.meshBuildOptions.clearance;
    m_runningPlannerMethod = method;
    AppendLog("Path computation started.");
    QApplication::processEvents();

    SetPlanningUiBusy(true);
    m_planTimer.start();

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
        const bool wasAlreadyRequested =
            m_cancelRequested->exchange(true, std::memory_order_relaxed);
        if (!wasAlreadyRequested)
        {
            AppendLog("Stop requested. Waiting for planner to reach a cancellation point...");
        }
        else
        {
            AppendLog("Stop already requested; still waiting for current blocking step to return...");
        }
    }
}

void WorkbenchMainWindow::OpenPathTuningSettings()
{
    QDialog dialog(this);
    dialog.setWindowTitle("Path Settings");

    QVBoxLayout* dialogLayout = new QVBoxLayout(&dialog);

    const PathTuningSettings defaults;

    QGroupBox* astarGroup = new QGroupBox("A* 搜索代价", &dialog);
    QFormLayout* astarLayout = new QFormLayout(astarGroup);
    QDoubleSpinBox* heuristicWeight = MakeTuningDoubleSpin(
        m_pathTuningSettings.heuristicWeight,
        0.0,
        1000.0);
    SetDefaultTooltip(
        heuristicWeight,
        "heuristicWeight",
        "启发式距离权重。值越大越偏向直奔终点，值过大可能牺牲最优性。",
        defaults.heuristicWeight);
    QSpinBox* searchBoundsExtraRadius = MakeTuningIntSpin(
        m_pathTuningSettings.searchBoundsExtraRadius,
        0,
        1000000);
    SetDefaultTooltip(
        searchBoundsExtraRadius,
        "searchBoundsExtraRadius",
        "在模型包围盒和起终点外额外扩展的搜索体素半径。",
        defaults.searchBoundsExtraRadius);
    QDoubleSpinBox* turnPenaltyMultiplier = MakeTuningDoubleSpin(
        m_pathTuningSettings.turnPenaltyVoxelMultiplier,
        0.0,
        1000.0);
    SetDefaultTooltip(
        turnPenaltyMultiplier,
        "turnPenaltyVoxelMultiplier",
        "A* 转弯惩罚系数，最终惩罚为该值乘以体素尺寸。",
        defaults.turnPenaltyVoxelMultiplier);
    QDoubleSpinBox* endpointPenaltyMultiplier = MakeTuningDoubleSpin(
        m_pathTuningSettings.endpointDirectionPenaltyVoxelMultiplier,
        0.0,
        1000.0);
    SetDefaultTooltip(
        endpointPenaltyMultiplier,
        "endpointDirectionPenaltyVoxelMultiplier",
        "起点/终点附近方向偏离惩罚系数，最终惩罚为该值乘以体素尺寸。",
        defaults.endpointDirectionPenaltyVoxelMultiplier);
    QSpinBox* endpointDirectionRadius = MakeTuningIntSpin(
        m_pathTuningSettings.endpointDirectionRadius,
        0,
        1000000);
    SetDefaultTooltip(
        endpointDirectionRadius,
        "endpointDirectionRadius",
        "起点/终点方向约束影响的体素半径。",
        defaults.endpointDirectionRadius);
    astarLayout->addRow("启发式权重", heuristicWeight);
    astarLayout->addRow("搜索边界额外半径", searchBoundsExtraRadius);
    astarLayout->addRow("转弯惩罚 x 体素", turnPenaltyMultiplier);
    astarLayout->addRow(
        "端点方向惩罚 x 体素",
        endpointPenaltyMultiplier);
    astarLayout->addRow("端点方向影响半径", endpointDirectionRadius);
    dialogLayout->addWidget(astarGroup);

    QGroupBox* shortcutGroup = new QGroupBox("拉直与平滑", &dialog);
    QFormLayout* shortcutLayout = new QFormLayout(shortcutGroup);
    QSpinBox* maxShortcutLookAhead = MakeTuningIntSpin(
        m_pathTuningSettings.optimizerMaxShortcutLookAhead,
        0,
        1000000);
    SetDefaultTooltip(
        maxShortcutLookAhead,
        "optimizerMaxShortcutLookAhead",
        "Line-of-sight 拉直时向前尝试跳过的最大路径点数，0 表示不限。",
        defaults.optimizerMaxShortcutLookAhead);
    QSpinBox* smoothSamplesPerSegment = MakeTuningIntSpin(
        m_pathTuningSettings.smoothPathSamplesPerSegment,
        0,
        1000000);
    SetDefaultTooltip(
        smoothSamplesPerSegment,
        "smoothPathSamplesPerSegment",
        "曲线平滑每段至少采样的点数，0 会关闭曲线采样。",
        defaults.smoothPathSamplesPerSegment);
    QSpinBox* displaySamplesPerSegment = MakeTuningIntSpin(
        m_pathTuningSettings.displayPathSamplesPerSegment,
        1,
        1000000);
    SetDefaultTooltip(
        displaySamplesPerSegment,
        "displayPathSamplesPerSegment",
        "displayPointPath samples per smoothed control segment. Only affects Workbench visual density.",
        defaults.displayPathSamplesPerSegment);
    QDoubleSpinBox* smoothSpacingMin = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothPathSampleSpacingMin,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        smoothSpacingMin,
        "smoothPathSampleSpacingMin",
        "曲线采样间距下限。",
        defaults.smoothPathSampleSpacingMin);
    QDoubleSpinBox* smoothSpacingVoxelMultiplier = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothPathSampleSpacingVoxelMultiplier,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        smoothSpacingVoxelMultiplier,
        "smoothPathSampleSpacingVoxelMultiplier",
        "曲线采样间距按体素尺寸计算的倍率。",
        defaults.smoothPathSampleSpacingVoxelMultiplier);
    QDoubleSpinBox* maxDeviationVoxelMultiplier = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothPathMaxDeviationVoxelMultiplier,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        maxDeviationVoxelMultiplier,
        "smoothPathMaxDeviationVoxelMultiplier",
        "平滑曲线允许偏离控制折线的距离，按体素尺寸计算的倍率。",
        defaults.smoothPathMaxDeviationVoxelMultiplier);
    QDoubleSpinBox* maxDeviationClearanceMultiplier = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothPathMaxDeviationClearanceMultiplier,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        maxDeviationClearanceMultiplier,
        "smoothPathMaxDeviationClearanceMultiplier",
        "平滑曲线允许偏离控制折线的距离，按安全距离计算的倍率。",
        defaults.smoothPathMaxDeviationClearanceMultiplier);
    shortcutLayout->addRow("最大拉直前视点数", maxShortcutLookAhead);
    shortcutLayout->addRow("每段采样点数", smoothSamplesPerSegment);
    shortcutLayout->addRow("采样间距下限", smoothSpacingMin);
    shortcutLayout->addRow(
        "采样间距 x 体素",
        smoothSpacingVoxelMultiplier);
    shortcutLayout->insertRow(
        2,
        "displayPointPath samples",
        displaySamplesPerSegment);
    shortcutLayout->addRow(
        "最大偏离 x 体素",
        maxDeviationVoxelMultiplier);
    shortcutLayout->addRow(
        "最大偏离 x 安全距离",
        maxDeviationClearanceMultiplier);
    dialogLayout->addWidget(shortcutGroup);

    QGroupBox* scoreGroup = new QGroupBox("平滑评分权重", &dialog);
    QFormLayout* scoreLayout = new QFormLayout(scoreGroup);
    QDoubleSpinBox* significantTurnWeight = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothingSignificantTurnWeight,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        significantTurnWeight,
        "smoothingSignificantTurnWeight",
        "明显转弯数量的评分权重，值越大越倾向少拐弯。",
        defaults.smoothingSignificantTurnWeight);
    QDoubleSpinBox* totalTurnWeight = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothingTotalTurnWeight,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        totalTurnWeight,
        "smoothingTotalTurnWeight",
        "总转向严重度的评分权重，值越大越倾向整体更顺。",
        defaults.smoothingTotalTurnWeight);
    QDoubleSpinBox* maxTurnWeight = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothingMaxTurnWeight,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        maxTurnWeight,
        "smoothingMaxTurnWeight",
        "最大单处转向严重度的评分权重，值越大越抑制局部急弯。",
        defaults.smoothingMaxTurnWeight);
    QDoubleSpinBox* detourWeight = MakeTuningDoubleSpin(
        m_pathTuningSettings.smoothingDetourWeight,
        0.0,
        1000000.0);
    SetDefaultTooltip(
        detourWeight,
        "smoothingDetourWeight",
        "绕行率评分权重，值越大越倾向短路径。",
        defaults.smoothingDetourWeight);
    scoreLayout->addRow("明显转弯权重", significantTurnWeight);
    scoreLayout->addRow("总转向权重", totalTurnWeight);
    scoreLayout->addRow("最大转向权重", maxTurnWeight);
    scoreLayout->addRow("绕行权重", detourWeight);
    dialogLayout->addWidget(scoreGroup);

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

    m_pathTuningSettings.heuristicWeight = heuristicWeight->value();
    m_pathTuningSettings.searchBoundsExtraRadius =
        searchBoundsExtraRadius->value();
    m_pathTuningSettings.turnPenaltyVoxelMultiplier =
        turnPenaltyMultiplier->value();
    m_pathTuningSettings.endpointDirectionPenaltyVoxelMultiplier =
        endpointPenaltyMultiplier->value();
    m_pathTuningSettings.endpointDirectionRadius =
        endpointDirectionRadius->value();
    m_pathTuningSettings.optimizerMaxShortcutLookAhead =
        maxShortcutLookAhead->value();
    m_pathTuningSettings.smoothPathSamplesPerSegment =
        smoothSamplesPerSegment->value();
    m_pathTuningSettings.displayPathSamplesPerSegment =
        displaySamplesPerSegment->value();
    m_pathTuningSettings.smoothPathSampleSpacingMin =
        smoothSpacingMin->value();
    m_pathTuningSettings.smoothPathSampleSpacingVoxelMultiplier =
        smoothSpacingVoxelMultiplier->value();
    m_pathTuningSettings.smoothPathMaxDeviationVoxelMultiplier =
        maxDeviationVoxelMultiplier->value();
    m_pathTuningSettings.smoothPathMaxDeviationClearanceMultiplier =
        maxDeviationClearanceMultiplier->value();
    m_pathTuningSettings.smoothingSignificantTurnWeight =
        significantTurnWeight->value();
    m_pathTuningSettings.smoothingTotalTurnWeight =
        totalTurnWeight->value();
    m_pathTuningSettings.smoothingMaxTurnWeight =
        maxTurnWeight->value();
    m_pathTuningSettings.smoothingDetourWeight =
        detourWeight->value();

    AppendLog("Path settings updated.");
    NotifyPlanningInputChanged();
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
    const double wallMs =
        m_planTimer.isValid() ? static_cast<double>(m_planTimer.elapsed()) : 0.0;
    AppendLog("Path computation finished.");

    if (wasCancelled)
    {
        AppendLog("Path computation was cancelled.");
        if (wallMs > 0.0)
        {
            AppendLog(QString("Time: wall=%1.")
                .arg(FormatDuration(wallMs)));
        }
        return;
    }

    AppendLog(QString("Result: %1.")
        .arg(result.success ? "succeeded" : "failed"));
    if (m_runningPlannerMethod == PlannerMethod::VoxelFullBounds &&
        result.hasReusableFullBoundsVoxelSpace)
    {
        m_cachedFullBoundsVoxelSpace =
            result.reusableFullBoundsVoxelSpace;
        m_cachedFullBoundsBuildResult =
            result.reusableFullBoundsBuildResult;
        m_cachedFullBoundsVoxelSize =
            m_runningVoxelSize;
        m_cachedFullBoundsClearance =
            m_runningClearance;
        m_hasCachedFullBoundsVoxelSpace = true;
        AppendLog(QString("Cached FullBounds voxel states: cells=%1.")
            .arg(m_cachedFullBoundsVoxelSpace.CellCount()));
    }
    AppendLog(QString("Mode: %1.")
        .arg(QString::fromStdString(result.profile.buildRegionMode)));
    AppendLog(QString("Cost: %1.")
        .arg(result.profile.totalCost, 0, 'f', 3));
    AppendLog(QString("Mesh: triangles=%1, cells=%2.")
        .arg(result.profile.triangleCount)
        .arg(result.profile.storedCellCount));
    AppendLog(QString("Voxels: occupied=%1, safety=%2.")
        .arg(result.profile.occupiedCount)
        .arg(result.profile.clearanceBandCount));
    AppendLog(QString("Path: raw=%1, optimized=%2.")
        .arg(result.profile.rawPathCount)
        .arg(result.profile.optimizedPathCount));
    AppendLog(QString("Raw quality: length=%1, detour=%2%, turns=%3, totalTurn=%4, maxTurn=%5.")
        .arg(result.profile.rawPathLength, 0, 'f', 3)
        .arg(result.profile.rawPathDetourRatio * 100.0, 0, 'f', 2)
        .arg(result.profile.rawPathTurnCount)
        .arg(result.profile.rawPathTotalTurnSeverity, 0, 'f', 3)
        .arg(result.profile.rawPathMaxTurnSeverity, 0, 'f', 3));
    AppendLog(QString("Final quality: length=%1, detour=%2%, turns=%3, totalTurn=%4, maxTurn=%5.")
        .arg(result.profile.finalPathLength, 0, 'f', 3)
        .arg(result.profile.finalPathDetourRatio * 100.0, 0, 'f', 2)
        .arg(result.profile.finalPathTurnCount)
        .arg(result.profile.finalPathTotalTurnSeverity, 0, 'f', 3)
        .arg(result.profile.finalPathMaxTurnSeverity, 0, 'f', 3));
    AppendLog(QString("Endpoint alignment: raw start=%1, raw goal=%2, final start=%3, final goal=%4.")
        .arg(result.profile.rawStartDirectionAlignment, 0, 'f', 3)
        .arg(result.profile.rawGoalDirectionAlignment, 0, 'f', 3)
        .arg(result.profile.finalStartDirectionAlignment, 0, 'f', 3)
        .arg(result.profile.finalGoalDirectionAlignment, 0, 'f', 3));

    if (result.profile.smoothingRequested &&
        result.profile.optimizedPathCount > 2 &&
        !result.profile.smoothingSucceeded)
    {
        AppendLog("Smoothing: rejected by voxel constraints.");
    }
    else if (result.profile.smoothingRequested)
    {
        AppendLog(QString("Smoothing: %1, points=%2.")
            .arg(result.profile.smoothingSucceeded ? "accepted" : "not applied")
            .arg(result.profile.smoothedPathPointCount));
    }

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

    AppendLog(QString("Timing: wall=%1.")
        .arg(FormatDuration(wallMs)));
    AppendLog(QString("Timing: measured modules=%1.")
        .arg(FormatDuration(TotalProfileDuration(result.profile))));
    if (result.profile.shapeMeshExportMs > 0.0)
    {
        AppendLog(QString("Timing: shape mesh export=%1.")
            .arg(FormatDuration(result.profile.shapeMeshExportMs)));
    }
    AppendLog(QString("Timing: triangulation=%1.")
        .arg(FormatDuration(result.profile.triangulationMs)));
    AppendLog(QString("Timing: spatial index=%1.")
        .arg(FormatDuration(result.profile.spatialIndexBuildMs)));
    AppendLog(QString("Timing: voxel build=%1.")
        .arg(FormatDuration(result.profile.voxelBuildMs)));
    if (result.profile.lazyBuildEnabled)
    {
        AppendLog(QString("Timing: lazy query=%1.")
            .arg(FormatDuration(result.profile.lazyVoxelQueryMs)));
        AppendLog(QString("Timing: lazy candidate query=%1.")
            .arg(FormatDuration(result.profile.lazyCandidateQueryMs)));
        AppendLog(QString("Timing: lazy candidate filter=%1.")
            .arg(FormatDuration(result.profile.lazyCandidateFilterMs)));
        AppendLog(QString("Timing: lazy voxel evaluation=%1.")
            .arg(FormatDuration(result.profile.lazyVoxelMarkMs)));
    }
    AppendLog(QString("Timing: A* search=%1.")
        .arg(FormatDuration(result.profile.astarMs)));
    AppendLog(QString("Timing: optimization=%1.")
        .arg(FormatDuration(result.profile.optimizeMs)));
    if (result.profile.vtkExportMs > 0.0)
    {
        AppendLog(QString("Timing: VTK export=%1.")
            .arg(FormatDuration(result.profile.vtkExportMs)));
    }

    if (!result.success)
    {
        return;
    }

    const std::vector<Vec> pathPoints =
        !result.displayPathPoints.empty() ?
            ToPathPoints(result.displayPathPoints) :
            ToPathPoints(result.optimizeResult.pointPath);
    m_view->DisplayPath(pathPoints);

    if (m_showPathVoxelsCheck->isChecked())
    {
        m_view->DisplayVoxelBoxes(
            result.pathFreeVoxelCenters,
            m_runningVoxelSize,
            Quantity_Color(0.05, 0.45, 1.0, Quantity_TOC_RGB));
        m_view->DisplayVoxelBoxes(
            result.pathClearanceVoxelCenters,
            m_runningVoxelSize,
            Quantity_Color(0.0, 0.85, 0.65, Quantity_TOC_RGB));
        m_view->DisplayVoxelBoxes(
            result.pathOccupiedVoxelCenters,
            m_runningVoxelSize,
            Quantity_Color(0.95, 0.15, 0.05, Quantity_TOC_RGB));
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

void WorkbenchMainWindow::BeginRestrictedRegionPointPick(std::size_t index)
{
    if (index >= m_restrictedRegions.size())
    {
        return;
    }

    m_pickRestrictedRegionIndex = index;
    BeginPick(PointPickMode::RestrictedRegionPoint);
}

void WorkbenchMainWindow::ApplyPickedPoint(
    const gp_Pnt& point,
    const gp_Vec& normal)
{
    switch (m_pickMode)
    {
    case PointPickMode::StartPoint:
        m_startX->setValue(point.X());
        m_startY->setValue(point.Y());
        m_startZ->setValue(point.Z());
        SetDirectionSpins(m_startDirX, m_startDirY, m_startDirZ, normal);
        break;
    case PointPickMode::GoalPoint:
        m_goalX->setValue(point.X());
        m_goalY->setValue(point.Y());
        m_goalZ->setValue(point.Z());
        SetDirectionSpins(m_goalDirX, m_goalDirY, m_goalDirZ, normal);
        break;
    case PointPickMode::RestrictedRegionPoint:
        if (m_pickRestrictedRegionIndex >= m_restrictedRegions.size())
        {
            m_pickMode = PointPickMode::None;
            return;
        }
        m_restrictedRegions[m_pickRestrictedRegionIndex].point =
            Vec(point.X(), point.Y(), point.Z());
        RefreshRestrictedRegionList();
        NotifyPlanningInputChanged();
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
    if (event->key() == Qt::Key_Space)
    {
        m_view->ResetToInitialView();
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

std::vector<VoxelRestrictedHalfSpace>
WorkbenchMainWindow::ReadRestrictedHalfSpaces() const
{
    std::vector<VoxelRestrictedHalfSpace> halfSpaces;

    if (m_enableRestrictedRegionsCheck == nullptr ||
        !m_enableRestrictedRegionsCheck->isChecked())
    {
        return halfSpaces;
    }

    for (const RestrictedRegion& region : m_restrictedRegions)
    {
        VoxelRestrictedHalfSpace halfSpace;
        halfSpace.point = region.point;
        halfSpace.normal = region.normal;

        if (halfSpace.normal.SquareMagnitude() <= 1.0e-20)
        {
            continue;
        }

        halfSpace.normal.Normalize();
        VoxelWalkability::CachePointNormalDot(halfSpace);
        halfSpaces.push_back(halfSpace);
    }

    return halfSpaces;
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
    options.astarOptions.heuristicWeight =
        m_pathTuningSettings.heuristicWeight;
    options.searchBoundsExtraRadius =
        m_pathTuningSettings.searchBoundsExtraRadius;
    options.astarOptions.turnPenalty =
        std::max(
            0.0,
            m_voxelSizeSpin->value() *
                m_pathTuningSettings.turnPenaltyVoxelMultiplier);
    options.astarOptions.endpointDirectionPenalty =
        std::max(
            0.0,
            m_voxelSizeSpin->value() *
                m_pathTuningSettings
                    .endpointDirectionPenaltyVoxelMultiplier);
    options.astarOptions.endpointDirectionRadius =
        m_pathTuningSettings.endpointDirectionRadius;
    options.optimizerMaxShortcutLookAhead =
        m_pathTuningSettings.optimizerMaxShortcutLookAhead;
    options.restrictedHalfSpaces = ReadRestrictedHalfSpaces();
    options.smoothOptimizedPath =
        m_smoothPathCheck != nullptr && m_smoothPathCheck->isChecked();
    options.smoothPathSamplesPerSegment =
        m_pathTuningSettings.smoothPathSamplesPerSegment;
    options.displayPathSamplesPerSegment =
        m_pathTuningSettings.displayPathSamplesPerSegment;
    options.smoothPathSampleSpacing =
        std::max(
            m_pathTuningSettings.smoothPathSampleSpacingMin,
            m_voxelSizeSpin->value() *
                m_pathTuningSettings.smoothPathSampleSpacingVoxelMultiplier);
    options.smoothPathMaxDeviation =
        std::max(
            m_voxelSizeSpin->value() *
                m_pathTuningSettings.smoothPathMaxDeviationVoxelMultiplier,
            m_clearanceSpin->value() *
                m_pathTuningSettings
                    .smoothPathMaxDeviationClearanceMultiplier);
    options.smoothingSignificantTurnWeight =
        m_pathTuningSettings.smoothingSignificantTurnWeight;
    options.smoothingTotalTurnWeight =
        m_pathTuningSettings.smoothingTotalTurnWeight;
    options.smoothingMaxTurnWeight =
        m_pathTuningSettings.smoothingMaxTurnWeight;
    options.smoothingDetourWeight =
        m_pathTuningSettings.smoothingDetourWeight;

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
    options.runOptions.debugNeighborhood = false;
    options.runOptions.verbose = false;
    options.runOptions.collectVoxelOverlayCenters =
        m_showPathVoxelsCheck != nullptr &&
        m_showPathVoxelsCheck->isChecked();
    options.runOptions.maxVoxelOverlayCentersPerState = 0;

    if (method == PlannerMethod::VoxelLazy)
    {
        options.lazyBuildOptions.enabled = true;
        options.lazyBuildOptions.timeBudgetMs =
            m_lazyTimeBudgetSpin->value();
    }
    else
    {
        options.lazyBuildOptions.enabled = false;
    }

    return options;
}

} // namespace path_planning_workbench
