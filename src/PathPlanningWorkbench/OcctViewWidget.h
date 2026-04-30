#pragma once

#include "ModelLoader.h"
#include "VoxelPathPlanner.h"

#include <AIS_InteractiveContext.hxx>
#include <AIS_Shape.hxx>
#include <AIS_Triangulation.hxx>
#include <Aspect_DisplayConnection.hxx>
#include <OpenGl_GraphicDriver.hxx>
#include <TopoDS_Shape.hxx>
#include <V3d_View.hxx>
#include <V3d_Viewer.hxx>

#include <QWidget>

#include <functional>
#include <vector>

namespace path_planning_workbench
{

enum class ViewDisplayMode
{
    Shaded,
    Wireframe,
    Mesh
};

class OcctViewWidget final : public QWidget
{
public:
    explicit OcctViewWidget(QWidget* parent = nullptr);

    void DisplayShape(
        const TopoDS_Shape& shape,
        ViewDisplayMode mode);

    void DisplayMesh(
        const TriangleMeshData& mesh,
        ViewDisplayMode mode);

    void SetDisplayMode(ViewDisplayMode mode);

    void DisplayEndpoints(
        const gp_Pnt& startPoint,
        const gp_Vec& startDir,
        const gp_Pnt& goalPoint,
        const gp_Vec& goalDir);

    void DisplayPath(
        const std::vector<Vec>& points);

    void DisplayKeyVoxels(
        const std::vector<Vec>& voxelCenters,
        double voxelSize);

    void ClearOverlays();

    void SetPointPickCallback(
        std::function<void(const gp_Pnt&)> callback);

protected:
    QPaintEngine* paintEngine() const override;
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    void EnsureViewer();
    void RedisplayModel(bool fitView);
    void DisplayTransientShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color,
        double width = 1.0);
    void DisplayEndpointShape(
        const TopoDS_Shape& shape,
        const Quantity_Color& color,
        double width = 1.0);
    void DisplayWorldAxes();
    void ClearEndpointOverlays();
    void ClearWorldAxes();
    TopoDS_Shape BuildMeshEdgeShape() const;
    TopoDS_Shape BuildTriangleMeshEdgeShape() const;
    bool PickModelPoint(
        const QPoint& screenPoint,
        gp_Pnt& outPoint) const;

    Handle(Aspect_DisplayConnection) m_displayConnection;
    Handle(OpenGl_GraphicDriver) m_graphicDriver;
    Handle(V3d_Viewer) m_viewer;
    Handle(V3d_View) m_view;
    Handle(AIS_InteractiveContext) m_context;

    TopoDS_Shape m_modelShape;
    TopoDS_Shape m_cachedMeshEdgeShape;
    TriangleMeshData m_meshData;
    Handle(Poly_Triangulation) m_meshTriangulation;
    Handle(AIS_Shape) m_modelPresentation;
    Handle(AIS_Shape) m_meshPresentation;
    Handle(AIS_Triangulation) m_meshTriangulationPresentation;
    std::vector<Handle(AIS_Shape)> m_overlays;
    std::vector<Handle(AIS_Shape)> m_endpointOverlays;
    std::vector<Handle(AIS_Shape)> m_worldAxisOverlays;
    std::function<void(const gp_Pnt&)> m_pickCallback;
    ViewDisplayMode m_displayMode = ViewDisplayMode::Shaded;
    QPoint m_lastMousePosition;
};

} // namespace path_planning_workbench
