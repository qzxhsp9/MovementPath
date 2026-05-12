#include "OcctViewWidget.h"

#include <AIS_DisplayMode.hxx>
#include <AIS_Triangulation.hxx>
#include <BRep_Builder.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRep_Tool.hxx>
#include <Graphic3d_GraphicDriver.hxx>
#include <Poly_Triangle.hxx>
#include <Poly_Triangulation.hxx>
#include <Prs3d_LineAspect.hxx>
#include <Quantity_Color.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <V3d_TypeOfOrientation.hxx>
#include <WNT_Window.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <QMouseEvent>
#include <QPaintEngine>
#include <QResizeEvent>
#include <QWheelEvent>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <utility>

namespace path_planning_workbench
{
namespace
{
gp_Pnt ToPoint(const Vec& point)
{
    return gp_Pnt(point.x, point.y, point.z);
}

gp_Pnt ComputeMeshCentroid(const TriangleMeshData& mesh)
{
    if (mesh.points.empty())
    {
        return gp_Pnt(0.0, 0.0, 0.0);
    }

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    for (const gp_Pnt& point : mesh.points)
    {
        x += point.X();
        y += point.Y();
        z += point.Z();
    }

    const double count = static_cast<double>(mesh.points.size());
    return gp_Pnt(x / count, y / count, z / count);
}

TopoDS_Shape MakePolylineShape(const std::vector<Vec>& points)
{
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    for (std::size_t i = 1; i < points.size(); ++i)
    {
        BRepBuilderAPI_MakeEdge edge(ToPoint(points[i - 1]), ToPoint(points[i]));
        if (edge.IsDone())
        {
            builder.Add(compound, edge.Edge());
        }
    }

    return compound;
}

TopoDS_Shape MakeDirectionShape(
    const gp_Pnt& point,
    const gp_Vec& direction)
{
    gp_Vec dir = direction;
    if (dir.SquareMagnitude() <= 1.0e-12)
    {
        dir = gp_Vec(1.0, 0.0, 0.0);
    }
    dir.Normalize();
    dir *= 10.0;

    BRepBuilderAPI_MakeEdge edge(point, point.Translated(dir));
    return edge.IsDone() ? TopoDS_Shape(edge.Edge()) : TopoDS_Shape();
}

bool IntersectRayTriangle(
    const gp_Pnt& origin,
    const gp_Vec& direction,
    const gp_Pnt& a,
    const gp_Pnt& b,
    const gp_Pnt& c,
    double& outT)
{
    constexpr double epsilon = 1.0e-10;
    const gp_Vec edge1(a, b);
    const gp_Vec edge2(a, c);
    const gp_Vec pvec = direction.Crossed(edge2);
    const double det = edge1.Dot(pvec);

    if (std::abs(det) <= epsilon)
    {
        return false;
    }

    const double invDet = 1.0 / det;
    const gp_Vec tvec(a, origin);
    const double u = tvec.Dot(pvec) * invDet;
    if (u < 0.0 || u > 1.0)
    {
        return false;
    }

    const gp_Vec qvec = tvec.Crossed(edge1);
    const double v = direction.Dot(qvec) * invDet;
    if (v < 0.0 || u + v > 1.0)
    {
        return false;
    }

    const double t = edge2.Dot(qvec) * invDet;
    if (t <= epsilon)
    {
        return false;
    }

    outT = t;
    return true;
}

Handle(Poly_Triangulation) ToPolyTriangulation(
    const TriangleMeshData& mesh)
{
    if (mesh.IsEmpty())
    {
        return nullptr;
    }

    const gp_Pnt centroid = ComputeMeshCentroid(mesh);
    Handle(Poly_Triangulation) triangulation =
        new Poly_Triangulation(
            static_cast<Standard_Integer>(mesh.triangles.size() * 3),
            static_cast<Standard_Integer>(mesh.triangles.size()),
            false);

    for (std::size_t i = 0; i < mesh.triangles.size(); ++i)
    {
        const std::array<int, 3>& tri = mesh.triangles[i];
        gp_Pnt p0 = mesh.points[static_cast<std::size_t>(tri[0])];
        gp_Pnt p1 = mesh.points[static_cast<std::size_t>(tri[1])];
        gp_Pnt p2 = mesh.points[static_cast<std::size_t>(tri[2])];

        const gp_Vec edge01(p0, p1);
        const gp_Vec edge02(p0, p2);
        const gp_Vec normal = edge01.Crossed(edge02);
        const gp_Pnt triangleCenter(
            (p0.X() + p1.X() + p2.X()) / 3.0,
            (p0.Y() + p1.Y() + p2.Y()) / 3.0,
            (p0.Z() + p1.Z() + p2.Z()) / 3.0);

        if (normal.SquareMagnitude() > 1.0e-20 &&
            normal.Dot(gp_Vec(centroid, triangleCenter)) < 0.0)
        {
            std::swap(p1, p2);
        }

        const Standard_Integer n0 =
            static_cast<Standard_Integer>(i * 3 + 1);
        const Standard_Integer n1 =
            static_cast<Standard_Integer>(i * 3 + 2);
        const Standard_Integer n2 =
            static_cast<Standard_Integer>(i * 3 + 3);

        triangulation->SetNode(n0, p0);
        triangulation->SetNode(n1, p1);
        triangulation->SetNode(n2, p2);
        triangulation->SetTriangle(
            static_cast<Standard_Integer>(i + 1),
            Poly_Triangle(n0, n1, n2));
    }

    triangulation->ComputeNormals();
    return triangulation;
}

std::uint64_t MakeEdgeKey(int first, int second)
{
    if (first > second)
    {
        std::swap(first, second);
    }

    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(first)) << 32) |
        static_cast<std::uint32_t>(second);
}

void AddUniqueMeshEdge(
    const TriangleMeshData& mesh,
    int first,
    int second,
    std::unordered_set<std::uint64_t>& seenEdges,
    BRep_Builder& builder,
    TopoDS_Compound& compound)
{
    if (first < 0 ||
        second < 0 ||
        static_cast<std::size_t>(first) >= mesh.points.size() ||
        static_cast<std::size_t>(second) >= mesh.points.size())
    {
        return;
    }

    const std::uint64_t key = MakeEdgeKey(first, second);
    if (!seenEdges.insert(key).second)
    {
        return;
    }

    BRepBuilderAPI_MakeEdge edge(
        mesh.points[static_cast<std::size_t>(first)],
        mesh.points[static_cast<std::size_t>(second)]);
    if (edge.IsDone())
    {
        builder.Add(compound, edge.Edge());
    }
}
}

OcctViewWidget::OcctViewWidget(QWidget* parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_PaintOnScreen);
    setAttribute(Qt::WA_NoSystemBackground);
    setMouseTracking(true);
}

void OcctViewWidget::DisplayShape(
    const TopoDS_Shape& shape,
    ViewDisplayMode mode)
{
    EnsureViewer();
    m_modelShape = shape;
    m_cachedMeshEdgeShape.Nullify();
    m_meshData = TriangleMeshData();
    m_meshTriangulation.Nullify();
    m_displayMode = mode;
    RedisplayModel(true);
}

void OcctViewWidget::DisplayMesh(
    const TriangleMeshData& mesh,
    ViewDisplayMode mode)
{
    EnsureViewer();
    m_modelShape.Nullify();
    m_cachedMeshEdgeShape.Nullify();
    m_meshData = mesh;
    m_meshTriangulation = ToPolyTriangulation(m_meshData);
    m_displayMode = mode;
    RedisplayModel(true);
}

void OcctViewWidget::SetDisplayMode(ViewDisplayMode mode)
{
    m_displayMode = mode;
    RedisplayModel(false);
}

void OcctViewWidget::DisplayEndpoints(
    const gp_Pnt& startPoint,
    const gp_Vec& startDir,
    const gp_Pnt& goalPoint,
    const gp_Vec& goalDir)
{
    EnsureViewer();
    ClearEndpointOverlays();

    DisplayEndpointShape(
        BRepPrimAPI_MakeSphere(startPoint, 2.0).Shape(),
        Quantity_Color(0.1, 0.8, 0.2, Quantity_TOC_RGB));
    DisplayEndpointShape(
        MakeDirectionShape(startPoint, startDir),
        Quantity_Color(0.1, 0.8, 0.2, Quantity_TOC_RGB),
        3.0);

    DisplayEndpointShape(
        BRepPrimAPI_MakeSphere(goalPoint, 2.0).Shape(),
        Quantity_Color(0.9, 0.2, 0.1, Quantity_TOC_RGB));
    DisplayEndpointShape(
        MakeDirectionShape(goalPoint, goalDir),
        Quantity_Color(0.9, 0.2, 0.1, Quantity_TOC_RGB),
        3.0);
    m_view->Redraw();
}

void OcctViewWidget::DisplayPath(
    const std::vector<Vec>& points)
{
    if (points.size() < 2)
    {
        return;
    }

    EnsureViewer();
    DisplayTransientShape(
        MakePolylineShape(points),
        Quantity_Color(1.0, 0.8, 0.05, Quantity_TOC_RGB),
        4.0);
}

void OcctViewWidget::DisplayKeyVoxels(
    const std::vector<Vec>& voxelCenters,
    double voxelSize)
{
    DisplayVoxelBoxes(
        voxelCenters,
        voxelSize,
        Quantity_Color(0.05, 0.45, 1.0, Quantity_TOC_RGB));
}

void OcctViewWidget::DisplayVoxelBoxes(
    const std::vector<Vec>& voxelCenters,
    double voxelSize,
    const Quantity_Color& color)
{
    EnsureViewer();

    const double half = voxelSize * 0.5;

    for (std::size_t i = 0; i < voxelCenters.size(); ++i)
    {
        const Vec& c = voxelCenters[i];
        const gp_Pnt minPoint(c.x - half, c.y - half, c.z - half);
        BRepPrimAPI_MakeBox box(
            minPoint,
            voxelSize,
            voxelSize,
            voxelSize);
        DisplayTransientShape(box.Shape(), color);
    }
}

void OcctViewWidget::ClearOverlays()
{
    EnsureViewer();

    for (const Handle(AIS_Shape)& overlay : m_overlays)
    {
        m_context->Remove(overlay, Standard_False);
    }
    m_overlays.clear();
    ClearEndpointOverlays();
    m_view->Redraw();
}

void OcctViewWidget::ResetToInitialView()
{
    EnsureViewer();

    if (m_view.IsNull())
    {
        return;
    }

    m_view->SetProj(V3d_TypeOfOrientation_Zup_AxoRight);
    m_view->FitAll();
    m_view->Redraw();
}

void OcctViewWidget::SetPointPickCallback(
    std::function<void(const gp_Pnt&)> callback)
{
    m_pickCallback = std::move(callback);
}

QPaintEngine* OcctViewWidget::paintEngine() const
{
    return nullptr;
}

void OcctViewWidget::paintEvent(QPaintEvent*)
{
    EnsureViewer();
    m_view->Redraw();
}

void OcctViewWidget::resizeEvent(QResizeEvent*)
{
    if (!m_view.IsNull())
    {
        m_view->MustBeResized();
    }
}

void OcctViewWidget::showEvent(QShowEvent*)
{
    EnsureViewer();
}

void OcctViewWidget::mousePressEvent(QMouseEvent* event)
{
    EnsureViewer();
    m_lastMousePosition = event->pos();

    if (event->button() == Qt::LeftButton)
    {
        m_view->StartRotation(event->pos().x(), event->pos().y());
    }
}

void OcctViewWidget::mouseMoveEvent(QMouseEvent* event)
{
    EnsureViewer();

    const QPoint current = event->pos();
    if (event->buttons() & Qt::LeftButton)
    {
        m_view->Rotation(
            current.x(),
            current.y());
    }
    else if (event->buttons() & Qt::MiddleButton)
    {
        m_view->Pan(
            current.x() - m_lastMousePosition.x(),
            m_lastMousePosition.y() - current.y());
    }

    m_lastMousePosition = current;
}

void OcctViewWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    EnsureViewer();

    if (!m_pickCallback)
    {
        return;
    }

    gp_Pnt pickedPoint;
    if (PickModelPoint(event->pos(), pickedPoint))
    {
        m_pickCallback(pickedPoint);
    }
}

void OcctViewWidget::wheelEvent(QWheelEvent* event)
{
    EnsureViewer();
    const int delta = event->angleDelta().y();

    if (delta != 0)
    {
        const QPoint position = event->position().toPoint();
        const int zoomDelta = delta > 0 ? 80 : -80;
        m_view->StartZoomAtPoint(position.x(), position.y());
        m_view->ZoomAtPoint(
            position.x(),
            position.y(),
            position.x(),
            position.y() + zoomDelta);
        event->accept();
    }
}

void OcctViewWidget::EnsureViewer()
{
    if (!m_context.IsNull())
    {
        return;
    }

    m_displayConnection = new Aspect_DisplayConnection();
    m_graphicDriver = new OpenGl_GraphicDriver(m_displayConnection);
    m_viewer = new V3d_Viewer(m_graphicDriver);
    m_viewer->SetDefaultLights();
    m_viewer->SetLightOn();

    m_context = new AIS_InteractiveContext(m_viewer);
    m_view = m_viewer->CreateView();

    Handle(WNT_Window) window =
        new WNT_Window(reinterpret_cast<Aspect_Handle>(winId()));
    m_view->SetWindow(window);
    if (!window->IsMapped())
    {
        window->Map();
    }

    m_view->SetBackgroundColor(Quantity_NOC_GRAY20);
    m_view->TriedronDisplay(
        Aspect_TOTP_LEFT_LOWER,
        Quantity_NOC_WHITE,
        0.08,
        V3d_ZBUFFER);
}

void OcctViewWidget::RedisplayModel(bool fitView)
{
    EnsureViewer();

    if (!m_modelPresentation.IsNull())
    {
        m_context->Remove(m_modelPresentation, Standard_False);
        m_modelPresentation.Nullify();
    }
    if (!m_meshPresentation.IsNull())
    {
        m_context->Remove(m_meshPresentation, Standard_False);
        m_meshPresentation.Nullify();
    }
    if (!m_meshTriangulationPresentation.IsNull())
    {
        m_context->Remove(m_meshTriangulationPresentation, Standard_False);
        m_meshTriangulationPresentation.Nullify();
    }

    if (m_modelShape.IsNull() && m_meshTriangulation.IsNull())
    {
        ClearWorldAxes();
        m_view->Redraw();
        return;
    }

    if (!m_meshTriangulation.IsNull())
    {
        if (m_displayMode == ViewDisplayMode::Shaded)
        {
            m_meshTriangulationPresentation =
                new AIS_Triangulation(m_meshTriangulation);
            m_context->Display(
                m_meshTriangulationPresentation,
                Standard_False);
            m_meshTriangulationPresentation->SetColor(
                Quantity_Color(0.82, 0.82, 0.82, Quantity_TOC_RGB));
            m_meshTriangulationPresentation->SetTransparency(0.15);
        }
        else
        {
            if (m_cachedMeshEdgeShape.IsNull())
            {
                m_cachedMeshEdgeShape = BuildTriangleMeshEdgeShape();
            }

            if (!m_cachedMeshEdgeShape.IsNull())
            {
                m_meshPresentation = new AIS_Shape(m_cachedMeshEdgeShape);
                m_meshPresentation->SetColor(
                    Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_RGB));
                m_meshPresentation->Attributes()->SetLineAspect(
                    new Prs3d_LineAspect(
                        Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_RGB),
                        Aspect_TOL_SOLID,
                        1.0));
                m_context->Display(m_meshPresentation, Standard_False);
                m_context->SetDisplayMode(
                    m_meshPresentation,
                    AIS_WireFrame,
                    Standard_False);
            }
        }

        DisplayWorldAxes();
        if (fitView)
        {
            m_view->FitAll();
        }
        m_view->Redraw();
        return;
    }

    if (m_displayMode != ViewDisplayMode::Mesh)
    {
        m_modelPresentation = new AIS_Shape(m_modelShape);
        m_context->Display(m_modelPresentation, Standard_False);
        m_context->SetDisplayMode(
            m_modelPresentation,
            m_displayMode == ViewDisplayMode::Wireframe ?
                AIS_WireFrame :
                AIS_Shaded,
            Standard_False);
    }

    if (m_displayMode == ViewDisplayMode::Mesh)
    {
        if (m_cachedMeshEdgeShape.IsNull())
        {
            m_cachedMeshEdgeShape = BuildMeshEdgeShape();
        }

        if (!m_cachedMeshEdgeShape.IsNull())
        {
            m_meshPresentation = new AIS_Shape(m_cachedMeshEdgeShape);
            m_meshPresentation->SetColor(
                Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_RGB));
            m_meshPresentation->Attributes()->SetLineAspect(
                new Prs3d_LineAspect(
                    Quantity_Color(0.0, 0.0, 0.0, Quantity_TOC_RGB),
                    Aspect_TOL_SOLID,
                    1.0));
            m_context->Display(m_meshPresentation, Standard_False);
            m_context->SetDisplayMode(
                m_meshPresentation,
                AIS_WireFrame,
                Standard_False);
        }
    }

    DisplayWorldAxes();
    if (fitView)
    {
        m_view->FitAll();
    }
    m_view->Redraw();
}

void OcctViewWidget::DisplayTransientShape(
    const TopoDS_Shape& shape,
    const Quantity_Color& color,
    double width)
{
    if (shape.IsNull())
    {
        return;
    }

    Handle(AIS_Shape) presentation = new AIS_Shape(shape);
    presentation->SetColor(color);
    presentation->Attributes()->SetLineAspect(
        new Prs3d_LineAspect(color, Aspect_TOL_SOLID, width));

    m_context->Display(presentation, Standard_False);
    m_overlays.push_back(presentation);
    m_view->Redraw();
}

void OcctViewWidget::DisplayEndpointShape(
    const TopoDS_Shape& shape,
    const Quantity_Color& color,
    double width)
{
    if (shape.IsNull())
    {
        return;
    }

    Handle(AIS_Shape) presentation = new AIS_Shape(shape);
    presentation->SetColor(color);
    presentation->Attributes()->SetLineAspect(
        new Prs3d_LineAspect(color, Aspect_TOL_SOLID, width));

    m_context->Display(presentation, Standard_False);
    m_endpointOverlays.push_back(presentation);
}

void OcctViewWidget::DisplayWorldAxes()
{
    ClearWorldAxes();

    double xmin = 0.0;
    double ymin = 0.0;
    double zmin = 0.0;
    double xmax = 0.0;
    double ymax = 0.0;
    double zmax = 0.0;

    if (!m_modelShape.IsNull())
    {
        Bnd_Box box;
        BRepBndLib::Add(m_modelShape, box);
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
    }
    else if (!m_meshData.points.empty())
    {
        xmin = xmax = m_meshData.points.front().X();
        ymin = ymax = m_meshData.points.front().Y();
        zmin = zmax = m_meshData.points.front().Z();

        for (const gp_Pnt& point : m_meshData.points)
        {
            xmin = std::min(xmin, point.X());
            ymin = std::min(ymin, point.Y());
            zmin = std::min(zmin, point.Z());
            xmax = std::max(xmax, point.X());
            ymax = std::max(ymax, point.Y());
            zmax = std::max(zmax, point.Z());
        }
    }

    const double dx = xmax - xmin;
    const double dy = ymax - ymin;
    const double dz = zmax - zmin;
    const double axisLength = std::max({dx, dy, dz, 10.0}) * 0.35;
    const gp_Pnt origin(0.0, 0.0, 0.0);

    struct AxisDef
    {
        gp_Pnt end;
        Quantity_Color color;
    };

    const AxisDef axes[] = {
        { gp_Pnt(axisLength, 0.0, 0.0),
          Quantity_Color(0.9, 0.05, 0.05, Quantity_TOC_RGB) },
        { gp_Pnt(0.0, axisLength, 0.0),
          Quantity_Color(0.05, 0.75, 0.05, Quantity_TOC_RGB) },
        { gp_Pnt(0.0, 0.0, axisLength),
          Quantity_Color(0.1, 0.25, 1.0, Quantity_TOC_RGB) }
    };

    for (const AxisDef& axis : axes)
    {
        BRepBuilderAPI_MakeEdge edge(origin, axis.end);
        if (!edge.IsDone())
        {
            continue;
        }

        Handle(AIS_Shape) presentation = new AIS_Shape(edge.Edge());
        presentation->SetColor(axis.color);
        presentation->Attributes()->SetLineAspect(
            new Prs3d_LineAspect(axis.color, Aspect_TOL_SOLID, 3.0));
        m_context->Display(presentation, Standard_False);
        m_worldAxisOverlays.push_back(presentation);
    }
}

void OcctViewWidget::ClearEndpointOverlays()
{
    for (const Handle(AIS_Shape)& overlay : m_endpointOverlays)
    {
        m_context->Remove(overlay, Standard_False);
    }
    m_endpointOverlays.clear();
}

void OcctViewWidget::ClearWorldAxes()
{
    for (const Handle(AIS_Shape)& overlay : m_worldAxisOverlays)
    {
        m_context->Remove(overlay, Standard_False);
    }
    m_worldAxisOverlays.clear();
}

TopoDS_Shape OcctViewWidget::BuildMeshEdgeShape() const
{
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    for (TopExp_Explorer explorer(m_modelShape, TopAbs_FACE);
        explorer.More();
        explorer.Next())
    {
        const TopoDS_Face face = TopoDS::Face(explorer.Current());
        TopLoc_Location location;
        const Handle(Poly_Triangulation) triangulation =
            BRep_Tool::Triangulation(face, location);

        if (triangulation.IsNull())
        {
            continue;
        }

        const gp_Trsf transform = location.Transformation();
        for (Standard_Integer i = 1;
            i <= triangulation->NbTriangles();
            ++i)
        {
            Standard_Integer n1 = 0;
            Standard_Integer n2 = 0;
            Standard_Integer n3 = 0;
            triangulation->Triangle(i).Get(n1, n2, n3);

            const gp_Pnt p1 = triangulation->Node(n1).Transformed(transform);
            const gp_Pnt p2 = triangulation->Node(n2).Transformed(transform);
            const gp_Pnt p3 = triangulation->Node(n3).Transformed(transform);

            BRepBuilderAPI_MakeEdge e12(p1, p2);
            BRepBuilderAPI_MakeEdge e23(p2, p3);
            BRepBuilderAPI_MakeEdge e31(p3, p1);

            if (e12.IsDone())
            {
                builder.Add(compound, e12.Edge());
            }
            if (e23.IsDone())
            {
                builder.Add(compound, e23.Edge());
            }
            if (e31.IsDone())
            {
                builder.Add(compound, e31.Edge());
            }
        }
    }

    return compound;
}

TopoDS_Shape OcctViewWidget::BuildTriangleMeshEdgeShape() const
{
    if (m_meshData.IsEmpty())
    {
        return TopoDS_Shape();
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    std::unordered_set<std::uint64_t> seenEdges;
    seenEdges.reserve(m_meshData.triangles.size() * 3);

    for (const std::array<int, 3>& tri : m_meshData.triangles)
    {
        AddUniqueMeshEdge(
            m_meshData,
            tri[0],
            tri[1],
            seenEdges,
            builder,
            compound);
        AddUniqueMeshEdge(
            m_meshData,
            tri[1],
            tri[2],
            seenEdges,
            builder,
            compound);
        AddUniqueMeshEdge(
            m_meshData,
            tri[2],
            tri[0],
            seenEdges,
            builder,
            compound);
    }

    return compound;
}

bool OcctViewWidget::PickModelPoint(
    const QPoint& screenPoint,
    gp_Pnt& outPoint) const
{
    if (m_view.IsNull() ||
        (m_modelShape.IsNull() && m_meshData.IsEmpty()))
    {
        return false;
    }

    Standard_Real x = 0.0;
    Standard_Real y = 0.0;
    Standard_Real z = 0.0;
    Standard_Real dx = 0.0;
    Standard_Real dy = 0.0;
    Standard_Real dz = 0.0;
    m_view->ConvertWithProj(
        screenPoint.x(),
        screenPoint.y(),
        x,
        y,
        z,
        dx,
        dy,
        dz);

    const gp_Pnt origin(x, y, z);
    gp_Vec direction(dx, dy, dz);
    if (direction.SquareMagnitude() <= 1.0e-12)
    {
        return false;
    }
    direction.Normalize();

    bool hit = false;
    double bestT = std::numeric_limits<double>::infinity();

    if (!m_meshData.IsEmpty())
    {
        for (const std::array<int, 3>& tri : m_meshData.triangles)
        {
            double t = 0.0;
            if (IntersectRayTriangle(
                    origin,
                    direction,
                    m_meshData.points[static_cast<std::size_t>(tri[0])],
                    m_meshData.points[static_cast<std::size_t>(tri[1])],
                    m_meshData.points[static_cast<std::size_t>(tri[2])],
                    t) &&
                t < bestT)
            {
                bestT = t;
                hit = true;
            }
        }
    }
    else
    {
        for (TopExp_Explorer explorer(m_modelShape, TopAbs_FACE);
            explorer.More();
            explorer.Next())
        {
            const TopoDS_Face face = TopoDS::Face(explorer.Current());
            TopLoc_Location location;
            const Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, location);

            if (triangulation.IsNull())
            {
                continue;
            }

            const gp_Trsf transform = location.Transformation();
            for (Standard_Integer i = 1;
                i <= triangulation->NbTriangles();
                ++i)
            {
                Standard_Integer n1 = 0;
                Standard_Integer n2 = 0;
                Standard_Integer n3 = 0;
                triangulation->Triangle(i).Get(n1, n2, n3);

                const gp_Pnt p1 = triangulation->Node(n1).Transformed(transform);
                const gp_Pnt p2 = triangulation->Node(n2).Transformed(transform);
                const gp_Pnt p3 = triangulation->Node(n3).Transformed(transform);

                double t = 0.0;
                if (IntersectRayTriangle(origin, direction, p1, p2, p3, t) &&
                    t < bestT)
                {
                    bestT = t;
                    hit = true;
                }
            }
        }
    }

    if (!hit)
    {
        return false;
    }

    outPoint = origin.Translated(direction * bestT);
    return true;
}

} // namespace path_planning_workbench
