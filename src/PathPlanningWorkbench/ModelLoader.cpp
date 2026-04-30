#include "ModelLoader.h"

#include <BRep_Builder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <STEPControl_Reader.hxx>

#include <QFile>
#include <QFileInfo>
#include <QTextStream>

#include <vector>

namespace path_planning_workbench
{
bool ModelLoader::Load(
    const QString& path,
    const ModelImportOptions& options,
    ImportedModel& outModel,
    QString& outError)
{
    outModel = ImportedModel();
    outError.clear();

    const QFileInfo info(path);
    const QString suffix = info.suffix().toLower();

    TopoDS_Shape shape;
    TriangleMeshData mesh;
    ImportedModelKind kind = ImportedModelKind::Unknown;
    bool loaded = false;

    if (suffix == "step" || suffix == "stp")
    {
        loaded = LoadStep(path, shape, outError);
        kind = ImportedModelKind::Shape;
    }
    else if (suffix == "brep")
    {
        loaded = LoadBrep(path, shape, outError);
        kind = ImportedModelKind::Shape;
    }
    else if (suffix == "vtk")
    {
        loaded = LoadLegacyVtk(path, mesh, outError);
        kind = ImportedModelKind::Mesh;
    }
    else
    {
        outError = "Unsupported model file type: " + suffix;
        return false;
    }

    if (!loaded)
    {
        return false;
    }

    if (kind == ImportedModelKind::Shape && options.remeshShape)
    {
        if (!RemeshShape(shape, options, outError))
        {
            return false;
        }
    }

    outModel.kind = kind;
    outModel.sourcePath = path;
    outModel.shape = shape;
    outModel.mesh = std::move(mesh);
    return true;
}

bool ModelLoader::RemeshShape(
    TopoDS_Shape& shape,
    const ModelImportOptions& options,
    QString& outError)
{
    if (shape.IsNull())
    {
        outError = "Cannot remesh a null shape.";
        return false;
    }

    BRepMesh_IncrementalMesh mesher(
        shape,
        options.linearDeflection,
        false,
        options.angularDeflection,
        true);

    if (!mesher.IsDone())
    {
        outError = "OCCT shape remeshing failed.";
        return false;
    }

    return true;
}

bool ModelLoader::LoadStep(
    const QString& path,
    TopoDS_Shape& outShape,
    QString& outError)
{
    STEPControl_Reader reader;
    const IFSelect_ReturnStatus status =
        reader.ReadFile(path.toLocal8Bit().constData());

    if (status != IFSelect_RetDone)
    {
        outError = "Failed to read STEP file: " + path;
        return false;
    }

    reader.TransferRoots();
    outShape = reader.OneShape();

    if (outShape.IsNull())
    {
        outError = "STEP file did not contain a valid shape: " + path;
        return false;
    }

    return true;
}

bool ModelLoader::LoadBrep(
    const QString& path,
    TopoDS_Shape& outShape,
    QString& outError)
{
    BRep_Builder builder;

    if (!BRepTools::Read(outShape, path.toLocal8Bit().constData(), builder) ||
        outShape.IsNull())
    {
        outError = "Failed to read BREP file: " + path;
        return false;
    }

    return true;
}

bool ModelLoader::LoadLegacyVtk(
    const QString& path,
    TriangleMeshData& outMesh,
    QString& outError)
{
    outMesh = TriangleMeshData();

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        outError = "Failed to open VTK file: " + path;
        return false;
    }

    QTextStream stream(&file);
    std::vector<gp_Pnt> points;
    std::vector<std::array<int, 3>> triangles;

    while (!stream.atEnd())
    {
        QString token;
        stream >> token;

        if (token.compare("POINTS", Qt::CaseInsensitive) == 0)
        {
            int count = 0;
            QString typeName;
            stream >> count >> typeName;
            points.reserve(static_cast<std::size_t>(count));

            for (int i = 0; i < count; ++i)
            {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                stream >> x >> y >> z;
                points.emplace_back(x, y, z);
            }
        }
        else if (token.compare("POLYGONS", Qt::CaseInsensitive) == 0)
        {
            int polygonCount = 0;
            int indexValueCount = 0;
            stream >> polygonCount >> indexValueCount;

            for (int i = 0; i < polygonCount; ++i)
            {
                int vertexCount = 0;
                stream >> vertexCount;

                std::vector<int> ids(static_cast<std::size_t>(vertexCount));
                for (int j = 0; j < vertexCount; ++j)
                {
                    stream >> ids[static_cast<std::size_t>(j)];
                }

                if (vertexCount < 3)
                {
                    continue;
                }

                for (int j = 1; j + 1 < vertexCount; ++j)
                {
                    const int ia = ids[0];
                    const int ib = ids[static_cast<std::size_t>(j)];
                    const int ic = ids[static_cast<std::size_t>(j + 1)];

                    if (ia < 0 || ib < 0 || ic < 0 ||
                        ia >= static_cast<int>(points.size()) ||
                        ib >= static_cast<int>(points.size()) ||
                        ic >= static_cast<int>(points.size()))
                    {
                        outError = "VTK polygon index is out of range: " + path;
                        return false;
                    }

                    triangles.push_back({ ia, ib, ic });
                }
            }
        }
    }

    if (points.empty() || triangles.empty())
    {
        outError = "VTK file did not contain a valid polygon mesh: " + path;
        return false;
    }

    outMesh.points = std::move(points);
    outMesh.triangles = std::move(triangles);
    return true;
}

} // namespace path_planning_workbench
