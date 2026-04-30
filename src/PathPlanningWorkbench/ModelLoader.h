#pragma once

#include <QString>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>

#include <array>
#include <string>
#include <vector>

namespace path_planning_workbench
{

enum class ImportedModelKind
{
    Unknown,
    Shape,
    Mesh
};

struct ModelImportOptions
{
    double linearDeflection = 0.1;
    double angularDeflection = 0.5;
    bool remeshShape = true;
};

struct TriangleMeshData
{
    std::vector<gp_Pnt> points;
    std::vector<std::array<int, 3>> triangles;

    bool IsEmpty() const
    {
        return points.empty() || triangles.empty();
    }
};

struct ImportedModel
{
    ImportedModelKind kind = ImportedModelKind::Unknown;
    QString sourcePath;
    TopoDS_Shape shape;
    TriangleMeshData mesh;
};

class ModelLoader
{
public:
    static bool Load(
        const QString& path,
        const ModelImportOptions& options,
        ImportedModel& outModel,
        QString& outError);

    static bool RemeshShape(
        TopoDS_Shape& shape,
        const ModelImportOptions& options,
        QString& outError);

private:
    static bool LoadStep(
        const QString& path,
        TopoDS_Shape& outShape,
        QString& outError);

    static bool LoadBrep(
        const QString& path,
        TopoDS_Shape& outShape,
        QString& outError);

    static bool LoadLegacyVtk(
        const QString& path,
        TriangleMeshData& outMesh,
        QString& outError);
};

} // namespace path_planning_workbench
