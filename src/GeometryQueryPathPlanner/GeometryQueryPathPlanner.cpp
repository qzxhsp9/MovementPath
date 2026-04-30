#include "GeometryQueryPathPlanner.h"
#include "GeometryQueryContext.h"

namespace movement_path::geometry
{

GeometryPathResult GeometryQueryPathPlanner::Plan(
    const GeometryPathRequest& request,
    const GeometryPathOptions& options) const
{
    GeometryPathResult result;
    result.profile.triangleCount = request.triangles.size();

    if (request.triangles.empty())
    {
        result.status = GeometryPathStatus::InvalidInput;
        result.message = "GeometryQueryPathPlanner requires mesh triangles.";
        return result;
    }

    GeometryQueryContext queryContext;

    if (!queryContext.Build(request.triangles))
    {
        result.status = GeometryPathStatus::Failed;
        result.message = "GeometryQueryPathPlanner failed to build query context.";
        result.profile = queryContext.Profile();
        return result;
    }

    result.profile = queryContext.Profile();
    (void)options;

    result.status = GeometryPathStatus::NotImplemented;
    result.message =
        "Geometry-query planner module is scaffolded but not implemented.";
    return result;
}

} // namespace movement_path::geometry
