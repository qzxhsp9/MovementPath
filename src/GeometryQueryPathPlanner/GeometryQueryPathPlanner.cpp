#include "GeometryQueryPathPlanner.h"

namespace movement_path::geometry
{

GeometryPathResult GeometryQueryPathPlanner::Plan(
    const GeometryPathRequest& request,
    const GeometryPathOptions& options) const
{
    (void)options;

    GeometryPathResult result;
    result.profile.triangleCount = request.triangles.size();

    if (request.triangles.empty())
    {
        result.status = GeometryPathStatus::InvalidInput;
        result.message = "GeometryQueryPathPlanner requires mesh triangles.";
        return result;
    }

    result.status = GeometryPathStatus::NotImplemented;
    result.message =
        "Geometry-query planner module is scaffolded but not implemented.";
    return result;
}

} // namespace movement_path::geometry
