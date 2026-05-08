#include "GeometryQueryPathPlanner.h"
#include "GeometryQueryContext.h"

#include <cmath>
#include <limits>

namespace movement_path::geometry
{
namespace
{
bool IsValidOptions(const GeometryPathOptions& options)
{
    return std::isfinite(options.clearance) &&
        options.clearance >= 0.0 &&
        options.maxExpansionCount <=
            static_cast<std::size_t>(std::numeric_limits<int>::max());
}
}

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

    if (!request.startPoint.IsFinite() || !request.goalPoint.IsFinite())
    {
        result.status = GeometryPathStatus::InvalidInput;
        result.message =
            "GeometryQueryPathPlanner requires finite start and goal points.";
        return result;
    }

    if (!IsValidOptions(options))
    {
        result.status = GeometryPathStatus::InvalidInput;
        result.message =
            "GeometryQueryPathPlanner received invalid options.";
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

    result.status = GeometryPathStatus::NotImplemented;
    result.message =
        "Geometry-query planner module is scaffolded but not implemented.";
    return result;
}

} // namespace movement_path::geometry
