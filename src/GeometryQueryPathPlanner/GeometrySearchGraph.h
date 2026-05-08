#pragma once

#include "GeometryPrimitives.h"
#include "GeometryQueries.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace movement_path::geometry
{

enum class GeometrySearchNodeKind
{
    Start,
    Goal,
    Sample
};

struct GeometrySearchNode
{
    int id = -1;
    GeometrySearchNodeKind kind = GeometrySearchNodeKind::Sample;
    Vec3 position;
};

struct GeometrySearchEdge
{
    int id = -1;
    int fromNodeId = -1;
    int toNodeId = -1;
    double cost = 0.0;
    SegmentClearanceResult clearance;
};

struct GeometrySearchGraphProfile
{
    std::size_t nodeCount = 0;
    std::size_t edgeCount = 0;
    std::size_t feasibleEdgeCount = 0;
    std::size_t blockedEdgeCount = 0;
};

class GeometrySearchGraph
{
public:
    int AddNode(
        const Vec3& position,
        GeometrySearchNodeKind kind = GeometrySearchNodeKind::Sample)
    {
        const int id = static_cast<int>(m_nodes.size());
        m_nodes.push_back({ id, kind, position });
        return id;
    }

    int AddEdge(
        int fromNodeId,
        int toNodeId,
        const SegmentClearanceResult& clearance)
    {
        const int id = static_cast<int>(m_edges.size());
        const Vec3 delta =
            m_nodes[toNodeId].position - m_nodes[fromNodeId].position;
        GeometrySearchEdge edge;
        edge.id = id;
        edge.fromNodeId = fromNodeId;
        edge.toNodeId = toNodeId;
        edge.cost = std::sqrt(delta.SquaredLength());
        edge.clearance = clearance;
        m_edges.push_back(edge);
        return id;
    }

    const std::vector<GeometrySearchNode>& Nodes() const
    {
        return m_nodes;
    }

    const std::vector<GeometrySearchEdge>& Edges() const
    {
        return m_edges;
    }

    GeometrySearchGraphProfile Profile() const
    {
        GeometrySearchGraphProfile profile;
        profile.nodeCount = m_nodes.size();
        profile.edgeCount = m_edges.size();

        for (const GeometrySearchEdge& edge : m_edges)
        {
            if (edge.clearance.pass)
            {
                ++profile.feasibleEdgeCount;
            }
            else
            {
                ++profile.blockedEdgeCount;
            }
        }

        return profile;
    }

private:
    std::vector<GeometrySearchNode> m_nodes;
    std::vector<GeometrySearchEdge> m_edges;
};

} // namespace movement_path::geometry
