#pragma once

#include <unordered_map>
#include <vector>
#include <limits>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <algorithm>

#include <gp_Pnt.hxx>

// ============================================================
// 体素索引
// ============================================================

struct VoxelIndex
{
    int x = 0;
    int y = 0;
    int z = 0;

    VoxelIndex() = default;

    VoxelIndex(int ix, int iy, int iz)
        : x(ix), y(iy), z(iz)
    {
    }

    bool operator==(const VoxelIndex& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    bool operator!=(const VoxelIndex& other) const
    {
        return !(*this == other);
    }
};

struct VoxelIndexHash
{
    std::size_t operator()(const VoxelIndex& index) const
    {
        const std::size_t hx =
            static_cast<std::size_t>(index.x) * 73856093u;
        const std::size_t hy =
            static_cast<std::size_t>(index.y) * 19349663u;
        const std::size_t hz =
            static_cast<std::size_t>(index.z) * 83492791u;

        return hx ^ hy ^ hz;
    }
};

// ============================================================
// 体素状态
// ============================================================

enum class VoxelState
{
    Free = 0,

    // 与障碍物表面三角片相交，不可通行
    Occupied,

    // 安全距离层 / 候选运动层。
    // 在 ClearanceBand A* 模式下，该状态是可通行区域。
    ClearanceBand,

    Start,
    Goal,
    Path
};

// ============================================================
// 体素单元
// ============================================================

struct VoxelCell
{
    VoxelIndex index;

    VoxelState state = VoxelState::Free;

    // 到最近三角面的距离，可选字段
    double distanceToSurface = std::numeric_limits<double>::max();

    // A* 字段
    double gCost = std::numeric_limits<double>::max();
    double hCost = 0.0;
    double fCost = std::numeric_limits<double>::max();

    bool opened = false;
    bool closed = false;

    bool hasParent = false;
    VoxelIndex parent;

    VoxelCell() = default;

    explicit VoxelCell(const VoxelIndex& idx)
        : index(idx)
    {
    }

    bool IsSurfaceBlocked() const
    {
        return state == VoxelState::Occupied;
    }

    void ResetSearchData()
    {
        gCost = std::numeric_limits<double>::max();
        hCost = 0.0;
        fCost = std::numeric_limits<double>::max();

        opened = false;
        closed = false;

        hasParent = false;
        parent = VoxelIndex();
    }
};

// ============================================================
// 搜索范围
// ============================================================

struct VoxelBounds
{
    VoxelIndex minIndex;
    VoxelIndex maxIndex;

    bool IsValid() const
    {
        return minIndex.x <= maxIndex.x &&
            minIndex.y <= maxIndex.y &&
            minIndex.z <= maxIndex.z;
    }

    bool Contains(const VoxelIndex& index) const
    {
        return index.x >= minIndex.x && index.x <= maxIndex.x &&
            index.y >= minIndex.y && index.y <= maxIndex.y &&
            index.z >= minIndex.z && index.z <= maxIndex.z;
    }
};

inline void ExpandBoundsToInclude(
    VoxelBounds& bounds,
    const VoxelIndex& index)
{
    if (!bounds.IsValid())
    {
        bounds.minIndex = index;
        bounds.maxIndex = index;
        return;
    }

    bounds.minIndex.x = std::min(bounds.minIndex.x, index.x);
    bounds.minIndex.y = std::min(bounds.minIndex.y, index.y);
    bounds.minIndex.z = std::min(bounds.minIndex.z, index.z);

    bounds.maxIndex.x = std::max(bounds.maxIndex.x, index.x);
    bounds.maxIndex.y = std::max(bounds.maxIndex.y, index.y);
    bounds.maxIndex.z = std::max(bounds.maxIndex.z, index.z);
}

inline void ExpandBoundsByVoxelRadius(
    VoxelBounds& bounds,
    int radius)
{
    if (!bounds.IsValid() || radius <= 0)
    {
        return;
    }

    bounds.minIndex.x -= radius;
    bounds.minIndex.y -= radius;
    bounds.minIndex.z -= radius;

    bounds.maxIndex.x += radius;
    bounds.maxIndex.y += radius;
    bounds.maxIndex.z += radius;
}

// ============================================================
// 邻接类型
// ============================================================

enum class VoxelNeighborType
{
    Face6,
    FaceEdge18,
    FaceEdgeVertex26
};

// ============================================================
// VoxelSpace
// ============================================================

class VoxelSpace
{
public:
    using CellMap =
        std::unordered_map<VoxelIndex, VoxelCell, VoxelIndexHash>;

public:
    VoxelSpace() = default;

    VoxelSpace(const gp_Pnt& origin, double voxelSize)
        : m_origin(origin),
        m_voxelSize(voxelSize)
    {
        if (voxelSize <= 0.0)
        {
            throw std::invalid_argument("voxelSize must be positive.");
        }
    }

public:
    void Clear()
    {
        m_cells.clear();
    }

    bool IsValid() const
    {
        return m_voxelSize > 0.0;
    }

    void SetOrigin(const gp_Pnt& origin)
    {
        m_origin = origin;
    }

    const gp_Pnt& GetOrigin() const
    {
        return m_origin;
    }

    void SetVoxelSize(double voxelSize)
    {
        if (voxelSize <= 0.0)
        {
            throw std::invalid_argument("voxelSize must be positive.");
        }

        m_voxelSize = voxelSize;
    }

    double GetVoxelSize() const
    {
        return m_voxelSize;
    }

    double GetHalfDiagonal() const
    {
        return 0.5 * std::sqrt(3.0) * m_voxelSize;
    }

public:
    void SetSearchBounds(const VoxelBounds& bounds)
    {
        m_bounds = bounds;
        m_hasBounds = bounds.IsValid();
    }

    bool HasSearchBounds() const
    {
        return m_hasBounds;
    }

    const VoxelBounds& GetSearchBounds() const
    {
        return m_bounds;
    }

    bool IsInsideSearchBounds(const VoxelIndex& index) const
    {
        if (!m_hasBounds)
        {
            return true;
        }

        return m_bounds.Contains(index);
    }

public:
    VoxelIndex WorldToIndex(const gp_Pnt& p) const
    {
        return VoxelIndex(
            static_cast<int>(std::floor((p.X() - m_origin.X()) / m_voxelSize)),
            static_cast<int>(std::floor((p.Y() - m_origin.Y()) / m_voxelSize)),
            static_cast<int>(std::floor((p.Z() - m_origin.Z()) / m_voxelSize))
        );
    }

    gp_Pnt IndexToMinCorner(const VoxelIndex& index) const
    {
        return gp_Pnt(
            m_origin.X() + static_cast<double>(index.x) * m_voxelSize,
            m_origin.Y() + static_cast<double>(index.y) * m_voxelSize,
            m_origin.Z() + static_cast<double>(index.z) * m_voxelSize
        );
    }

    gp_Pnt IndexToMaxCorner(const VoxelIndex& index) const
    {
        gp_Pnt minP = IndexToMinCorner(index);

        return gp_Pnt(
            minP.X() + m_voxelSize,
            minP.Y() + m_voxelSize,
            minP.Z() + m_voxelSize
        );
    }

    gp_Pnt IndexToCenter(const VoxelIndex& index) const
    {
        return gp_Pnt(
            m_origin.X() + (static_cast<double>(index.x) + 0.5) * m_voxelSize,
            m_origin.Y() + (static_cast<double>(index.y) + 0.5) * m_voxelSize,
            m_origin.Z() + (static_cast<double>(index.z) + 0.5) * m_voxelSize
        );
    }

public:
    bool HasCell(const VoxelIndex& index) const
    {
        return m_cells.find(index) != m_cells.end();
    }

    VoxelCell* FindCell(const VoxelIndex& index)
    {
        auto it = m_cells.find(index);
        if (it == m_cells.end())
        {
            return nullptr;
        }

        return &it->second;
    }

    const VoxelCell* FindCell(const VoxelIndex& index) const
    {
        auto it = m_cells.find(index);
        if (it == m_cells.end())
        {
            return nullptr;
        }

        return &it->second;
    }

    VoxelCell& GetOrCreateCell(const VoxelIndex& index)
    {
        auto it = m_cells.find(index);
        if (it != m_cells.end())
        {
            return it->second;
        }

        VoxelCell cell(index);
        auto result = m_cells.emplace(index, cell);

        return result.first->second;
    }

    VoxelState GetCellState(const VoxelIndex& index) const
    {
        const VoxelCell* cell = FindCell(index);

        if (cell == nullptr)
        {
            // 稀疏体素场：未存储体素默认 Free
            return VoxelState::Free;
        }

        return cell->state;
    }

    void SetCellState(const VoxelIndex& index, VoxelState state)
    {
        VoxelCell& cell = GetOrCreateCell(index);
        cell.state = state;
    }

    void SetCellDistanceIfSmaller(
        const VoxelIndex& index,
        double distanceToSurface)
    {
        VoxelCell& cell = GetOrCreateCell(index);

        if (distanceToSurface < cell.distanceToSurface)
        {
            cell.distanceToSurface = distanceToSurface;
        }
    }

    std::size_t CellCount() const
    {
        return m_cells.size();
    }

    CellMap& Cells()
    {
        return m_cells;
    }

    const CellMap& Cells() const
    {
        return m_cells;
    }

public:
    std::vector<VoxelIndex> GetNeighborIndices(
        const VoxelIndex& index,
        VoxelNeighborType type) const
    {
        std::vector<VoxelIndex> neighbors;

        if (type == VoxelNeighborType::Face6)
        {
            neighbors.reserve(6);

            neighbors.emplace_back(index.x + 1, index.y, index.z);
            neighbors.emplace_back(index.x - 1, index.y, index.z);
            neighbors.emplace_back(index.x, index.y + 1, index.z);
            neighbors.emplace_back(index.x, index.y - 1, index.z);
            neighbors.emplace_back(index.x, index.y, index.z + 1);
            neighbors.emplace_back(index.x, index.y, index.z - 1);

            return neighbors;
        }

        neighbors.reserve(
            type == VoxelNeighborType::FaceEdge18 ? 18 : 26
        );

        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }

                    int manhattan =
                        std::abs(dx) + std::abs(dy) + std::abs(dz);

                    if (type == VoxelNeighborType::FaceEdge18 &&
                        manhattan == 3)
                    {
                        continue;
                    }

                    neighbors.emplace_back(
                        index.x + dx,
                        index.y + dy,
                        index.z + dz
                    );
                }
            }
        }

        return neighbors;
    }

    double GetMoveCost(
        const VoxelIndex& a,
        const VoxelIndex& b) const
    {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        int dz = a.z - b.z;

        return m_voxelSize * std::sqrt(
            static_cast<double>(dx * dx + dy * dy + dz * dz)
        );
    }

    void ResetSearchData()
    {
        for (auto& kv : m_cells)
        {
            kv.second.ResetSearchData();
        }
    }

private:
    gp_Pnt m_origin;
    double m_voxelSize = 1.0;

    bool m_hasBounds = false;
    VoxelBounds m_bounds;

    CellMap m_cells;
};