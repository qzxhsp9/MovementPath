#include "TestCommon.h"

#include "VoxelVtkExporter.h"
#include "VoxelChunkCache.h"

#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace
{
bool TestAppendVoxelSpaceExpandsBoundsAndPreservesState()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(0, 0, 0), Vec(10, 0, 0), Vec(0, 10, 0) }
    );

    VoxelMeshBuildOptions options;
    options.voxelSize = 1.0;
    options.clearance = 2.0;
    options.conservativeClearance = true;
    options.storeFreeCells = false;

    TriangleSpatialHashOptions hashOptions;
    hashOptions.cellSize = 4.0;
    TriangleSpatialHash hash;
    bool ok = Expect(hash.Build(triangles, hashOptions), "hash should build");

    MeshAABB baseBox;
    baseBox.minP = Vec(-1, -1, -1);
    baseBox.maxP = Vec(2, 2, 1);

    VoxelSpace space;
    const VoxelMeshBuildResult baseResult =
        VoxelMeshBuilder::BuildVoxelSpaceFromTrianglesInBox(
            triangles,
            &hash,
            baseBox,
            options,
            space
        );

    ok &= Expect(baseResult.success, "base voxel build should succeed");

    const VoxelIndex preservedIndex = space.WorldToIndex(Vec(0, 0, 0));
    const VoxelState preservedState = space.GetCellState(preservedIndex);

    const VoxelIndex appendOnlyIndex = space.WorldToIndex(Vec(5, 0, 0));
    ok &= Expect(
        !space.IsInsideSearchBounds(appendOnlyIndex),
        "append-only index should start outside base bounds");

    MeshAABB appendBox;
    appendBox.minP = Vec(4, -1, -1);
    appendBox.maxP = Vec(7, 2, 1);

    const VoxelMeshBuildResult appendResult =
        VoxelMeshBuilder::AppendVoxelSpaceFromTrianglesInBox(
            triangles,
            &hash,
            appendBox,
            options,
            space
        );

    ok &= Expect(appendResult.success, "append voxel build should succeed");
    ok &= Expect(
        space.IsInsideSearchBounds(appendOnlyIndex),
        "append should expand search bounds to include appended box");
    ok &= Expect(
        space.GetCellState(appendOnlyIndex) != VoxelState::Free,
        "append should populate voxels outside the original bounds");
    ok &= Expect(
        space.GetCellState(preservedIndex) == preservedState,
        "append should preserve existing voxel state");

    return ok;
}

bool TestVoxelChunkCacheBuildsEachChunkOnce()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(0, 0, 0), Vec(10, 0, 0), Vec(0, 10, 0) }
    );

    VoxelMeshBuildOptions buildOptions;
    buildOptions.voxelSize = 1.0;
    buildOptions.clearance = 2.0;
    buildOptions.conservativeClearance = true;

    TriangleSpatialHashOptions hashOptions;
    hashOptions.cellSize = 4.0;
    TriangleSpatialHash hash;

    bool ok = Expect(hash.Build(triangles, hashOptions), "hash should build");

    VoxelChunkCacheOptions cacheOptions;
    cacheOptions.chunkVoxelSize = 4;
    cacheOptions.buildPadding = 0.0;

    VoxelChunkCache cache;
    ok &= Expect(
        cache.Configure(
            &triangles,
            &hash,
            buildOptions,
            cacheOptions),
        "chunk cache should configure");

    VoxelSpace space(Vec(-1, -1, -1), buildOptions.voxelSize);
    const VoxelIndex targetIndex = space.WorldToIndex(Vec(1, 1, 0));

    ok &= Expect(
        cache.EnsureChunkForIndex(space, targetIndex),
        "chunk cache should build target chunk");

    const VoxelChunkCacheStats firstStats = cache.GetStats();
    ok &= Expect(
        firstStats.chunkBuildCount == 1,
        "first ensure should build one chunk");
    ok &= Expect(
        space.GetCellState(targetIndex) != VoxelState::Free,
        "built chunk should populate target voxel");

    ok &= Expect(
        cache.EnsureChunkForIndex(space, targetIndex),
        "second ensure should hit cache");

    const VoxelChunkCacheStats secondStats = cache.GetStats();
    ok &= Expect(
        secondStats.chunkBuildCount == 1,
        "cache hit should not rebuild chunk");
    ok &= Expect(
        secondStats.cacheHitCount == 1,
        "second ensure should increment cache hit count");

    return ok;
}

bool TestVoxelChunkCacheConfigurationFailures()
{
    VoxelChunkCache cache;
    VoxelMeshBuildOptions buildOptions;
    VoxelChunkCacheOptions cacheOptions;
    std::vector<MeshTriangle> triangles;

    bool ok = true;
    ok &= Expect(
        !cache.Configure(nullptr, nullptr, buildOptions, cacheOptions),
        "chunk cache should reject null triangle storage");
    ok &= Expect(
        !cache.IsConfigured(),
        "chunk cache should remain unconfigured after null triangles");

    ok &= Expect(
        !cache.Configure(&triangles, nullptr, buildOptions, cacheOptions),
        "chunk cache should reject empty triangles");

    triangles.push_back(
        { Vec(0, 0, 0), Vec(4, 0, 0), Vec(0, 4, 0) }
    );

    VoxelMeshBuildOptions badBuildOptions = buildOptions;
    badBuildOptions.voxelSize = 0.0;

    ok &= Expect(
        !cache.Configure(
            &triangles,
            nullptr,
            badBuildOptions,
            cacheOptions),
        "chunk cache should reject non-positive voxel size");

    VoxelChunkCacheOptions badCacheOptions = cacheOptions;
    badCacheOptions.chunkVoxelSize = 0;

    ok &= Expect(
        !cache.Configure(
            &triangles,
            nullptr,
            buildOptions,
            badCacheOptions),
        "chunk cache should reject non-positive chunk size");

    VoxelSpace invalidSpace;
    ok &= Expect(
        !cache.EnsureChunkForIndex(invalidSpace, VoxelIndex()),
        "unconfigured chunk cache ensure should fail");
    ok &= Expect(
        cache.GetStats().failedBuildCount == 1,
        "failed unconfigured ensure should update failed count");

    return ok;
}

bool TestVoxelChunkCacheClearResetsState()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(0, 0, 0), Vec(4, 0, 0), Vec(0, 4, 0) }
    );

    VoxelMeshBuildOptions buildOptions;
    buildOptions.voxelSize = 1.0;
    buildOptions.clearance = 1.0;

    VoxelChunkCacheOptions cacheOptions;
    cacheOptions.chunkVoxelSize = 4;

    VoxelChunkCache cache;
    bool ok = Expect(
        cache.Configure(
            &triangles,
            nullptr,
            buildOptions,
            cacheOptions),
        "chunk cache should configure without spatial hash");

    VoxelSpace space(Vec(-1, -1, -1), buildOptions.voxelSize);
    const VoxelIndex index = space.WorldToIndex(Vec(1, 1, 0));

    ok &= Expect(
        cache.EnsureChunkForIndex(space, index),
        "chunk cache should build without spatial hash");
    ok &= Expect(
        cache.GetStats().chunkBuildCount == 1,
        "chunk cache should record one build before clear");

    cache.Clear();

    ok &= Expect(!cache.IsConfigured(), "clear should unconfigure cache");
    ok &= Expect(
        cache.GetStats().chunkBuildCount == 0,
        "clear should reset chunk build count");
    ok &= Expect(
        !cache.IsChunkBuiltForIndex(index),
        "clear should remove built chunk records");

    return ok;
}

bool TestVoxelChunkCacheNegativeAndAdjacentChunks()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(-6, -2, 0), Vec(6, -2, 0), Vec(-6, 6, 0) }
    );

    VoxelMeshBuildOptions buildOptions;
    buildOptions.voxelSize = 1.0;
    buildOptions.clearance = 1.0;
    buildOptions.conservativeClearance = true;

    TriangleSpatialHashOptions hashOptions;
    hashOptions.cellSize = 4.0;
    TriangleSpatialHash hash;

    bool ok = Expect(hash.Build(triangles, hashOptions), "hash should build");

    VoxelChunkCacheOptions cacheOptions;
    cacheOptions.chunkVoxelSize = 4;

    VoxelChunkCache cache;
    ok &= Expect(
        cache.Configure(
            &triangles,
            &hash,
            buildOptions,
            cacheOptions),
        "chunk cache should configure for negative chunk test");

    VoxelSpace space(Vec(0, 0, 0), buildOptions.voxelSize);
    const VoxelIndex negativeIndex = space.WorldToIndex(Vec(-1, -1, 0));
    const VoxelIndex sameNegativeChunkIndex =
        space.WorldToIndex(Vec(-4, -1, 0));
    const VoxelIndex adjacentNegativeChunkIndex =
        space.WorldToIndex(Vec(-5, -1, 0));
    const VoxelIndex positiveIndex = space.WorldToIndex(Vec(1, 1, 0));

    ok &= Expect(
        cache.EnsureChunkForIndex(space, negativeIndex),
        "negative chunk should build");
    ok &= Expect(
        cache.GetStats().chunkBuildCount == 1,
        "negative chunk should count as one build");

    ok &= Expect(
        cache.EnsureChunkForIndex(space, sameNegativeChunkIndex),
        "same negative chunk should ensure");
    ok &= Expect(
        cache.GetStats().chunkBuildCount == 1,
        "same negative chunk should not rebuild");
    ok &= Expect(
        cache.GetStats().cacheHitCount == 1,
        "same negative chunk should hit cache");

    ok &= Expect(
        cache.EnsureChunkForIndex(space, adjacentNegativeChunkIndex),
        "adjacent negative chunk should build");
    ok &= Expect(
        cache.GetStats().chunkBuildCount == 2,
        "adjacent negative chunk should be a distinct build");

    ok &= Expect(
        cache.EnsureChunkForIndex(space, positiveIndex),
        "positive chunk should build");
    ok &= Expect(
        cache.GetStats().chunkBuildCount == 3,
        "positive chunk should be another distinct build");

    ok &= Expect(
        space.IsInsideSearchBounds(negativeIndex) &&
            space.IsInsideSearchBounds(positiveIndex),
        "search bounds should cover negative and positive chunks");
    ok &= Expect(
        cache.GetStats().totalRawCandidateTriangleCount > 0,
        "chunk cache should accumulate raw candidate counts");

    return ok;
}

bool TestVoxelChunkCachePaddingExpandsBuildCoverage()
{
    std::vector<MeshTriangle> triangles;
    triangles.push_back(
        { Vec(4.0, 0, 0), Vec(5.0, 0, 0), Vec(4.5, 1.0, 0) }
    );

    VoxelMeshBuildOptions buildOptions;
    buildOptions.voxelSize = 1.0;
    buildOptions.clearance = 0.0;
    buildOptions.conservativeClearance = true;

    VoxelChunkCacheOptions cacheOptions;
    cacheOptions.chunkVoxelSize = 4;
    cacheOptions.buildPadding = 2.0;

    VoxelChunkCache cache;
    bool ok = Expect(
        cache.Configure(
            &triangles,
            nullptr,
            buildOptions,
            cacheOptions),
        "chunk cache should configure for padding test");

    VoxelSpace space(Vec(0, 0, 0), buildOptions.voxelSize);
    const VoxelIndex chunkZeroIndex = space.WorldToIndex(Vec(1, 0, 0));
    const VoxelIndex coreBoundaryIndex = space.WorldToIndex(Vec(3.5, 0, 0));
    const VoxelIndex adjacentChunkIndex = space.WorldToIndex(Vec(4.5, 0, 0));

    ok &= Expect(
        cache.EnsureChunkForIndex(space, chunkZeroIndex),
        "chunk zero should build with padding");
    ok &= Expect(
        space.IsInsideSearchBounds(coreBoundaryIndex),
        "core boundary index should be inside built chunk bounds");
    ok &= Expect(
        !space.IsInsideSearchBounds(adjacentChunkIndex),
        "query padding should not expand voxel write bounds");
    ok &= Expect(
        space.GetCellState(coreBoundaryIndex) != VoxelState::Free,
        "query padding should include adjacent triangle influence in core chunk");

    return ok;
}

bool TestVoxelChunkBoundsVtkExport()
{
    const std::string filePath = "test_lazy_chunk_bounds.vtk";

    std::remove(filePath.c_str());

    VoxelSpace space(Vec(0, 0, 0), 1.0);
    std::vector<VoxelChunkIndex> chunks;
    chunks.push_back({ 0, 0, 0 });
    chunks.push_back({ -1, 0, 0 });

    bool ok = Expect(
        VoxelVtkExporter::ExportChunkBoundsToVtk(
            space,
            chunks.data(),
            chunks.size(),
            4,
            filePath),
        "chunk bounds VTK export should succeed");

    std::ifstream ifs(filePath.c_str(), std::ios::in);
    ok &= Expect(ifs.is_open(), "chunk bounds VTK file should exist");

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    ok &= Expect(
        content.find("Lazy voxel chunk bounds") != std::string::npos,
        "chunk bounds VTK should include title");
    ok &= Expect(
        content.find("POINTS 16 double") != std::string::npos,
        "chunk bounds VTK should export 8 points per chunk");
    ok &= Expect(
        content.find("CELLS 2 18") != std::string::npos,
        "chunk bounds VTK should export one hexahedron per chunk");
    ok &= Expect(
        content.find("SCALARS chunk_x int 1") != std::string::npos,
        "chunk bounds VTK should export chunk index data");

    std::remove(filePath.c_str());

    return ok;
}
}

int main()
{
    bool ok = true;
    ok &= TestAppendVoxelSpaceExpandsBoundsAndPreservesState();
    ok &= TestVoxelChunkCacheBuildsEachChunkOnce();
    ok &= TestVoxelChunkCacheConfigurationFailures();
    ok &= TestVoxelChunkCacheClearResetsState();
    ok &= TestVoxelChunkCacheNegativeAndAdjacentChunks();
    ok &= TestVoxelChunkCachePaddingExpandsBuildCoverage();
    ok &= TestVoxelChunkBoundsVtkExport();
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
