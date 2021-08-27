#include "MLSMapSlopeLoader.hpp"

using namespace maps;

MLSMapSlopeLoader::MLSMapSlopeLoader(std::string const& name, TaskCore::TaskState initial_state)
    : MLSMapSlopeLoaderBase(name, initial_state)
{
}

MLSMapSlopeLoader::~MLSMapSlopeLoader()
{
}

void MLSMapSlopeLoader::createMLS(const pcl::PointCloud<pcl::PointXYZ>& pc,
                                  maps::grid::Vector2ui gridSize,
                                  maps::grid::Vector2d cellSize,
                                  maps::grid::Vector3d offset)
{
    mMap = maps::grid::MLSMapSloped(gridSize, cellSize, _mls_config);
    mMap.translate(offset);
    mMap.mergePointCloud(pc, base::Transform3d::Identity());
}

void MLSMapSlopeLoader::writeMLS()
{
    _map.write(mMap);
}
