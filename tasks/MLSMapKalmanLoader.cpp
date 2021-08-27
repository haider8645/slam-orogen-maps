#include "MLSMapKalmanLoader.hpp"

using namespace maps;

MLSMapKalmanLoader::MLSMapKalmanLoader(std::string const& name, TaskCore::TaskState initial_state)
    : MLSMapKalmanLoaderBase(name, initial_state)
{
}

MLSMapKalmanLoader::~MLSMapKalmanLoader()
{
}

void MLSMapKalmanLoader::createMLS(const pcl::PointCloud<pcl::PointXYZ>& pc,
                                  maps::grid::Vector2ui gridSize,
                                  maps::grid::Vector2d cellSize,
                                  maps::grid::Vector3d offset)
{
    mMap = maps::grid::MLSMapKalman(gridSize, cellSize, _mls_config);
    mMap.translate(offset);
    mMap.mergePointCloud(pc, base::Transform3d::Identity());
}

void MLSMapKalmanLoader::writeMLS()
{
    _map.write(mMap);
}
