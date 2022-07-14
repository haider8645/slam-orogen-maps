#include "MLSMapLoader.hpp"

using namespace maps;

#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>

MLSMapLoader::MLSMapLoader(std::string const& name, TaskCore::TaskState initial_state)
    : MLSMapLoaderBase(name, initial_state)
{
}

MLSMapLoader::~MLSMapLoader()
{
}

void MLSMapLoader::createMLS(const pcl::PointCloud<pcl::PointXYZ>& pc,
                             maps::grid::Vector2ui gridSize,
                             maps::grid::Vector2d cellSize,
                             maps::grid::Vector3d offset)
{
    std::cerr << "Not implemented, this Task is abstract!" << std::endl;
}

void MLSMapLoader::writeMLS()
{
    std::cerr << "Not implemented, this Task is abstract!" << std::endl;
}

bool MLSMapLoader::publishMap()
{
    if(mMapLoaded)
    {
        writeMLS();
        return true;
    }else
    {
        std::cerr << "No map loaded!" << std::endl;
        return false;
    }
}

bool MLSMapLoader::configureHook()
{
    if (! MLSMapLoaderBase::configureHook())
        return false;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PLYReader plyReader;
    if (plyReader.read(_path, cloud) < 0)
    {
        std::cerr << "Loading PLY failed!" << std::endl;
        return false;
    }

    pcl::PointXYZ mi, ma; 
    pcl::getMinMax3D (cloud, mi, ma);

    const double mls_res = _resolution;
    const double size_x = ma.x - mi.x;
    const double size_y = ma.y - mi.y;
    
    const maps::grid::Vector2ui gridSize(size_x / mls_res + 3, size_y / mls_res + 3);
    const maps::grid::Vector2d cellSize(mls_res, mls_res);
    const maps::grid::Vector2d mapSize(gridSize[0]*mls_res, gridSize[1]*mls_res);

    std::cout << "Grid-Size: " << gridSize[0] << " * " << gridSize[1] << std::endl;
    std::cout << "Map-Size: " << mapSize[0] << " * " << mapSize[1] << std::endl;
    
    const maps::grid::Vector3d offset(mi.x-1.5*mls_res, mi.y-1.5*mls_res, 0);
    std::cout << "Range(x): " << offset[0] << " - " << mapSize[0]+offset[0] << std::endl;
    std::cout << "Range(y): " << offset[1] << " - " << mapSize[1]+offset[1] << std::endl;

    createMLS(cloud, gridSize, cellSize, offset);
    mMapLoaded = true;
    return true; 
}
