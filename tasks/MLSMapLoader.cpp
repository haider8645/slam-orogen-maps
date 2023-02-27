#include "MLSMapLoader.hpp"

using namespace maps;

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
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
        _cloud.write(mPointcloud);
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

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::PLYReader plyReader;
    if (plyReader.read(_path, pcl_cloud) < 0)
    {
        pcl::PCDReader pcdReader;
        if(pcdReader.read(_path, pcl_cloud) < 0)
        {
            std::cerr << "Loading Pointcloud failed!" << std::endl;
            return false;
        }
    }

    // Convert to base-type
    mPointcloud.time.fromMicroseconds(pcl_cloud.header.stamp);
    mPointcloud.points.reserve(pcl_cloud.size());
    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = pcl_cloud.begin(); it < pcl_cloud.end(); it++)
    {
        base::Point p;
        p[0] = it->x;
        p[1] = it->y;
        p[2] = it->z;
        mPointcloud.points.push_back(p);
    }

    // Set grid parameters
    pcl::PointXYZ mi, ma; 
    pcl::getMinMax3D (pcl_cloud, mi, ma);

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

    createMLS(pcl_cloud, gridSize, cellSize, offset);
    mMapLoaded = true;
    return true; 
}
