#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/conversions.h>
#include <pcl/io/auto_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/TextureMesh.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::shared_ptr<pcl::visualization::PCLVisualizer>
simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  //viewer->addCoordinateSystem (1.0, "global");
  viewer->initCameraParameters ();
  return (viewer);
}

int
main(int argc, char** argv)
{
    if (argc < 2 ) 
    {
        std::cout << "./bin mesh.ply or  ./bin mesh.obj texturepath" << std::endl;
        return -1;
    } 
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument  (argc, argv, ".obj");
    if ( filenames.size() > 0 )
    {
        //oops.... only support obj format texture mesh....
        std::cout << "load obj mesh" << std::endl;
        pcl::TextureMesh mesh;
        if ( pcl::io::loadPolygonFileOBJ(argv[filenames[0]], mesh) < 0 )
        //if ( pcl::io::load(argv[filenames[0]], mesh) < 0 )
        {
           std::cout << "load mesh file " << argv[1] << " failed." << std::endl;
           return -1;
        }
        std::cout << "texture file " << argv[2] << std::endl;
        viewer->addTextureMesh(mesh, argv[2]);
    }
    else
    {
        pcl::PolygonMesh mesh; 
        if ( pcl::io::loadPolygonFile(argv[1], mesh) < 0 ) 
        {
            std::cout << "load mesh file " << argv[1] << " failed." << std::endl;
            return -1;
        }
        viewer->addPolygonMesh(mesh,"meshes",0);
    }
   
       
    //viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
    /*
    //show the point cloud from mesh
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = simpleVis(cloud);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
         boost::this_thread::sleep (boost::posix_time::microseconds (100000));            
    }
    */
    //pcl::toPCLPointCloud2 
    return 0;
} 
