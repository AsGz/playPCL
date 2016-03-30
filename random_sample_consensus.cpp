#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/auto_io.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/surface/simplification_remove_unused_vertices.h>

#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/marching_cubes_rbf.h>

#include <pcl/surface/poisson.h>

#include <pcl/surface/organized_fast_mesh.h>

#include <pcl/surface/convex_hull.h>

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
  // initialize PointClouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
  
  if (argc < 2){
      std::cout << "./exe target.ply -sf" << std::endl;
      return -1;
  }
  // populate our PointCloud with points
  /*
  cloud->width    = 500;
  cloud->height   = 1;
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    if (pcl::console::find_argument (argc, argv, "-s") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if (i % 5 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else if(i % 2 == 0)
        cloud->points[i].z =  sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                      - (cloud->points[i].y * cloud->points[i].y));
      else
        cloud->points[i].z =  - sqrt( 1 - (cloud->points[i].x * cloud->points[i].x)
                                        - (cloud->points[i].y * cloud->points[i].y));
    }
    else
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0);
      if( i % 2 == 0)
        cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0);
      else
        cloud->points[i].z = -1 * (cloud->points[i].x + cloud->points[i].y);
    }
  }
  */
  
  //load PointCloud 
  /*
  if (pcl::io::loadPLYFile (argv[1], *cloud) < 0)  {
      std::cout << "Error loading point cloud " << argv[1] << std::endl << std::endl;
      return -1;
  }*/

  //load mesh
  pcl::PolygonMesh mesh; 
  if ( pcl::io::loadPolygonFile(argv[1], mesh) < 0 ) 
  {
      std::cout << "load mesh file " << argv[1] << " failed." << std::endl;
      return -1;
  }

  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  std::vector<int> inliers;

  std::cout << "befor random sample consensus:" << cloud->size() << std::endl;
  // created RandomSampleConsensus object and compute the appropriated model
  pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr
    model_s(new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ> (cloud));
  
  /*
  if(argc >= 3)
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }
  else*/ 
  {
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_s);
    float threshold = 0.005;
    if (argc > 4) 
    {
        std::string str = argv[4];
        threshold = std::stof(str);
    }
    std::cout << "threshold is " << threshold << std::endl;
    //点之间多少距离以内,算作inlier
    ransac.setDistanceThreshold (threshold);
    ransac.computeModel();
    ransac.getInliers(inliers);
  }

  // copies all inliers of the model computed to another PointCloud
  //pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);

  pcl::PointIndices::Ptr inliersPtr (new pcl::PointIndices ());
  inliersPtr->indices = inliers;
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (inliersPtr);
  extract.setNegative (true);
  extract.filter (*final);

 // creates the visualization object and adds either our orignial cloud or all of the inliers
  // depending on the command line arguments specified.
  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  //if (pcl::console::find_argument (argc, argv, "-f") >= 0 || pcl::console::find_argument (argc, argv, "-sf") >= 0)
  //viewer = simpleVis(final);
  //else  
  //  viewer = simpleVis(cloud);
  
  // Visualization
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  // Create two verticaly separated viewports
  int v1 (0);
  int v2 (1);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
 
  // The color we will be using
  float bckgr_gray_level = 0.0;  // Black
  float txt_gray_lvl = 1.0 - bckgr_gray_level;

  // Original point cloud is white
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h (cloud, (int) 255 * txt_gray_lvl, (int) 255 * txt_gray_lvl,
                                                                             (int) 255 * txt_gray_lvl);
  if (argc >= 3)
      viewer.addPointCloud (cloud, cloud_in_color_h, "cloud_in_v1", v1);
  else
    viewer.addPolygonMesh(mesh, "mesh0", v1);

  //pcl::PolygonMesh mesh2 = mesh;
  //pcl::surface::SimplificationRemoveUnusedVertices s;
  //std::cout << "inliers size:" <<  inliers.size() << std::endl;
  //mesh2.cloud.data.clear();
  //std::cout << "final random sample consensus:" << final->size() << std::endl;
  //pcl::toPCLPointCloud2(*final, mesh2.cloud);
  //s.simplify(mesh, mesh2, inliers);
  
  pcl::PointCloud<pcl::PointXYZ> hull;
  std::vector<pcl::Vertices> polygons;

  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud (final);
  chull.reconstruct (hull, polygons);

  pcl::PolygonMesh mesh2;
  toPCLPointCloud2 (hull, mesh2.cloud);
  mesh2.polygons = polygons;

  /*
  pcl::PointCloud<pcl::PointNormal>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointNormal> ());
  pcl::fromPCLPointCloud2 (mesh2.cloud, *xyz_cloud);
  std::cout << "after random sample consensus:" << xyz_cloud->size() << std::endl;

  pcl::MarchingCubesHoppe<pcl::PointNormal>  mc;

  mc.setIsoLevel (0.000);
  mc.setGridResolution (50.0,50.0, 50.0);
  mc.setPercentageExtendGrid (0.000);
  mc.setInputCloud (xyz_cloud);

  mc.reconstruct (mesh2);
  */
  /*
  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth (8);
  poisson.setSolverDivide (8);
  poisson.setIsoDivide (8);
  poisson.setPointWeight (4.0000);
  poisson.setInputCloud (xyz_cloud);

  poisson.reconstruct (mesh2);
  */

  // Transformed point cloud is green
  pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (final, 20, 180, 20);
  if (argc >= 3)
    viewer.addPointCloud (final, cloud_tr_color_h, "cloud_tr_v1", v2);
  else
    viewer.addPolygonMesh(mesh2, "mesh1", v2);

  pcl::io::savePLYFileASCII("cut_result.ply", *final);
  while (!viewer.wasStopped ())
  {
    viewer.spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
  return 0;
 }
