#include <iostream>
//#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/auto_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  if (argc < 2)
  {
    printf ("Usage :\n");
    printf ("\t\t%s target.ply\n", argv[0]);
    PCL_ERROR ("Provide one ply file.\n");
    return (-1);
  }

  if (pcl::io::load (argv[1], *cloud) < 0)
  {
    PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
    return (-1);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  // The number of neighbors to analyze for each point 
  sor.setMeanK (50);
  // the standard deviation multiplier
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;
  pcl::io::savePLYFile("sor_inliers.ply", *cloud_filtered);

  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  pcl::io::savePLYFile("sor_outliers.ply", *cloud_filtered);  

  return (0);
}