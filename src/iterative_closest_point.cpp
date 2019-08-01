#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <csv.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr read_csv_data(std::string file)
{
  // read input data
  io::CSVReader<9> in(file);
  std::cout << "file lines: " << in.get_file_line() << std::endl;
  //in.read_header(io::ignore_extra_column, "vendor", "size", "speed");
  std::string time;
  int id, intensity, pid;
  float x,y,z, azimuth, range;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
  while(in.read_row(time,id,intensity, x,y,z,azimuth,range,pid)){
    // do stuff with the data
    // std::cout << "x: " << x << std::endl << "y: " << y  << std::endl << "z: " << z << std::endl;
    pc->push_back(pcl::PointXYZ(x,y,z));
  }
  return pc;
}

int main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_1 = read_csv_data("data/input.csv"); 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_2 = read_csv_data("data/input_tf_noise.csv");
  
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in_2);
  icp.setInputTarget(cloud_in_1);
 
  // Set the max correspondence distance to 5cm
  //icp.setMaxCorrespondenceDistance (0.5);

  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations (1500);
  
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  std::cout << "Has converged:" << icp.hasConverged() << std::endl;
  std::cout << "Fitness score: " <<  icp.getFitnessScore() << std::endl;
  std::cout << "Final transformation: " << std::endl << std::endl << icp.getFinalTransformation() << std::endl;

 return (0);
}
