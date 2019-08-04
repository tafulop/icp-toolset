#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <csv.h>
#include <fstream>

pcl::PointCloud<pcl::PointXYZ>::Ptr read_csv_data(std::string file)
{
  // read input data
  io::CSVReader<3> in(file);
  in.read_header(io::ignore_extra_column, "x", "y", "z");
  float x,y,z;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
  while(in.read_row(x,y,z)){
    pc->push_back(pcl::PointXYZ(x,y,z));
  }
  return pc;
}

inline bool file_exist (const std::string& name) {
	std::ifstream f(name.c_str());
	return f.good();
}

std::vector<std::string> read_input(int argc, char** argv){
	if(argc != 3)
	{
		std::cout << "Usage:" << std::endl 
						  << "ARG 1: pointcloud 1, which will be used as the original frame" << std::endl 
						  << "ARG 2: pointcould 2, which will be used as a transformed one" << std::endl;
	} 
	else
	{
		std::cout << "Basis pointcloud\t\t" << argv[1] << std::endl;
		std::cout << "Transformed pointcloud\t\t" << argv[2] << std::endl;
		
		// check if files exist
		if(file_exist(argv[1]) && file_exist(argv[2]))
		{
			std::vector<std::string> result;
			result.push_back(argv[1]);
			result.push_back(argv[2]);
			return result;
		}
		else
		{
			std::cout << "Cannot read input file(s), please check if you have provided the correct path." << std::endl;
		}
	}
}

int main (int argc, char** argv)
{
	std::vector<std::string> files = read_input(argc, argv);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_1 = read_csv_data(files.at(0)); 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_2 = read_csv_data(files.at(1));

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
