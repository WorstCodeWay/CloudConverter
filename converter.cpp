#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct POINT_XYZ
{
  float x;
  float y;
  float z;
};



bool GetData(std::vector<POINT_XYZ> &data, const std::string &filename)
{
  std::ifstream file(filename.c_str(), std::ifstream::in);

  if (!file.is_open())
  {
    std::cout << "Open point cloud file failed." << std::endl;
    return false;
  }

  float tmp = 0.0;
  while (!file.eof())
  {
    POINT_XYZ point;
    file >> point.x >> point.y >> point.z;
    point.x /= 1000.0;
    point.y /= 1000.0;
    point.z /= 1000.0;
    data.push_back(point);
  }
  return true;
}

void TxtToPcd(std::vector<POINT_XYZ> &from, pcl::PointCloud< pcl::PointXYZ >::Ptr &cloud)
{
  cloud->width = from.size();
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(cloud->width * cloud->height);
  for(int32_t num = 0; num < cloud->width; num++)
  {
    cloud->points[num].x = from[num].x;
    cloud->points[num].y = from[num].y;
    cloud->points[num].z = from[num].z;
  }
}

int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    std::cout << "Usage: \n";
    std::cout << "./cloud_converter input_cloud_file [output_cloud]\n";
    return -1;
  }

  std::string input(argv[1]);
  std::string output;
  if(argc == 3)
  {
    std::string tmp(argv[2]);
    output.assign(tmp);
  }
  else if(argc == 2)
  {
    std::string tmp(argv[1]);
    int pos = tmp.find(".txt");
    if(pos !=std::string::npos)
    {
      std::string file_name(tmp.substr(0, pos));
      file_name = file_name + ".pcd";
      output.assign(file_name);
    }
    else
    {
      output.assign("default.pcd");
    }
  }
  std::vector<POINT_XYZ> txt_data;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if(GetData(txt_data, input))
  {
    TxtToPcd(txt_data, cloud);
  }
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ>(output.c_str(), *cloud, false);

  return 0;
}
