#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

struct POINT_XYZ
{
  float x;
  float y;
  float z;
};
struct POINT_XYZI
{
  float x;
  float y;
  float z;
  uint32_t intensity;
};
enum TXT_POINT_TYPE
{
  NONE = 0,
  XYZ = 1,
  XYZI = 2,
  XYZRGB = 3
};

void string_split(const string &str, vector<string> &results, const string &delimiters = " ")
{
  // Skip delimiters at beginning
  string::size_type lastPos = str.find_first_not_of(delimiters, 0);
  
  // Find first non-delimiter
  string::size_type pos = str.find_first_of(delimiters, lastPos);
  while (string::npos != pos || string::npos != lastPos) {
      // Found a substr, add it to the vector
      results.push_back(str.substr(lastPos, pos - lastPos));
      // Skip delimiters
      lastPos = str.find_first_not_of(delimiters, pos);
      // Find next non-delimiter
      pos = str.find_first_of(delimiters, lastPos);
  }
}

uint32_t CountColsOfTxt(const string one_line_from_txt_file)
{
  vector<string> columns;
  string_split(one_line_from_txt_file, columns);
  
  return columns.size();
}

TXT_POINT_TYPE GetTxtPointType(std::ifstream &file)
{
  TXT_POINT_TYPE file_type = NONE;

  char one_line[500];
  file.getline(one_line, sizeof(one_line));
  std::string line(one_line);

  uint32_t cols = CountColsOfTxt(one_line);

  switch(cols)
  {
    case 0:
    case 1:
    case 2:
    case 5:
      break;
    case 3:
      file_type = XYZ;
      break;
    case 4:
      file_type = XYZI;
      break;
    case 6:
      file_type = XYZRGB;
      break;
    default:
     break;
  }

  file.clear();
  file.seekg(0, ios::beg); // 指向begin处

  return file_type;
}
bool GetData(std::vector<POINT_XYZ> &data, std::ifstream &file)
{
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
bool GetData(std::vector<POINT_XYZI> &data, std::ifstream &file)
{
  while (!file.eof())
  {
    POINT_XYZI point;
    file >> point.x >> point.y >> point.z >> point.intensity;
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
void TxtToPcd(std::vector<POINT_XYZI> &from, pcl::PointCloud< pcl::PointXYZI >::Ptr &cloud)
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
    cloud->points[num].intensity = from[num].intensity;
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

  std::ifstream file(input.c_str(), std::ifstream::in);
  if (!file.is_open())
  {
    std::cout << "Open point cloud file failed." << std::endl;
    return -1;
  }

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
  std::vector<POINT_XYZ> txt_data_xyz;
  std::vector<POINT_XYZI> txt_data_xyzi;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gray_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  TXT_POINT_TYPE point_type = GetTxtPointType(file);
  pcl::PCDWriter writer;
  if(point_type == XYZ)
  {
    GetData(txt_data_xyz, file);
    TxtToPcd(txt_data_xyz, cloud);

    writer.write<pcl::PointXYZ>(output.c_str(), *cloud, false);
  }
  else if(point_type == XYZI)
  {
    GetData(txt_data_xyzi, file);
    TxtToPcd(txt_data_xyzi, gray_cloud);

    writer.write<pcl::PointXYZI>(output.c_str(), *gray_cloud, false);
  }
  else
  {
    cout << "file type has not been supported right now.\n";
  }

  return 0;
}
