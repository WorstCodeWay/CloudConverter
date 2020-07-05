#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

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
enum FILE_TYPE
{
  FILE_TYPE_NONE = 0,
  FILE_TYPE_TXT  = 1,
  FILE_TYPE_PLY  = 2,
  FILE_TYPE_PCD  = 3
};
enum TXT_POINT_TYPE
{
  TXT_POINT_TYPE_NONE   = 0,
  TXT_POINT_TYPE_XYZ    = 1,
  TXT_POINT_TYPE_XYZI   = 2,
  TXT_POINT_TYPE_XYZRGB = 3
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
  TXT_POINT_TYPE file_type = TXT_POINT_TYPE_NONE;

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
      file_type = TXT_POINT_TYPE_XYZ;
      break;
    case 4:
      file_type = TXT_POINT_TYPE_XYZI;
      break;
    case 6:
      file_type = TXT_POINT_TYPE_XYZRGB;
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
FILE_TYPE GetFileType(const std::string &file_name, std::string &name, std::string &suffix)
{
  FILE_TYPE type = FILE_TYPE_NONE;
  std::vector<std::string> tokens;
  name.assign(file_name);

  boost::split(tokens, file_name, boost::is_any_of ("."), boost::token_compress_on);
  if(tokens[tokens.size() -1 ] == "txt")
  {
    type = FILE_TYPE_TXT;
    suffix.assign(".txt");
    boost::algorithm::erase_last_copy(name, ".txt");
  }
  else if(tokens[tokens.size() -1 ] == "pcd")
  {
    type = FILE_TYPE_PCD;
    suffix.assign(".pcd");
    boost::algorithm::erase_last_copy(name, ".pcd");
  }
  else if(tokens[tokens.size() -1 ] == "ply")
  {
    type = FILE_TYPE_PLY;
    suffix.assign(".ply");
    boost::algorithm::erase_last_copy(name, ".ply");
  }
  
  return type;
}
int main(int argc, char *argv[])
{
  if(argc < 2)
  {
    std::cout << "Usage: \n";
    std::cout << "./cloud_converter input_cloud_file [output_cloud]\n";
    return -1;
  }


  std::string input(argv[1]), input_suffix, input_name;
  std::string output;

  FILE_TYPE input_type = GetFileType(input, input_name, input_suffix);
  FILE_TYPE output_type = FILE_TYPE_PCD;

  if(input_type == FILE_TYPE_NONE)
  {
    std::cout << "Invalid input file type. Must be one of .txt .ply .pcd." << std::endl;
    return -1;
  }

  if(argc == 3)
  {
    std::string tmp;
    output.assign(argv[2]);

    output_type = GetFileType(output, tmp, tmp);
  }
  else if(argc == 2)
  {
    output.assign(input_name + ".pcd");
    output_type = FILE_TYPE_PCD;
  }

  if(output_type == FILE_TYPE_NONE)
  {
    std::cout << "Invalid output file type. Must be one of .txt .ply .pcd." << std::endl;
    return -1;
  }

  std::vector<POINT_XYZ> txt_data_xyz;
  std::vector<POINT_XYZI> txt_data_xyzi;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr gray_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDWriter writer;

  if(input_type == FILE_TYPE_TXT && output_type == FILE_TYPE_PCD)
  {
    std::ifstream file(input.c_str(), std::ifstream::in);
    if (!file.is_open())
    {
      std::cout << "Open point cloud file failed." << std::endl;
      return -1;
    }
    TXT_POINT_TYPE point_type = GetTxtPointType(file);
    
    if(point_type == TXT_POINT_TYPE_XYZ)
    {
      GetData(txt_data_xyz, file);
      TxtToPcd(txt_data_xyz, cloud);

      writer.write<pcl::PointXYZ>(output.c_str(), *cloud, false);
    }
    else if(point_type == TXT_POINT_TYPE_XYZI)
    {
      GetData(txt_data_xyzi, file);
      TxtToPcd(txt_data_xyzi, gray_cloud);

      writer.write<pcl::PointXYZI>(output.c_str(), *gray_cloud, false);
    }
    else
    {
      cout << "file type has not been supported right now.\n";
    }
    file.close();
  }
  else if(input_type == FILE_TYPE_PLY && output_type == FILE_TYPE_PCD)
  {
    pcl::PLYReader plyreader;
    plyreader.read<pcl::PointXYZRGB>(input, *color_cloud);

    writer.write<pcl::PointXYZRGB>(output.c_str(), *color_cloud, false);
  }

  return 0;
}
