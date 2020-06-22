[toc]

# CloudConverter

A handy mini program for converting point cloud file.

## What this tool can do?

1. Converting `.txt` cloud file to `.pcd` file
2. Converting `.pcd` cloud file to `.txt` file
3. ...

# Dependience

This tool is tested with PCL 1.10.1. Other newer PCL version should work aslo.

# Get & Build

```bash
$ cd ~/Documents
$ git clone https://github.com/WorstCodeWay/CloudConverter.git
$ cd CloudConverter & mkdir build & cd build
$ cmake .. & make
```

# Usage

```bash
$ ./cloud_converter input_cloud_file_name [output_cloud_file_name]
```

