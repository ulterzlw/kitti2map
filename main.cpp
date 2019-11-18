#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cout << "usage: kitti2map $PATH_TO_DATASET SEQUENCE" << std::endl;
    return -1;
  }
  std::string kitti_path = argv[1];
  std::string sequence = argv[2];
  std::string data_path = kitti_path + "/sequences/" + sequence;
  std::string velodyne_path = data_path + "/velodyne";
  std::string calibration_file = data_path + "/calib.txt";
  std::string time_file = data_path + "/times.txt";
  std::string pose_file = kitti_path + "/poses/" + sequence + ".txt";

  std::ifstream calibration_fin(calibration_file);
  std::string value;
  for (int i = 0; i < 4; i++) {
    std::getline(calibration_fin, value);
  }
  Eigen::Matrix<double, 4, 3> matrix_in;
  Eigen::Matrix<double, 4, 4> t_cam_velo(
      Eigen::Matrix<double, 4, 4>::Identity());

  std::getline(calibration_fin, value, ' ');
  for (int i = 0; i < 11; i++) {
    std::getline(calibration_fin, value, ' ');
    matrix_in(i) = stof(value);
  }
  std::getline(calibration_fin, value, '\n');
  matrix_in(11) = stof(value);
  t_cam_velo.block<3, 4>(0, 0) = matrix_in.transpose();
  //  std::cout << t_cam_velo << std::endl;

  std::ifstream time_fin(time_file);

  std::ifstream pose_fin(pose_file);

  int line_num = 0;

  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
  while (std::getline(time_fin, value)) {
    std::stringstream velo_ss;
    velo_ss << velodyne_path << "/" << std::setfill('0') << std::setw(6)
            << line_num << ".bin";

    int32_t num = 1000000;
    float *data = (float *)malloc(num * sizeof(float));

    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    FILE *stream;
    stream = fopen(velo_ss.str().c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;

    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (int32_t i = 0; i < num; i++) {
      pcl::PointXYZI point_xyzi;
      point_xyzi.x = *px;
      point_xyzi.y = *py;
      point_xyzi.z = *pz;
      point_xyzi.intensity = *pr;
      point_cloud->push_back(point_xyzi);
      px += 4;
      py += 4;
      pz += 4;
      pr += 4;
    }
    fclose(stream);
    delete data;

    for (int i = 0; i < 11; i++) {
      std::getline(pose_fin, value, ' ');
      matrix_in(i) = stof(value);
    }
    std::getline(pose_fin, value, '\n');
    matrix_in(11) = stof(value);
    Eigen::Matrix4d pose_cam(Eigen::Matrix4d::Identity());
    pose_cam.block<3, 4>(0, 0) = matrix_in.transpose();
    Eigen::Matrix4d pose_velo = pose_cam * t_cam_velo;
    pcl::transformPointCloud(*point_cloud, *point_cloud, pose_velo);

    pcl::VoxelGrid<pcl::PointXYZI> sor;
    sor.setLeafSize(.5, .5, .5);
    pcl::PointCloud<pcl::PointXYZI> filtered;
    sor.setInputCloud(point_cloud);
    sor.filter(filtered);
    *map += filtered;
    if (line_num % 100 == 1) {
      sor.setInputCloud(map);
      sor.filter(filtered);
      *map = filtered;
    }
    std::cout << line_num++ << '\r' << std::flush;
  }
  std::cout<<std::endl;
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setLeafSize(.5, .5, .5);
  pcl::PointCloud<pcl::PointXYZI> filtered;
  sor.setInputCloud(map);
  sor.filter(filtered);
  *map = filtered;

  pcl::transformPointCloud(*map, *map, t_cam_velo.inverse().cast<float>());
  pcl::io::savePCDFile("map.pcd", *map);
  return 0;
}