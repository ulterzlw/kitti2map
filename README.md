The repository convert [kitti odometry](http://www.cvlibs.net/datasets/kitti/eval_odometry.php) velodyne date to PCD map

PCL & Eigen is required

Download data and unzip
```
cd $PATH_TO_DATA
unzip data_odometry_poses.zip
unzip data_odometry_velodyne.zip
unzip data_odometry_calib.zip
```

run
```
mkdir build
cd build
cmake ..
make
./kitti2map $PATH_TO_DATA/dataset 07
```

```
pcl_viewer ./map.pcd
```