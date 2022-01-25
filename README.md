# ros-pcl-tutorials

[Point Cloud Library](https://pcl.readthedocs.io/projects/tutorials/en/master/index.html)のサンプルプログラムの一部をROS環境で動作できるようにしています。
サンプルコードに解説を追加して、学びの促進を目指しています。

## 動作環境

以下の環境にて動作確認を行っています。

- ROS Melodic
  - OS: Ubuntu 20.04.3 LTS
  - ROS Distribution: Noetic ninjemys 1.15.11
  - Rviz 1.12.16
  
## インストール方法

### ソースからビルドする方法

- [ROS Wiki](https://wiki.ros.org/melodic/Installation/Ubuntu)を参照しROSをインストールします。

- `git`を使用して本パッケージをダウンロードします。

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/piropann/ros-pcl-tutorials.git
  ```

- `catkin_make`を使用して本パッケージをビルドします。

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

## 使い方

- 一つ目のターミナル
  ```bash
  roscore
  ```
  
- 二つ目のターミナル
  ```bash
  rosrun ros-pcl-tutorials <プログラム名>
  ```

- 処理を行う前の点群を表示
  ```bash
  cd ~/data
  pcl_viewer sample.pcd
  ```

- 処理を行ったあとの点群を表示
  ```bash
  cd ~/data/result
  pcl_viewer 表示したい点群.pcd
  ```

## プログラム概要

### icp.cpp

[ICP](https://pcl.readthedocs.io/projects/tutorials/en/master/iterative_closest_point.html)のプログラムです。反復最接近点アルゴリズムを使用し、２つの点群が座標変換されてるか判断します。

### passthrough.cpp

[Passthroughフィルター](https://pcl.readthedocs.io/projects/tutorials/en/master/passthrough.html)を行うプログラムです。


### transforms.cpp

点群データを座標変換するプログラムです。

### voxel_grid.cpp

[Voxel Gridフィルター](https://pcl.readthedocs.io/projects/tutorials/en/master/voxel_grid.html#voxelgrid)を行うプログラムです。

### statistical_removal.cpp

[外れ値や異常値を切り取るためのプログラム](https://pcl.readthedocs.io/projects/tutorials/en/master/statistical_outlier.html#statistical-outlier-removal)です。

### kdtree_search.cpp

[kdtree](https://pcl.readthedocs.io/projects/tutorials/en/master/kdtree_search.html#kdtree-search)を行うプログラムです。

### resampling.cpp

[平滑化](https://pcl.readthedocs.io/projects/tutorials/en/master/resampling.html#moving-least-squares)を行うプログラムです。

## 参考サイト
- [PCl GitHub](https://github.com/PointCloudLibrary)
- [PCL](https://pointclouds.org/documentation/index.html)
