# 固定场景地图的A*路径规划
## 1.介绍
本工程基于ROS和rviz实现了无人机在固定场景下的A*路径规划，后续将通过Gradient-Based Trajectory Optimizer算法实现轨迹的优化
## 2.运行工程并展示结果
克隆工程并且编译:
```
  cd ~/catkin_ws/src
  git clone https://github.com/ckunj/Panning_in_room.git
  cd ../
  catkin_make
  source ~/catkin_ws/devel/setup.bash
```
开始运行:
```
  roslaunch traj_planner traj.launch
```
快捷键“Ctrl + G"选择目标点后 显示规划路径：
  <div align=center>
  <img src="https://github.com/ckunj/Panning_in_room/tree/master/src/pic/A*_result.gif" width = "360" height = "360">
  </div>
