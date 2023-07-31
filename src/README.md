# 固定场景地图的路径规划与轨迹优化
## 1.介绍
本工程基于ROS和rviz实现了无人机在固定场景(30*30*5m)地图下的路径规划和轨迹优化，地图中障碍物信息提前确定好，使用sdf-tools构建sdf地图，首先通过通过A*算法进行路径规划显示栅格地图下的路径，然后通过Gradient-Based Trajectory Optimizer算法实现轨迹的优化，显示轨迹优化后路径，并且模拟无人机从起点到终点的运动轨迹信息。
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
快捷键“Ctrl + G"选择目标点后首先使用A*算法进行路径规划：
<div align=center>
<img src="https://github.com/ckunj/Panning_in_room/blob/master/src/pic/A*_result.gif" width = "900" height = "600">
</div>

然后显示路径从起点到终点基于Gradient-Based Trajectory Optimizer算法的轨迹优化路线与无人机模拟运动路径信息：
<div align=center>
<img src="https://github.com/ckunj/Panning_in_room/blob/master/src/pic/Traj_result.gif" width = "900" height = "600">
</div>

基于minimum Snap闭式求解方法主要思路是：

1. 先确定轨迹阶数（比如$5$阶），再确定d向量中的约束量（pva），进而根据各段的时间分配求得$A_{total}$。
2. 根据连续性约束构造映射矩阵$M$，并确定$d$向量中哪些量是Fix(比如起点终点pva，中间点的p等)，哪些量是Free，进而构造置换矩阵$C$，并求得$K=A^{−1}MC$。
3. 计算QP目标函数中的$Q$（$minJerk/Snap$）并计算$R=K^TQK$，根据fix变量的长度将$R$拆分成$R_{FF},R_{FP},R_{PF},R_{PP}$四块。
4. 填入已知变量得到$d_F$，并根据$d_P=−R_{PP}^{−1}R_{FP}^Td_F$计算得到$d_P$。
5. 根据公式 $p=K\begin{bmatrix}d_F\\d_P\end{bmatrix}$计算得到轨迹参数$p$。

## 3.参考文献
Gao F, Lin Y, Shen S. Gradient-based online safe trajectory generation for quadrotor flight in complex environments[C]//2017 IEEE/RSJ international conference on intelligent robots and systems (IROS). IEEE, 2017: 3681-3688.

