### 介绍
None

### 目录组织
|          软件包          |                             描述                             |
| :----------------------: | :----------------------------------------------------------: |
|    **scene_loading**     |                       加载场景和机器人                       |
|      **laser_slam**      |      ros自带激光slam工具包，包括Gmapping,Karto,Hector等      |
|    **navigation_2d**     |      ros自带2d导航工具包，包括AMCL、Odometry Navigation      |
|      **rtab_slam**       | rtab_ros实现激光雷达与RGB-D相机的同步定位与建图，及稠密点云地图构建 |
| **b2_formation_control** |             基于行为法改进的多移动机器人编队控制             |
|  **unmanned_logistics**  |                     无人仓储物流项目展示                     |


### 下载和编译

环境要求：`ubuntu18.04、melodic、gazebo9.0、Qt5.9.9`

1. 克隆或下载`PPIP`到工作空间的`/src`目录下，例如`~/catkin_ws/src`:

   ```shell
   mkdir -p catkin_ws/src
   cd catkin_ws/src
   git clone git@gitee.com:puyuyuyu/ppip.git
   ```

2. 安装依赖：

   ```shell
   rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
   ```

3. 编译并刷新环境：

   ```shell
   catkin_make
   source catkin_ws/src/devel/setup.bash 
   # if zsh, use ' source catkin_ws/src/devel/setup.zsh '
   rospack profile
   ```

### 使用说明
使用`scene_loading`包下的`launch`文件加载场景，使用`b2_formation_control`包下的可执行文件进行仿真演示