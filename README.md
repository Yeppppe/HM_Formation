浦宇师兄的这个代码编译需要matplotplusplus作为依赖

git clone https://github.com/alandefreitas/matplotplusplus.git
安装说明
需要cmake 3.21以上
更新cmake的方法 https://blog.csdn.net/qq_17623363/article/details/137194500

更新好后
在matplotplusplus文件夹下:
cmake --preset=local
cmake --build --preset=local
cmake --install build/local

之后回到src下 catkin_make即可
