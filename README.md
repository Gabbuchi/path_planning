## 路径规划算法c++实现

讲解https://space.bilibili.com/289022202
## Astar
![astar](https://github.com/Gabbuchi/path_planning/blob/main/a_star/Figure_1.png)
## Lattice
![lattice](https://github.com/Gabbuchi/path_planning/blob/main/lattice_/Figure_3.png)
## RRT算法可视化采用opencv实现，参考https://github.com/kushuaiming/planning_algorithm
## RRT已更新，不再依赖opencv
![rrt_](https://github.com/Gabbuchi/path_planning/blob/main/rrt_with_path_optimizer/Figure_1.png)
![rrt](https://github.com/Gabbuchi/path_planning/blob/main/rrt_with_path_optimizer/Figure_2.png)
## DWA算法可视化采用matplotlib-cpp实现，配置文件的读取需依赖yaml-cpp库
![dwa](https://github.com/Gabbuchi/path_planning/blob/main/dwa/dwa.png)
## dijkstra算法中有bfs，dfs的实现，RunCode即可
## 三次样条曲线
![cubicspline](https://github.com/Gabbuchi/path_planning/blob/main/cubic_spline2D/Figure_1.png)

## 编译
    ```shell
    cd xxx
    ##若有build文件##
    //rm -rf build
    ################
    mkdir build
    cd build
    cmake ..
    make
    ./xxx_
