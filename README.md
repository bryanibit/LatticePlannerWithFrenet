## Usage

`dev` branch denotes the vehicle follows a reference line (*y = 10sin(10x)*). And `master` branch represents a complete process including obstacle avoidance. 

1. Download or clone all project and don't forget submodule(mapplotlib-cpp)
2. Renew a dir and compile
3. make sure CMakelists.txt has the right library location, such as Eigen include directory.
4. Just run it

```
git clone --recurse-submodules <**.git>
git checkout dev / master
mkdir build
cd build
cmake ..
make -j
./frenet
```

## Source

The project is achieved using Eigen and C++11 instead of Python and Numpy. The original project is from a GitHub project called [**AtsushiSakai/PythonRobotics**](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory) which is Python code. 

## Visualization

The submodule matplotlib-cpp dir is used for visulization. You can also choose OpenCV to show the result. You can only uncomment the OpenCV showing code in main.cpp.
