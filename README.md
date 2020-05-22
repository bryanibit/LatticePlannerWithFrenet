## Usage

1. Download or clone all project and don't forget submodule(mapplotlib-cpp)
2. Renew a dir and compile
3. make sure cmakelists.txt has the right library location, such as Eigen include directory.
4. Just run it

```
git clone --recurse-submodules <**.git>
mkdir build
cd build
cmake ..
make -j
./frenet
```

## Source

The project is achieved using Eigen and CPP instead of Python and Numpy.And it refers to a GitHub project called [**AtsushiSakai/PythonRobotics**](https://github.com/AtsushiSakai/PythonRobotics/tree/master/PathPlanning/FrenetOptimalTrajectory). At the same time, 
it adds some other functions, such as transforming from Cartesian coordinate to Frenet and modifies some functions.

## Visualization

The submodule matplotlib-cpp dir is used for visulization. You can switch `matplotshow` to *1* for matplotlib showing in main.cpp. The default visulization is accessible using OpenCV.
