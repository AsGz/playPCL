# playPCL
play with PCL ( point cloud library )

from:http://www.pointclouds.org/documentation/tutorials/

#MacOS brew install
```shell
#use official tutorials 【brew install pcl】 will generate vtk link error.
brew install pcl --HEAD
```


#build step

```shell
git clone https://github.com/AsGz/playPCL
cd playPCL && mkdir build && cd build
cmake ..
make
```
