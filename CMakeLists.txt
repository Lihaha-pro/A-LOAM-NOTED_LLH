cmake_minimum_required(VERSION 2.8.3)
# cmake_minimum_required规定cmake软件的最低版本,这行命令是可选的，可以不写这句话,
# 但在有些情况下，如果CMakeLists.txt文件中使用了一些高版本cmake特有的一些命令时,
# 就需要加上这一行，提醒用户升级到该版本之后再执行cmake。
project(aloam_velodyne)
# project(<projectname>)命令
# 语法：project(projectname [cxx] [c] [java])
# 可以指定工程采用的语言，选项分别表示：C++, C, java， 如不指定默认支持所有语言。

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# SET命令——用于设置变量，相当于为变量取别名
# SET(CMAKE_BUILE_TYPE DEBUG) 设置编译类型debug 或者release。
# debug 版会生成相关调试信息，可以使用GDB 进行调试；release不会生成调试信息。
# 注意：当无法进行调试时查看此处是否设置为debug.
# SET(CMAKE_C_FLAGS_DEBUG “-O3 -Wall -g”) 设置编译器的类型
# 表示采用-O3的优化等级，-Wall表示warning也会导致编译错误，-g参数表示带调试信息
# CMAKE_C_FLAGS_DEBUG ---- C 编译器
# CMAKE_CXX_FLAGS_DEBUG ---- C++ 编译器

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

#find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
# find_package（XXX）命令寻找包，比较好理解
include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  DEPENDS EIGEN3 PCL 
  INCLUDE_DIRS include
)


add_executable(ascanRegistration src/scanRegistration.cpp)
target_link_libraries(ascanRegistration ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(alaserOdometry src/laserOdometry.cpp)
target_link_libraries(alaserOdometry ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(alaserMapping src/laserMapping.cpp)
target_link_libraries(alaserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

add_executable(kittiHelper src/kittiHelper.cpp)
target_link_libraries(kittiHelper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
# add_executable（XXX XXX）表示编译可执行文件与对应的参与编译的文件路径
# target_link_libraries表示编译过程中链接的路径文件或库



