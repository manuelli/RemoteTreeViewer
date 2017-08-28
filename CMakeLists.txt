cmake_minimum_required(VERSION 3.5)
project(RemoteTreeViewer)
set(CMAKE_CXX_STANDARD 11)


message("RMT CMAKE_SOURCE_DIR ${CMAKE_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH    "${CMAKE_SOURCE_DIR}/cmake/Modules")
find_package(Eigen REQUIRED)

  include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
  )

# if you want to use these libraries call find_package on them so that it doesn't search for system installed versions
find_package(yaml-cpp)
find_package(lcm)

add_library(RemoteTreeViewerWrapper SHARED src/RemoteTreeViewerWrapper.cpp)
# target_link_libraries(RemoteTreeViewerWrapper lcm drakeRBM drakeJoints drakeShapes drakeCommon z pthread)
target_link_libraries(RemoteTreeViewerWrapper
	lcm
	z
	pthread)
install(TARGETS RemoteTreeViewerWrapper DESTINATION lib)
install(FILES src/RemoteTreeViewerWrapper.hpp DESTINATION include)

add_executable(testRemoteTreeViewerWrapper src/testRemoteTreeViewerWrapper.cpp)
target_link_libraries(testRemoteTreeViewerWrapper
	lcm
	yaml-cpp
	z
	pthread
	RemoteTreeViewerWrapper)

install(TARGETS testRemoteTreeViewerWrapper DESTINATION bin)
