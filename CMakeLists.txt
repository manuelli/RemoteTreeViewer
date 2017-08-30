cmake_minimum_required(VERSION 3.5)
project(RemoteTreeViewer)
set(CMAKE_CXX_STANDARD 11)


message("RMT CMAKE_SOURCE_DIR ${CMAKE_SOURCE_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")
find_package(Eigen REQUIRED)

include_directories(SYSTEM
	${EIGEN_INCLUDE_DIRS}
)

# if you want to use these libraries call find_package on them so that it doesn't search for system installed versions
find_package(yaml-cpp)
find_package(lcm REQUIRED)
#include_directories(${LCM_INCLUDE_DIRS})

set(HEADERS
		include/
		src/
		)
set(SOURCES
		src/RemoteTreeViewerWrapper.cpp
		src/geometry.cc)
set(PUBLIC_HEADERS
		include/)

add_library(RemoteTreeViewerWrapper SHARED ${SOURCES})
#set_target_properties(RemoteTreeViewerWrapper
#											PROPERTIES PUBLIC_HEADER ${PUBLIC_HEADERS})
target_include_directories(RemoteTreeViewerWrapper PUBLIC ${HEADERS})
target_link_libraries(RemoteTreeViewerWrapper
	lcm
	z
	pthread)

install(TARGETS RemoteTreeViewerWrapper
				LIBRARY DESTINATION lib)
install(DIRECTORY ${PUBLIC_HEADERS} DESTINATION include)

add_executable(testRemoteTreeViewerWrapper src/testRemoteTreeViewerWrapper.cpp)
target_include_directories(testRemoteTreeViewerWrapper PUBLIC
													${PUBLIC_HEADERS})
target_link_libraries(testRemoteTreeViewerWrapper
	yaml-cpp
	RemoteTreeViewerWrapper)


install(TARGETS testRemoteTreeViewerWrapper
				RUNTIME DESTINATION bin)
