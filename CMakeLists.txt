cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(pcl-matrix_transform)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#icp
add_executable (interactive_icp interactive_icp.cpp)
target_link_libraries (interactive_icp ${PCL_LIBRARIES})

add_executable (matrix_transform matrix_transform.cpp)
target_link_libraries (matrix_transform ${PCL_LIBRARIES})

#
add_executable (random_sample_consensus random_sample_consensus.cpp)
target_link_libraries (random_sample_consensus ${PCL_LIBRARIES})

#
add_executable (extract_indices extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})
#
add_executable (read_write_mesh read_write_mesh.cpp)
target_link_libraries (read_write_mesh ${PCL_LIBRARIES})

#
add_executable (greedy_projection greedy_projection.cpp)
target_link_libraries (greedy_projection ${PCL_LIBRARIES})

#
add_executable (alignment_prerejective alignment_prerejective.cpp)
target_link_libraries (alignment_prerejective ${PCL_LIBRARIES})
