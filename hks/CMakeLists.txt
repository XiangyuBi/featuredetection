cmake_minimum_required (VERSION 2.8)

#set (CMAKE_BUILD_TYPE Release)

project (hks)

##############################################################################
# Required packages                                                          #
##############################################################################

# To handle the point clouds, compute normals and KdTree
find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories (${PCL_LIBRARY_DIRS})

# Depending on the installation the package name might be Eigen3
find_package (Eigen3 REQUIRED)
include_directories (${Eigen3_INCLUDE_DIRS})

# CGAL Library used only to compute the Voronoi diagram
find_package (CGAL REQUIRED)
include_directories (${CGAL_INCLUDE_DIRS})

find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})
include( ${CGAL_USE_FILE} )
include( CGAL_CreateSingleSourceCGALProgram )

##############################################################################
# C++11 support required                                                     #
##############################################################################

#include_directories (include)
add_definitions (-std=c++0x ${CGAL_CXX_FLAGS_INIT})

file(GLOB_RECURSE sources src/*.h)
#add_executable(hks hks.cpp ${sources})
create_single_source_cgal_program("hks.cpp")
target_link_libraries (hks
    ${Boost_LIBRARIES}  
	#    ${CGAL_LIBRARY} 
	# ${CGAL_3RD_PARTY_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Eigen3_LIBRARIES})
