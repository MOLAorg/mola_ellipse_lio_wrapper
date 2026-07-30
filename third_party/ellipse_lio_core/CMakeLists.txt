# ellipse_lio_core: the EllipseLIO algorithm, stripped of ROS and MOLA
# dependencies. See ../../THIRD_PARTY_NOTICES.md.
find_package(Boost REQUIRED)
find_package(OpenMP REQUIRED)

add_library(ellipse_lio_core STATIC
  src/ImuProcess.cpp
  src/LidarProcess.cpp
  src/EllipseLioCore.cpp
)

target_include_directories(ellipse_lio_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/../          # for <ikfom/...> and <ioctree/...>
)

target_link_libraries(ellipse_lio_core PUBLIC
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  Boost::boost
  OpenMP::OpenMP_CXX
)
# See the comment at the top of fast_lio2_core/CMakeLists.txt (same
# reasoning applies here: pcl::VoxelGrid there, pcl::transformPointCloud /
# compute3DCentroid / computeCovarianceMatrixNormalized here) -- forces PCL
# template instantiation from headers instead of linking the precompiled
# system libpcl_*.so, to avoid an Eigen aligned-allocator ABI mismatch
# observed on this machine.
target_compile_definitions(ellipse_lio_core PUBLIC PCL_NO_PRECOMPILE)
target_include_directories(ellipse_lio_core PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(ellipse_lio_core PUBLIC ${PCL_DEFINITIONS})

set_target_properties(ellipse_lio_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_ellipse_lio_core test/test_ellipse_lio_core.cpp)
    target_link_libraries(test_ellipse_lio_core ellipse_lio_core)
  endif()
endif()
