cmake_minimum_required(VERSION 3.1)
# C++11 support
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(g2o)

ADD_EXECUTABLE(cauchy_example
./cauchy_example.cpp
./EdgeIDManager.cpp
./GraphManager.cpp
./VertexIDManager.cpp)

find_package(Eigen3 REQUIRED)
if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

INCLUDE_DIRECTORIES(${Base_SOURCE_DIR}/include)

TARGET_LINK_LIBRARIES(cauchy_example
  g2o::solver_cholmod
g2o::core
g2o::stuff

  ${G2O_SOLVER_CHOLMOD}
  ${CHOLMOD_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_EIGEN3_EIGEN_TARGET}
  )

