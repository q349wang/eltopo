cmake_minimum_required(VERSION 3.16)
project(talpa)

set(CMAKE_BUILD_TYPE "Release")

list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_CGAL              "Use CGAL"             ON)
include(libigl)

add_subdirectory(FastAndRobustMeshArrangements)

# Add your project files
include_directories( "${PROJECT_SOURCE_DIR}/common/" )
include_directories( "${PROJECT_SOURCE_DIR}/common/tunicate" )
include_directories( "${PROJECT_SOURCE_DIR}/common/newsparse" )
include_directories( "${PROJECT_SOURCE_DIR}/eltopo3d/" )
include_directories( "${PROJECT_SOURCE_DIR}/talpa/" )
include_directories( "${PROJECT_SOURCE_DIR}/talpa/drivers/" )
include_directories( "${PROJECT_SOURCE_DIR}/talpa/curlnoise/" )
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

file(GLOB SRC_FILES "${PROJECT_SOURCE_DIR}/talpa/*.cpp"
                    "${PROJECT_SOURCE_DIR}/talpa/drivers/*.cpp"
                    "${PROJECT_SOURCE_DIR}/talpa/curlnoise/*.cpp")

file(GLOB ELTOPO_SRCFILES             "${PROJECT_SOURCE_DIR}/eltopo3d/*.cpp"
                                      "${PROJECT_SOURCE_DIR}/common/*.cpp"
                                      "${PROJECT_SOURCE_DIR}/common/newsparse/*.cpp"
                                      "${PROJECT_SOURCE_DIR}/tunicate/*.cpp")

add_definitions(-DMESH_MELD -D__LITTLE_ENDIAN__ -DUSE_FORTRAN_BLAS -DGRID_100 -DEXTRA_PASSES -DREMOVE_RIZ -fPIC -DGL_GLEXT_PROTOTYPES)

add_executable(${PROJECT_NAME} ${SRC_FILES} ${ELTOPO_SRCFILES})

message(STATUS "OPENGL libs in ${OPENGL_LIBRARIES}")
message(STATUS "GLUT libs in ${GLUT_LIBRARY}")


target_link_libraries(${PROJECT_NAME} PUBLIC igl::core igl::cgal
  fastmesharrange
  ${BLAS_LIBRARIES} 
  ${LAPACK_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARY})