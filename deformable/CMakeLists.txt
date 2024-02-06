
cmake_minimum_required(VERSION 3.8)
project(testing_roloc)

include(CTest)

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads MODULE REQUIRED)

find_package(drake CONFIG REQUIRED PATHS /opt/drake)

get_filename_component(DRAKE_PYTHONPATH "${drake_DIR}" DIRECTORY)
get_filename_component(DRAKE_PYTHONPATH "${DRAKE_PYTHONPATH}" DIRECTORY)
set(DRAKE_PYTHONPATH
  "${DRAKE_PYTHONPATH}/python${PYTHON_VERSION_MAJOR_MINOR}/site-packages"
)


# find_package(drake::drake REQUIRED)

add_executable(testing_roloc testing_roloc.cpp)
target_link_libraries(testing_roloc drake::drake)