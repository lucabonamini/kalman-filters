cmake_minimum_required(VERSION 3.15)

project(ekf)

set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)

option(USE_TESTS "Compile test executables" ON)

find_package(Eigen3 REQUIRED)

add_library(Ekf SHARED src/ekf.cpp)
add_library(Filters::Ekf ALIAS Ekf)
target_include_directories(Ekf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>    
)
target_link_libraries(Ekf
    PRIVATE
      Eigen3::Eigen
)
target_compile_options(Ekf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)

if (USE_TESTS)
    set(CMAKE_CXX_CLANG_TIDY "")
    message("Compiling Ekf tests")
    add_subdirectory(tests)
endif()

install(TARGETS Ekf
    EXPORT Ekf-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

install(EXPORT Ekf-export
    FILE EkfTargets.cmake
    NAMESPACE Filters::
    DESTINATION lib/cmake/ekf
)
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
    DESTINATION "${CMAKE_INSTALL_INCLUDE_DIR}/ekf"
)
install(
    FILES
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ModelsConfig.cmake
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/ekf"
)