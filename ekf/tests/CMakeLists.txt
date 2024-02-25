set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

add_executable(example_ekf example_ekf.cpp)
target_link_libraries(example_ekf
  PRIVATE
    Filters::Ekf
)
target_compile_options(example_ekf PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=c++17>)
