cmake_minimum_required(VERSION 3.8.2)
project( planlab_ASP VERSION 0.1 )
find_package( Eigen3 3.3 REQUIRED NO_MODULE)

if(MSVC)
  add_compile_options(/W3 /WX)
else()
  add_compile_options(-W -Wall)
  #add_compile_options(-W -Wall)
endif()

set(CMAKE_CXX_STANDARD 11)

add_subdirectory( deps/or-tools )
add_executable( firstApp src/test.cpp )
target_link_libraries( firstApp ortools::ortools Eigen3::Eigen )

add_executable( Simulator src/simpleSimulator.cpp )
target_include_directories( Simulator PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include" )
target_link_libraries( Simulator ortools::ortools )

add_executable( BOMGen src/BOMGen.cpp )
target_include_directories( BOMGen PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include" )
target_link_libraries( BOMGen ortools::ortools )
