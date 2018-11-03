cmake_minimum_required(VERSION 3.8.2)
project( planlab_ASP VERSION 0.1 )
find_package( Eigen3 3.3 REQUIRED NO_MODULE)

if(MSVC)
  add_compile_options(/W3 /WX)
else()
  add_compile_options(-W -Wall)
  #add_compile_options(-W -Wall)
endif()

add_subdirectory( deps/or-tools )
add_executable( firstApp src/test.cpp )
target_link_libraries( firstApp ortools::ortools Eigen3::Eigen )
