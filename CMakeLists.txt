cmake_minimum_required(VERSION 3.8.2)
project( planlab_ASP VERSION 0.1 )
find_package( Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_STANDARD 14)
# dependency
add_subdirectory( deps/or-tools )
add_subdirectory( deps/json )

if(MSVC)
  add_compile_options(/W3 /WX)
else()
  #add_compile_options(-W -Wall)
  add_compile_options(-W -Wall -fno-omit-frame-pointer)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ggdb -O0 -fno-omit-frame-pointer")
endif()

LIST(APPEND SOURCES
  src/factoryWorld.cpp
  src/scheduler.cpp)
add_library( Factory SHARED ${SOURCES} )
target_include_directories( Factory PUBLIC "${CMAKE_CURRENT_LIST_DIR}/include" )
target_link_libraries( Factory PUBLIC ortools::ortools Eigen3::Eigen nlohmann_json::nlohmann_json )

add_executable( firstApp src/test.cpp )
target_link_libraries( firstApp Factory )

add_executable( Simulator src/simpleSimulator.cpp )
target_include_directories( Simulator PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include" )
target_link_libraries( Simulator Factory )

add_executable( BOMGen src/BOMGen.cpp )
target_include_directories( BOMGen PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include" )
target_link_libraries( BOMGen Factory )
