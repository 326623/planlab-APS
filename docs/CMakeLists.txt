# first we can indicate the documentation build as an option and set it to ON by default
option(BUILD_DOC "Build documentation" ON)
cmake_minimum_required(VERSION 3.5)
project(PlanlabAPSDoc)

# check if Doxygen is installed
find_package(Doxygen)
if (DOXYGEN_FOUND)
  # set input and output files
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
  set(PLANLAB_APS_PROJECT_NAME "Planlab-APS")
  set(PLANLAB_APS_VERSION "0.0")
  set(PLANLAB_APS_DOXY_INPUT "${CMAKE_CURRENT_SOURCE_DIR}/../src\" \"${CMAKE_CURRENT_SOURCE_DIR}/../include")
  set(PLANLAB_APS_ENABLE_SECTIONS "\"Constraints\"")

  set(PLANLAB_APS_INTERNAL_DOCUMENTATION "NO")

  # request to configure the file
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT})
  message("Doxygen build started")

  # note the option ALL which allows to build the docs together with the application
  add_custom_target( doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM )
else (DOXYGEN_FOUND)
  message("Doxygen need to be installed to generate the doxygen documentation")
endif (DOXYGEN_FOUND)
