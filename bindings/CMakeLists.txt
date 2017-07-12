cmake_minimum_required(VERSION 2.6)

option(BUILD_BINDINGS "Build python bindings" OFF)

if(BUILD_BINDINGS)
    find_package(SWIG REQUIRED)
    include(${SWIG_USE_FILE})
    find_package(PythonLibs)

    file(GLOB source *.i)
    
    include_directories(${YARP_INCLUDE_DIRS}
                        ${EVENTDRIVENLIBS_INCLUDE_DIRS}
			${PYTHON_INCLUDE_PATH}
			${CMAKE_CURRENT_SOURCE_DIR})
    set(CMAKE_SWIG_FLAGS "")
    set_source_files_properties(event_driven.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(event_driven.i PROPERTIES SWIG_FLAGS "-includeall")
    set_property(SOURCE event_driven.i PROPERTY CPLUSPLUS ON)
    swig_add_library(event_driven LANGUAGE python SOURCES event_driven.i)
    swig_link_libraries(event_driven ${PYTHON_LIBRARIES} ${YARP_LIBRARIES} ${EVENTDRIVEN_LIBRARIES})
    
endif(BUILD_BINDINGS)

