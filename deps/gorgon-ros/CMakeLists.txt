cmake_minimum_required(VERSION 3.14)
project(gorgon_ros
        VERSION 0.1
        DESCRIPTION "Library for extending gorgon-mutations-lib into ROS"
        LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)


option(BUILD_TESTING "BUILD_TESTING" OFF)
option(BUILD_DOCS "BUILD_DOCS" OFF)


include(FetchContent)


# use a local installation rather than fetching one
if(DEFINED LOCAL_JSON_LIB)
    find_package(nlohmann_json 
            NAMES nlohmann_json
            HINTS "${LOCAL_JSON_LIB}"
    )

    set(nlohmann_json_TARGET nlohmann_json)
else()
    FetchContent_Declare(
        json
        URL
        https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz
    )    

    FetchContent_MakeAvailable(json)

    set(nlohmann_json_TARGET nlohmann_json::nlohmann_json)
endif()


find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})



find_package(gorgon 
            NAMES gorgon   
            PATHS "../gorgon-mutations-lib/lib/"
)


###########
## Build ##
###########
set(HEADER_LIST "${PROJECT_SOURCE_DIR}/include/gorgon_ros/mutators/ros_base.h"
                "${PROJECT_SOURCE_DIR}/include/gorgon_ros/mutators/point_msgs.h"
                "${PROJECT_SOURCE_DIR}/include/gorgon_ros/mutators/strings.h"
                "${PROJECT_SOURCE_DIR}/include/gorgon_ros/mutators/arrays.h"
                "${PROJECT_SOURCE_DIR}/include/gorgon_ros/utils.h"

)

set(SRC_LIST "${PROJECT_SOURCE_DIR}/src/mutators/point_msgs.cpp"
             "${PROJECT_SOURCE_DIR}/src/mutators/strings.cpp"
             "${PROJECT_SOURCE_DIR}/src/mutators/arrays.cpp"
             "${PROJECT_SOURCE_DIR}/src/utils.cpp"
)

add_library(${PROJECT_NAME} SHARED
    ${SRC_LIST}
    ${HEADER_LIST}
)
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(${PROJECT_NAME} PRIVATE
    ${roscpp_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${std_msgs_LIBRARIES}
    gorgon::gorgon
    ${nlohmann_json_TARGET}
)

install(TARGETS gorgon_ros EXPORT gorgon)

target_include_directories(${PROJECT_NAME} PUBLIC 
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"  
    "$<INSTALL_INTERFACE:include>")

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)

###########
## Docs  ##
###########

if(BUILD_DOCS) 

    # Docs only available if this is the main app
    find_package(Doxygen)

    if(Doxygen_FOUND)
        set(DOXYGEN_EXTRACT_ALL YES)
        set(DOXYGEN_BUILTIN_STL_SUPPORT YES)

        file(GLOB_RECURSE PROJECT_HEADERS
            "${PROJECT_SOURCE_DIR}/include/*.h"
        )

        doxygen_add_docs(docs 
            "${PROJECT_SOURCE_DIR}/docs/mainpage.md"
            "${PROJECT_HEADERS}"
        )
    else()
            message(STATUS "Doxygen not found, not building docs")
    endif()

endif()

#############
## Testing ##
#############

if(BUILD_TESTING)

    include(CTest)

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    FetchContent_MakeAvailable(googletest)

    enable_testing()

    add_executable(
        ros_mutators_test
        tests/ros_mutators_point_test.cpp
        tests/ros_mutators_string_test.cpp
        tests/ros_mutators_arrays_test.cpp
        tests/ros_utils_test.cpp
    )

    target_link_libraries(
        ros_mutators_test
        PRIVATE
        gorgon_ros
        gorgon::gorgon
        ${roscpp_LIBRARIES}
        ${geometry_msgs_LIBRARIES}
        ${std_msgs_LIBRARIES}
        GTest::gtest_main
        ${nlohmann_json_TARGET}
    )

    include(GoogleTest)
    gtest_discover_tests(ros_mutators_test
        EXTRA_ARGS "--gtest_repeat=64" "--rerun_failed" "--output-on-failure"
    )


endif()


#############
## Install ##
#############
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/"
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake 
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME} 
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

install(FILES "${PROJECT_NAME}Config.cmake" "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)
