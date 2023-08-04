cmake_minimum_required(VERSION 3.14)

project(gorgon 
    VERSION 0.7
    DESCRIPTION "Library for mutating incoming data for robustness testing"
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 14)

option(BUILD_TESTING "Build GTEST unit tests for enabled mutators" OFF)
option(BUILD_DOCS "Build Doxygen HTML for all mutators" OFF)


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


# The compiled library code is here
add_subdirectory(src)

# Testing only available if this is the main app
# Emergency override MODERN_CMAKE_BUILD_TESTING provided as well
if( BUILD_TESTING)
    
    include(CTest)
    add_subdirectory(tests)
endif()

# Installation Instructions
include(GNUInstallDirs)

install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} 
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" # source directory
        DESTINATION  ${CMAKE_INSTALL_INCLUDEDIR} #target directory
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

if(BUILD_DOCS) 
    find_package(Doxygen)

    if(Doxygen_FOUND)
        add_subdirectory(docs)
    else()
        message(STATUS "Doxygen not found, not building docs")
    endif()

endif()
