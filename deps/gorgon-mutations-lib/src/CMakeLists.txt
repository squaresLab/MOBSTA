set(HEADER_LIST "${gorgon_SOURCE_DIR}/include/gorgon/utils.h")

set(SRC_LIST    "utils.cpp")

add_subdirectory(mutators)

# Make an automatic library - will be shared based on user setting
add_library(gorgon SHARED ${SRC_LIST} ${HEADER_LIST} $<TARGET_OBJECTS:gorgonMutators>)

target_link_libraries(gorgon PRIVATE ${nlohmann_json_TARGET} )

install(TARGETS gorgon EXPORT ${nlohmann_json_TARGET})

# We need this directory, and users of our library will need it too
target_include_directories(gorgon PUBLIC 
    "$<BUILD_INTERFACE:${gorgon_SOURCE_DIR}/include>"  
    "$<INSTALL_INTERFACE:include>")

# All users of this library will need at least C++14
target_compile_features(gorgon PUBLIC cxx_std_14)

set_target_properties(gorgon PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 0)


# IDEs should put the headers in a nice place
source_group(
  TREE "${PROJECT_SOURCE_DIR}/include"
  PREFIX "Header Files"
  FILES ${HEADER_LIST})

