cmake_minimum_required(VERSION 3.11)
project(rosbag)

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

set(ROS_MSG_PARSER_PATH "../../../ros_msg_parser/build/devel/share/ros_msg_parser/cmake/")
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

find_package(catkin REQUIRED COMPONENTS rosbag_storage rosconsole roscpp std_srvs topic_tools xmlrpcpp)
find_package(Boost REQUIRED COMPONENTS date_time regex program_options filesystem)
find_package(BZip2 REQUIRED)
find_package(ros_msg_parser REQUIRED PATHS ${ROS_MSG_PARSER_PATH})

find_package(gorgon 
            NAMES gorgon   
            HINTS "../../../gorgon-mutations-lib/lib/"
)
find_package(gorgon_ros 
            NAMES gorgon_ros   
            HINTS "../../../gorgon-ros/lib/"
)

catkin_python_setup()

# Support large bags (>2GB) on 32-bit systems
add_definitions(-D_FILE_OFFSET_BITS=64)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
  ${BZIP2_INCLUDE_DIR}
)

catkin_package(
  LIBRARIES rosbag
  INCLUDE_DIRS include
  CATKIN_DEPENDS rosbag_storage rosconsole roscpp std_srvs topic_tools xmlrpcpp)

add_library(rosbag
  src/player.cpp
  src/recorder.cpp
  src/time_translator.cpp)

add_library(rosbag_mutation
  src/player.cpp
  src/mutation_player.cpp
  src/recorder.cpp
  src/time_translator.cpp)


target_link_libraries(rosbag ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${BZIP2_LIBRARIES} ${ros_msg_parser_INCLUDE_DIRS} 
)
 
target_link_libraries(rosbag_mutation 
  "-Wl,--disable-new-dtags" # disable RunPath to use RPath for linking (https://stackoverflow.com/a/60126535)
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
  ${BZIP2_LIBRARIES} ${ros_msg_parser_LIBRARIES} ${nlohmann_json_TARGET} gorgon::gorgon 
  gorgon_ros::gorgon_ros
)

add_executable(record src/record.cpp)
target_link_libraries(record rosbag)

add_executable(play src/play.cpp)
target_link_libraries(play rosbag_mutation)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h")
install(TARGETS rosbag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS record play
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/bag2png.py
  scripts/bagsort.py
  scripts/fastrebag.py
  scripts/fixbag.py
  scripts/fixbag_batch.py
  scripts/fix_md5sums.py
  scripts/fix_moved_messages.py
  scripts/fix_msg_defs.py
  scripts/makerule.py
  scripts/savemsg.py
  scripts/topic_renamer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test)
endif()
