
cmake_minimum_required(VERSION 3.10)
project(imgui_rosmon)

find_package(catkin REQUIRED COMPONENTS
    imgui_ros
    rosmon_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 17)

add_library(imgui_rosmon
    src/gui.cpp
)
target_link_libraries(imgui_rosmon
    ${catkin_LIBRARIES}
)
target_link_options(imgui_rosmon PRIVATE
    "-Wl,--no-undefined"
)

install(TARGETS imgui_rosmon
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(FILES imgui_plugins.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
