cmake_minimum_required(VERSION 3.20)
project(Project4)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# macOS Homebrew: cmake -DCMAKE_PREFIX_PATH=/usr/local/opt/opencv ..
add_executable(calibration  main_calibration.cpp)
add_executable(ar_viewer    main_ar.cpp)
add_executable(features     main_features.cpp)
add_executable(static_ar    main_static.cpp)

target_link_libraries(calibration  ${OpenCV_LIBS})
target_link_libraries(ar_viewer    ${OpenCV_LIBS})
target_link_libraries(features     ${OpenCV_LIBS})
target_link_libraries(static_ar    ${OpenCV_LIBS})