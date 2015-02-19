cmake_minimum_required(VERSION 2.8)
project( transforms )
find_package( OpenCV REQUIRED )


add_executable( transforms htransformp.cpp )
target_link_libraries( transforms ${OpenCV_LIBS} )

add_executable( findContours findContours.cpp )
target_link_libraries( findContours ${OpenCV_LIBS} )

add_executable( thresh thresh.cpp )
target_link_libraries( thresh ${OpenCV_LIBS} )

add_executable( canny canny.cpp )
target_link_libraries( canny ${OpenCV_LIBS} )
