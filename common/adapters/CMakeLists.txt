cmake_minimum_required(VERSION 2.8)
project(adapters)

set(${PROJECT_NAME}_include_dirs
	${PCL_INCLUDE_DIRS}
	${OpenCV_LIBRARIES}
	${Boost_INCLUDE_DIRS}
	${G2O_INCLUDE_DIRS}
	${CSPARSE_INCLUDE_DIR}
	)

set(${PROJECT_NAME}_lib
	${PCL_LIBRARIES}
	${Boost_LIBRARIES}
	${OpenCV_LIBRARIES}
	${G2O_LIBRARIES}
	${CSPARSE_LIBRARY}
	)

file(GLOB H_FILES *.h)
file(GLOB HPP_FILES *.hpp)
set(${PROJECT_NAME}_header_files ${H_FILES} ${HPP_FILES})

file(GLOB C_FILES *.c)
file(GLOB CPP_FILES *.cpp)
set(${PROJECT_NAME}_src_files ${C_FILES} ${CPP_FILES})
list(REMOVE_ITEM ${PROJECT_NAME}_src_files pcl_viewer_test.cpp)

include_directories(${${PROJECT_NAME}_include_dirs})
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_src_files} ${${PROJECT_NAME}_header_files})
target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_lib})

set_target_properties(${PROJECT_NAME} PROPERTIES
	PUBLIC_HEADER "${${PROJECT_NAME}_header_files}"
	)

install(TARGETS ${PROJECT_NAME} 
	RUNTIME DESTINATION bin
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	PUBLIC_HEADER DESTINATION include/common/adapters
	)
