#  pcd reader; used to display a piece of pointcloud
# add_executable(bev bev_test.cc)
# target_link_libraries(bev ${LINK_LIBS})

add_executable(corner_detection corner_detection.cc)
target_link_libraries(corner_detection ${LINK_LIBS})
