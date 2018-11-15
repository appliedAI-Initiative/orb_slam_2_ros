set (mt_task_queue_SOURCE_DIR "${PROJECT_SOURCE_DIR}/orb_slam2/Thirdparty/mt_task_queue")

SET(mt_task_queue_LIBRARY_OUTPUT_DIRECTORY ${mt_task_queue_SOURCE_DIR}/lib CACHE PATH "Target for the libraries")

# Set up the top-level include directories
INCLUDE_DIRECTORIES(
${mt_task_queue_SOURCE_DIR}/include/
)

# Include the subdirectories
ADD_LIBRARY(mt_task_queue SHARED
#src
${mt_task_queue_SOURCE_DIR}/src/Task.cpp
${mt_task_queue_SOURCE_DIR}/src/TaskQueue.cpp
${mt_task_queue_SOURCE_DIR}/src/Worker.cpp
#include
${mt_task_queue_SOURCE_DIR}/include/Task.h
${mt_task_queue_SOURCE_DIR}/include/TaskQueue.h
${mt_task_queue_SOURCE_DIR}/include/Worker.h
)
