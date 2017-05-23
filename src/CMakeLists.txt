set(PROJ_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set(PROJ_SRCS
  ${PROJ_PATH}/main.cpp
  ${PROJ_PATH}/cli.cpp
  ${PROJ_PATH}/pcbnAPI.cpp
  ${PROJ_PATH}/util.cpp
  )

set(PROJ_HDRS
  ${PROJ_PATH}/
  )

add_subdirectory(PID)
add_subdirectory(Drivers)
add_subdirectory(bsmp)

set(PROJ_SRCS ${PROJ_SRCS} ${PROJ_SRCS} PARENT_SCOPE)
set(PROJ_HDRS ${PROJ_HDRS} ${PROJ_HDRS} PARENT_SCOPE)
