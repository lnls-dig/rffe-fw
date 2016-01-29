cmake_minimum_required (VERSION 2.8)

project (rffe-tests)

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")

include(toolchain/arm_none_eabi_toolchain.cmake)

set(TOOLCHAIN "GCC_ARM" CACHE STRING "Toolchain used to build project")

set(MBED_PATH mbed-lib)
set(MBED_MOUNT /dev/mbed/)

set(MBED_TARGET LPC1768)
set(USE_RTOS true )
set(USE_NET true )
set(USE_USBHOST true )
set(USE_NET true )

include(mbed.cmake)

#Add all project sources to be compiled
set(PROJ_SRCS main.cpp pcbnAPI.cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_subdirectory(PID)
add_subdirectory(Drivers)
add_subdirectory(bsmp)

include_directories(${PROJ_HDRS})
add_executable(${CMAKE_PROJECT_NAME} ${PROJ_SRCS} ${MBED_OBJECTS})

target_link_libraries(${CMAKE_PROJECT_NAME} ${MBED_LIBS} )

set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES
  SUFFIX ".axf"
  )
# ------------------------------------------------------------------------------
# custom target for copying to mbed device
add_custom_target(upload
  ${CMAKE_OBJCOPY} -O binary ${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}.bin
  COMMAND cp ${CMAKE_PROJECT_NAME}.bin ${MBED_MOUNT}
)

# ------------------------------------------------------------------------------
# custom target for opening serial console
add_custom_target(sercon
  command screen ${SERCON} 9600
)

