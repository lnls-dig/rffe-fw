CMAKE_MINIMUM_REQUIRED(VERSION 2.8.10)

# ------------------------------------------------------------------------------
# setup processor settings add aditional boards here
#  LPC1768, LPC11U24, NRF51822, K64F

# TARGET -> has to be set in CMakeLists.txt
#
# MBED_VENDOR -> CPU Manufacturer
#
message(STATUS "building for ${MBED_TARGET}")
# the settings for mbed is really messed up ;)
if(MBED_TARGET MATCHES "LPC1768")
  set(MBED_VENDOR "NXP")
  set(MBED_FAMILY "LPC176X")
  set(MBED_CPU "MBED_LPC1768")
  set(MBED_CORE "cortex-m3")
  set(MBED_ARCH "armv7-m")
  set(MBED_INSTRUCTIONSET "M3")

  set(MBED_STARTUP "startup_LPC17xx.o")
  set(MBED_SYSTEM "system_LPC17xx.o")
  set(MBED_LINK_TARGET ${MBED_TARGET})

elseif(MBED_TARGET MATCHES "LPC11U24")
  set(MBED_VENDOR "NXP")
  set(MBED_FAMILY "LPC11UXX")
  set(MBED_CPU "LPC11U24_401")
  set(MBED_CORE "cortex-m0")
  set(MBED_INSTRUCTIONSET "M0")

  set(MBED_STARTUP "startup_LPC11xx.o")
  set(MBED_SYSTEM "system_LPC11Uxx.o")
  set(MBED_LINK_TARGET ${MBED_TARGET})

elseif(MBED_TARGET MATCHES "RBLAB_NRF51822")
  set(MBED_VENDOR "NORDIC")
  set(MBED_FAMILY "MCU_NRF51822")
  set(MBED_CPU "RBLAB_NRF51822")
  set(MBED_CORE "cortex-m0")
  set(MBED_INSTRUCTIONSET "M0")

  set(MBED_STARTUP "startup_NRF51822.o")
  set(MBED_SYSTEM "system_nrf51822.o")
  set(MBED_LINK_TARGET "NRF51822")

else()
   message(FATAL_ERROR "No MBED_TARGET specified or available. Full stop :(")
endif()

if(USE_NET)
  list(APPEND MBED_BUILD_FLAGS "--eth" )
  include_directories("${MBED_PATH}/build/net/eth/")
  include_directories("${MBED_PATH}/build/net/eth/lwip/")
  include_directories("${MBED_PATH}/build/net/eth/lwip-sys/")
  include_directories("${MBED_PATH}/build/net/eth/lwip/include/")
  include_directories("${MBED_PATH}/build/net/eth/lwip/include/ipv4/")
  include_directories("${MBED_PATH}/build/net/eth/EthernetInterface/")
  include_directories("${MBED_PATH}/build/net/eth/Socket")
  include_directories("${MBED_PATH}/build/net/eth/lwip-eth/arch/TARGET_${MBED_VENDOR}/")
  set(MBED_LIBS ${MBED_LIBS} eth)
endif()
if(USE_USB)
  list(APPEND MBED_BUILD_FLAGS "--usb" )
endif()
if(USE_USBHOST)
  list(APPEND MBED_BUILD_FLAGS "--usb_host" )
  include_directories("${MBED_PATH}/build/usb_host/USBHost/")
  include_directories("${MBED_PATH}/build/usb_host/USBHostHub/")
  include_directories("${MBED_PATH}/build/usb_host/USBHostSerial/")
  set(MBED_LIBS ${MBED_LIBS} USBHost)
endif()
if(USE_DSP)
  list(APPEND MBED_BUILD_FLAGS "--dsp  " )
endif()
if(USE_UBLOX)
  list(APPEND MBED_BUILD_FLAGS "--ublox  " )
endif()
if(USE_RTOS)
  list(APPEND MBED_BUILD_FLAGS "--rtos" )
  include_directories("${MBED_PATH}/build/rtos/")
  include_directories("${MBED_PATH}/build/rtos/rtos/")
  #TODO: Hardcoded folder for cortex_m port
  include_directories("${MBED_PATH}/build/rtos/TARGET_CORTEX_M/")
  include_directories("${MBED_PATH}/build/rtos/rtx/TARGET_CORTEX_M/")
  include_directories("${MBED_PATH}/build/rtos/rtx/TARGET_CORTEX_M/TARGET_${MBED_INSTRUCTIONSET}")
  set(MBED_LIBS ${MBED_LIBS} rtos rtx)
endif()

# build mbed libs
execute_process(COMMAND python ./build.py -m ${MBED_TARGET} -t ${TOOLCHAIN} ${MBED_BUILD_FLAGS}
  WORKING_DIRECTORY ${MBED_PATH}/workspace_tools
  )

# ------------------------------------------------------------------------------
# compiler settings
SET(COMMON_FLAGS "${COMMON_FLAGS} -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers -fno-builtin -MMD")
SET(COMMON_FLAGS "${COMMON_FLAGS} -mcpu=cortex-m3 -mthumb -march=${MBED_ARCH} -Os -fno-exceptions -msoft-float -ffunction-sections -fdata-sections -fno-common -fmessage-length=0 -fomit-frame-pointer -MMD -MP")

SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_TARGET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_INSTRUCTIONSET}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTARGET_${MBED_VENDOR}")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC_ARM")
SET(MBED_DEFINES "${MBED_DEFINES} -DTOOLCHAIN_GCC")

SET(CMAKE_CXX_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu++0x")
SET(CMAKE_C_FLAGS "${COMMON_FLAGS} ${MBED_DEFINES} -std=gnu99")

# ------------------------------------------------------------------------------
# setup precompiled mbed files which will be needed for all projects
set(MBED_OBJECTS
  ${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}/${MBED_STARTUP}
  ${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}/${MBED_SYSTEM}
  ${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}/cmsis_nvic.o
  ${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}/retarget.o
  ${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}/board.o
)

# ------------------------------------------------------------------------------
# libraries for mbed
set(MBED_LIBS mbed stdc++ supc++ m gcc g c nosys ${MBED_LIBS})

# ------------------------------------------------------------------------------
# linker settings
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--gc-sections -Wl,--wrap,main --specs=nano.specs -u _printf_float -u _scanf_float -Wl,-Map=${CMAKE_PROJECT_NAME}.map,--cref")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} \"-T${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}/${MBED_LINK_TARGET}.ld\"")
# ------------------------------------------------------------------------------
# mbed
include_directories("${MBED_PATH}/build/mbed/")
include_directories("${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/")
include_directories("${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/${TOOLCHAIN}")
include_directories("${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/")
include_directories("${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TARGET_${MBED_VENDOR}/TARGET_${MBED_FAMILY}/TARGET_${MBED_CPU}")

link_directories("${MBED_PATH}/build/mbed/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}")
link_directories("${MBED_PATH}/build/rtos/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}")
link_directories("${MBED_PATH}/build/usb_host/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}")
link_directories("${MBED_PATH}/build/net/eth/TARGET_${MBED_TARGET}/TOOLCHAIN_${TOOLCHAIN}")

