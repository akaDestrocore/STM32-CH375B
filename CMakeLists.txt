cmake_minimum_required(VERSION 3.22)

include("cmake/gcc-arm-none-eabi.cmake")

# Define project
project(HID_Device_Modification)
enable_language(C CXX ASM)

#############################################################
#### GENERAL SETTINGS
#############################################################
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# MCU settings
set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F407xx)

# Define startup and linker files
set(APP_STARTUP_FILE "${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f407xx.s")
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F407VGTx_FLASH.ld")

# CPU parameters
set(cpu_PARAMS 
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Compiler flags for all targets
set(COMMON_FLAGS
    ${cpu_PARAMS}
    -Wall
    -ffunction-sections
    -fdata-sections
    $<$<CONFIG:Debug>:-O0 -g -gdwarf-2>
    $<$<CONFIG:Release>:-Os -g0>
)

# Linker flags for the target
set(COMMON_LINKER_FLAGS
    ${cpu_PARAMS}
    -Wl,--print-memory-usage
    -Wl,--gc-sections
    -Wl,--cref
)

# Include directories
set(COMMON_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/${MCU_FAMILY}/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/include
)

#############################################################
#### SOURCES
#############################################################
# Core project sources
set(PROJECT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/auto_gun_press.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/log.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32f4xx.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c
)

# CH375 driver sources
set(CH375_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/src/ch375_usbhost.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/src/ch375.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/src/hid/usbhid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/src/hid/hid_keyboard.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/src/hid/hid_mouse.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ch375/src/hid/hid.c
)
list(APPEND PROJECT_SOURCES ${CH375_SOURCES})

# HAL driver sources
set(HAL_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/${MCU_FAMILY}_HAL_Driver/Src/stm32f4xx_hal_uart.c
)
list(APPEND PROJECT_SOURCES ${HAL_SOURCES})

# USB Device sources
set(USB_DEVICE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/App/composite_hid.c
    ${CMAKE_CURRENT_SOURCE_DIR}/USB_DEVICE/Target/usbd_conf.c
)
list(APPEND PROJECT_SOURCES ${USB_DEVICE_SOURCES})

# USB Middleware sources
set(USB_MIDDLEWARE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/ST/STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c
)
list(APPEND PROJECT_SOURCES ${USB_MIDDLEWARE_SOURCES})

# Startup ASM
set(ASM_SOURCES
    ${APP_STARTUP_FILE}
)

#############################################################
#### TARGET
#############################################################
add_executable(${PROJECT_NAME}
    ${PROJECT_SOURCES}
    ${ASM_SOURCES}
)

# Includes and compile options
target_include_directories(${PROJECT_NAME} PRIVATE ${COMMON_INCLUDES})
target_compile_options(${PROJECT_NAME} PRIVATE ${COMMON_FLAGS})

# Compile-time definitions
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    ${MCU_MODEL}
    $<$<CONFIG:Debug>:DEBUG>
)

# Link options
target_link_options(${PROJECT_NAME} PRIVATE ${COMMON_LINKER_FLAGS})

# Link libraries (c and m only, nosys comes from --specs)
target_link_libraries(${PROJECT_NAME} PRIVATE
    c
    m
)

# Set the linker script and map
if(EXISTS ${LINKER_SCRIPT})
    set_target_properties(${PROJECT_NAME} PROPERTIES
        LINK_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${CMAKE_BINARY_DIR}/${PROJECT_NAME}.map"
    )
else()
    message(WARNING "Linker script ${LINKER_SCRIPT} not found. Please provide correct path.")
endif()

#############################################################
#### POST-BUILD COMMANDS
#############################################################
if(NOT DEFINED CMAKE_OBJCOPY)
    find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    if(NOT CMAKE_OBJCOPY)
        find_program(CMAKE_OBJCOPY objcopy)
    endif()
endif()

if(NOT DEFINED CMAKE_SIZE)
    find_program(CMAKE_SIZE arm-none-eabi-size)
    if(NOT CMAKE_SIZE)
        find_program(CMAKE_SIZE size)
    endif()
endif()

if(NOT CMAKE_OBJCOPY)
    message(FATAL_ERROR "objcopy not found. Ensure arm-none-eabi-objcopy (or objcopy) is in PATH or set CMAKE_OBJCOPY in toolchain file.")
endif()

# Generate .bin and .hex and print size
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary -S $<TARGET_FILE:${PROJECT_NAME}> ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Generating ${PROJECT_NAME}.hex/.bin and printing size"
)

# Flash target
add_custom_target(flash
    COMMAND openocd -f interface/stlink-v2-1.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin 0x08000000 bin" -c "verify_image ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.bin 0x08000000 bin" -c "reset run" -c shutdown
    DEPENDS ${PROJECT_NAME}
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Flashing ${PROJECT_NAME}.bin via openocd"
)