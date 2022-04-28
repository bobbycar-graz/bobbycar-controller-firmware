cmake_minimum_required (VERSION 3.5)

set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-gcc)

SET(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CROSSCOMPILING 1)
SET(CMAKE_SYSTEM_NAME Generic)

set(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp")
set(CMAKE_C_FLAGS "${COMMON_FLAGS}")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS}")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

set(A_FIRMWARE_LINKER_FLAGS "-T${CMAKE_SOURCE_DIR}/STM32F103RCTx_FLASH_a.ld")
set(B_FIRMWARE_LINKER_FLAGS "-T${CMAKE_SOURCE_DIR}/STM32F103RCTx_FLASH_b.ld")
set(AB_BOOT_FIRMWARE_LINKER_FLAGS "-T${CMAKE_SOURCE_DIR}/ab_boot/STM32F103RCTx_FLASH_boot.ld")
set(COMMON_LINKER_FLAGS -specs=nosys.specs -lc -lm -lnosys -lstdc++ -Wl,--gc-sections "-Wl,-Map=${CMAKE_BINARY_DIR}/hover.map,--cref")

set(A_FIRMWARE_APP_BASE 0x08002000)
set(B_FIRMWARE_APP_BASE 0x08020800)

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(bobbycar-controller-firmware ASM C CXX)

add_definitions(-DUSE_HAL_DRIVER -DSTM32F103xE)

include_directories(
    .
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Inc
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy
    STM32CubeF1/Drivers/CMSIS/Device/ST/STM32F1xx/Include
    STM32CubeF1/Drivers/CMSIS/Include
    bobbycar-foc-model
    bobbycar-protocol
)

add_library(stm32_hal STATIC
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_pwr.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio_ex.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_flash_ex.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
    STM32CubeF1/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_can.c
    startup_stm32f103xe.s
)

add_library(emanuel_foc_model STATIC
    bobbycar-foc-model/BLDC_controller.h
    bobbycar-foc-model/BLDC_controller.c
    bobbycar-foc-model/BLDC_controller_data.c
    bobbycar-foc-model/rtwtypes.h
)

add_library(bobbycar-protocol INTERFACE
)


set(COMMON_SOURCES
    config.h
    defines.h
    main.cpp
    system_stm32f1xx.c
)

#
# motor test
#
add_executable(motortest.elf ${COMMON_SOURCES})
target_link_options(motortest.elf PRIVATE ${COMMON_LINKER_FLAGS} ${A_FIRMWARE_LINKER_FLAGS})
target_link_libraries(motortest.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(motortest.elf PRIVATE
    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
#    -DHALL_BCA
#    -DPWM_FREQ_12KHZ
#    -DFEATURE_BUTTON
#    -DPETERS_PLATINE
#    -DHUART2
#    -DHUART3
#    -DFEATURE_SERIAL_CONTROL
#    -DFEATURE_SERIAL_FEEDBACK
#    -DLOG_TO_SERIAL
#    -DFEATURE_CAN
#    -DCAN_LOG_UNKNOWN_ADDR
#    -DIS_BACK

    -DVECT_TAB_OFFSET=${A_FIRMWARE_APP_BASE}
)
add_custom_command(OUTPUT motortest.hex COMMAND arm-none-eabi-objcopy -O ihex motortest.elf motortest.hex DEPENDS motortest.elf)
add_custom_command(OUTPUT motortest.bin COMMAND arm-none-eabi-objcopy -O binary -S motortest.elf motortest.bin DEPENDS motortest.elf)
add_custom_target(motortest ALL SOURCES motortest.hex motortest.bin)
add_custom_target(flash-motortest COMMAND st-flash --reset write motortest.bin ${A_FIRMWARE_APP_BASE} SOURCES motortest.bin DEPENDS motortest.bin)

#
# motor test peter
#
add_executable(motortest_peter.elf ${COMMON_SOURCES})
target_link_options(motortest_peter.elf PRIVATE ${COMMON_LINKER_FLAGS} ${A_FIRMWARE_LINKER_FLAGS})
target_link_libraries(motortest_peter.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(motortest_peter.elf PRIVATE
    #-DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
    -DHALL_ABC
    #    -DPWM_FREQ_12KHZ
    #    -DFEATURE_BUTTON
    -DPETERS_PLATINE
    #     -DFEATURE_INVERT_HALL
    #    -DHUART2
    #    -DHUART3
    #    -DFEATURE_SERIAL_CONTROL
    #    -DFEATURE_SERIAL_FEEDBACK
    #    -DLOG_TO_SERIAL
        -DFEATURE_CAN
    #    -DCAN_LOG_UNKNOWN_ADDR
        -DIS_BACK

    -DVECT_TAB_OFFSET=${A_FIRMWARE_APP_BASE}
)
add_custom_command(OUTPUT motortest_peter.hex COMMAND arm-none-eabi-objcopy -O ihex motortest_peter.elf motortest_peter.hex DEPENDS motortest_peter.elf)
add_custom_command(OUTPUT motortest_peter.bin COMMAND arm-none-eabi-objcopy -O binary -S motortest_peter.elf motortest_peter.bin DEPENDS motortest_peter.elf)
add_custom_target(motortest_peter ALL SOURCES motortest_peter.hex motortest_peter.bin)
add_custom_target(flash-motortest_peter COMMAND st-flash --reset write motortest_peter.bin ${A_FIRMWARE_APP_BASE} SOURCES motortest_peter.bin DEPENDS motortest_peter.bin)


#
# feedc0de front
#
add_executable(feedcode-front.elf ${COMMON_SOURCES})
target_link_options(feedcode-front.elf PRIVATE ${COMMON_LINKER_FLAGS} ${A_FIRMWARE_LINKER_FLAGS})
target_link_libraries(feedcode-front.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(feedcode-front.elf PRIVATE
#    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
    -DHALL_BCA
#    -DPWM_FREQ_12KHZ
#    -DFEATURE_BUTTON
    -DPETERS_PLATINE
#    -DHUART2
#    -DHUART3
#    -DFEATURE_SERIAL_CONTROL
#    -DFEATURE_SERIAL_FEEDBACK
#    -DLOG_TO_SERIAL
    -DFEATURE_CAN
#    -DCAN_LOG_UNKNOWN_ADDR
#    -DIS_BACK

    -DVECT_TAB_OFFSET=${A_FIRMWARE_APP_BASE}
)
add_custom_command(OUTPUT feedcode-front.hex COMMAND arm-none-eabi-objcopy -O ihex feedcode-front.elf feedcode-front.hex DEPENDS feedcode-front.elf)
add_custom_command(OUTPUT feedcode-front.bin COMMAND arm-none-eabi-objcopy -O binary -S feedcode-front.elf feedcode-front.bin DEPENDS feedcode-front.elf)
add_custom_target(feedcode-front ALL SOURCES feedcode-front.hex feedcode-front.bin)
add_custom_target(flash-feedcode-front COMMAND st-flash --reset write feedcode-front.bin ${A_FIRMWARE_APP_BASE} SOURCES feedcode-front.bin DEPENDS feedcode-front.bin)

#
# feedc0de back
#
add_executable(feedcode-back.elf ${COMMON_SOURCES})
target_link_options(feedcode-back.elf PRIVATE ${COMMON_LINKER_FLAGS} ${A_FIRMWARE_LINKER_FLAGS})
target_link_libraries(feedcode-back.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(feedcode-back.elf PRIVATE
#    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
#    -DHALL_BCA
#    -DPWM_FREQ_12KHZ
#    -DFEATURE_BUTTON
    -DPETERS_PLATINE
#    -DHUART2
#    -DHUART3
#    -DFEATURE_SERIAL_CONTROL
#    -DFEATURE_SERIAL_FEEDBACK
#    -DLOG_TO_SERIAL
    -DFEATURE_CAN
#    -DCAN_LOG_UNKNOWN_ADDR
    -DIS_BACK

    -DVECT_TAB_OFFSET=${A_FIRMWARE_APP_BASE}
)
add_custom_command(OUTPUT feedcode-back.hex COMMAND arm-none-eabi-objcopy -O ihex feedcode-back.elf feedcode-back.hex DEPENDS feedcode-back.elf)
add_custom_command(OUTPUT feedcode-back.bin COMMAND arm-none-eabi-objcopy -O binary -S feedcode-back.elf feedcode-back.bin DEPENDS feedcode-back.elf)
add_custom_target(feedcode-back ALL SOURCES feedcode-back.hex feedcode-back.bin)
add_custom_target(flash-feedcode-back COMMAND st-flash --reset write feedcode-back.bin ${A_FIRMWARE_APP_BASE} SOURCES feedcode-back.bin DEPENDS feedcode-back.bin)

#
# greyhash
#
add_executable(greyhash.elf ${COMMON_SOURCES})
target_link_options(greyhash.elf PRIVATE ${COMMON_LINKER_FLAGS} ${A_FIRMWARE_LINKER_FLAGS})
target_link_libraries(greyhash.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(greyhash.elf PRIVATE
#    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
#    -DHALL_BCA
    -DPWM_FREQ_12KHZ
#    -DFEATURE_BUTTON
    -DPETERS_PLATINE
#    -DHUART2
    -DHUART3
    -DFEATURE_SERIAL_CONTROL
    -DFEATURE_SERIAL_FEEDBACK
#    -DLOG_TO_SERIAL
#    -DFEATURE_CAN
#    -DCAN_LOG_UNKNOWN_ADDR
#    -DIS_BACK

    -DVECT_TAB_OFFSET=${A_FIRMWARE_APP_BASE}
)
add_custom_command(OUTPUT greyhash.hex COMMAND arm-none-eabi-objcopy -O ihex greyhash.elf greyhash.hex DEPENDS greyhash.elf)
add_custom_command(OUTPUT greyhash.bin COMMAND arm-none-eabi-objcopy -O binary -S greyhash.elf greyhash.bin DEPENDS greyhash.elf)
add_custom_target(greyhash ALL SOURCES greyhash.hex greyhash.bin)
add_custom_target(flash-greyhash COMMAND st-flash --reset write greyhash.bin ${A_FIRMWARE_APP_BASE} SOURCES greyhash.bin DEPENDS greyhash.bin)


add_executable(ab_boot.elf ab_boot/ab_boot.c system_stm32f1xx.c)
target_link_options(ab_boot.elf PRIVATE ${COMMON_LINKER_FLAGS} ${AB_BOOT_FIRMWARE_LINKER_FLAGS})
target_link_libraries(ab_boot.elf stm32_hal)
target_compile_options(ab_boot.elf PRIVATE
    -DVECT_TAB_OFFSET=0x00000000
)
add_custom_command(OUTPUT ab_boot.hex COMMAND arm-none-eabi-objcopy -O ihex ab_boot.elf ab_boot.hex DEPENDS ab_boot.elf)
add_custom_command(OUTPUT ab_boot.bin COMMAND arm-none-eabi-objcopy -O binary -S ab_boot.elf ab_boot.bin DEPENDS ab_boot.elf)
add_custom_target(ab_boot ALL SOURCES ab_boot.hex ab_boot.bin)
add_custom_target(flash-boot COMMAND st-flash --reset write ab_boot.bin 0x08000000 SOURCES ab_boot.bin DEPENDS ab_boot.bin)


# util targets
add_custom_target(debug
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg)
add_custom_target(unlock0
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0")
add_custom_target(unlock1
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" -c "mww 0x40022010 0x2220" -c "sleep 100" -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" -c targets -c "halt" -c "stm32f1x unlock 0")
add_custom_target(unlock2
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c targets -c "halt" -c "stm32f1x unlock 0")
