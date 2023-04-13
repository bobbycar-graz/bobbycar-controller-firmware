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
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -Wno-volatile")

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 20)

set(COMMON_LINKER_FLAGS "-specs=nosys.specs -T${CMAKE_SOURCE_DIR}/STM32F103RCTx_FLASH.ld -lc -lm -lnosys -lstdc++ -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/hover.map,--cref")
set(CMAKE_EXE_LINKER_FLAGS "${COMMON_LINKER_FLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "${COMMON_LINKER_FLAGS}")

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
    system_stm32f1xx.c
)

add_library(emanuel_foc_model STATIC
    bobbycar-foc-model/BLDC_controller.h
    bobbycar-foc-model/BLDC_controller.c
    bobbycar-foc-model/BLDC_controller_data.c
    bobbycar-foc-model/rtwtypes.h
)

add_library(bobbycar-protocol INTERFACE
)




#
# motor test
#
add_executable(motortest.elf config.h defines.h main.cpp)
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
)
add_custom_command(OUTPUT motortest.hex COMMAND arm-none-eabi-objcopy -O ihex motortest.elf motortest.hex DEPENDS motortest.elf)
add_custom_command(OUTPUT motortest.bin COMMAND arm-none-eabi-objcopy -O binary -S motortest.elf motortest.bin DEPENDS motortest.elf)
add_custom_target(motortest ALL SOURCES motortest.hex motortest.bin)
add_custom_target(flash-motortest COMMAND st-flash --reset write motortest.bin 0x8000000 SOURCES motortest.bin DEPENDS motortest.bin)

#
# motor test peter
#
add_executable(motortest_peter.elf config.h defines.h main.cpp)
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
)
add_custom_command(OUTPUT motortest_peter.hex COMMAND arm-none-eabi-objcopy -O ihex motortest_peter.elf motortest_peter.hex DEPENDS motortest_peter.elf)
add_custom_command(OUTPUT motortest_peter.bin COMMAND arm-none-eabi-objcopy -O binary -S motortest_peter.elf motortest_peter.bin DEPENDS motortest_peter.elf)
add_custom_target(motortest_peter ALL SOURCES motortest_peter.hex motortest_peter.bin)
add_custom_target(flash-motortest_peter COMMAND st-flash --reset write motortest_peter.bin 0x8000000 SOURCES motortest_peter.bin DEPENDS motortest_peter.bin)


#
# feedc0de front
#
add_executable(feedcode-front.elf config.h defines.h main.cpp)
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
)
add_custom_command(OUTPUT feedcode-front.hex COMMAND arm-none-eabi-objcopy -O ihex feedcode-front.elf feedcode-front.hex DEPENDS feedcode-front.elf)
add_custom_command(OUTPUT feedcode-front.bin COMMAND arm-none-eabi-objcopy -O binary -S feedcode-front.elf feedcode-front.bin DEPENDS feedcode-front.elf)
add_custom_target(feedcode-front ALL SOURCES feedcode-front.hex feedcode-front.bin)
add_custom_target(flash-feedcode-front COMMAND st-flash --reset write feedcode-front.bin 0x8000000 SOURCES feedcode-front.bin DEPENDS feedcode-front.bin)

#
# feedc0de back
#
add_executable(feedcode-back.elf config.h defines.h main.cpp)
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
)
add_custom_command(OUTPUT feedcode-back.hex COMMAND arm-none-eabi-objcopy -O ihex feedcode-back.elf feedcode-back.hex DEPENDS feedcode-back.elf)
add_custom_command(OUTPUT feedcode-back.bin COMMAND arm-none-eabi-objcopy -O binary -S feedcode-back.elf feedcode-back.bin DEPENDS feedcode-back.elf)
add_custom_target(feedcode-back ALL SOURCES feedcode-back.hex feedcode-back.bin)
add_custom_target(flash-feedcode-back COMMAND st-flash --reset write feedcode-back.bin 0x8000000 SOURCES feedcode-back.bin DEPENDS feedcode-back.bin)


#
# feedc0de 2 front
#
add_executable(feedcode2-front.elf config.h defines.h main.cpp)
target_link_libraries(feedcode2-front.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(feedcode2-front.elf PRIVATE
#    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
    -DLEFT_HALL_ABC
    -DRIGHT_HALL_ABC
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
)
add_custom_command(OUTPUT feedcode2-front.hex COMMAND arm-none-eabi-objcopy -O ihex feedcode2-front.elf feedcode2-front.hex DEPENDS feedcode2-front.elf)
add_custom_command(OUTPUT feedcode2-front.bin COMMAND arm-none-eabi-objcopy -O binary -S feedcode2-front.elf feedcode2-front.bin DEPENDS feedcode2-front.elf)
add_custom_target(feedcode2-front ALL SOURCES feedcode2-front.hex feedcode2-front.bin)
add_custom_target(flash-feedcode2-front COMMAND st-flash --reset write feedcode2-front.bin 0x8000000 SOURCES feedcode2-front.bin DEPENDS feedcode2-front.bin)

#
# feedc0de 2 back
#
add_executable(feedcode2-back.elf config.h defines.h main.cpp)
target_link_libraries(feedcode2-back.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(feedcode2-back.elf PRIVATE
#    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
    -DLEFT_HALL_ACB
    -DRIGHT_HALL_ACB
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
)
add_custom_command(OUTPUT feedcode2-back.hex COMMAND arm-none-eabi-objcopy -O ihex feedcode2-back.elf feedcode2-back.hex DEPENDS feedcode2-back.elf)
add_custom_command(OUTPUT feedcode2-back.bin COMMAND arm-none-eabi-objcopy -O binary -S feedcode2-back.elf feedcode2-back.bin DEPENDS feedcode2-back.elf)
add_custom_target(feedcode2-back ALL SOURCES feedcode2-back.hex feedcode2-back.bin)
add_custom_target(flash-feedcode2-back COMMAND st-flash --reset write feedcode2-back.bin 0x8000000 SOURCES feedcode2-back.bin DEPENDS feedcode2-back.bin)

#
# greyhash
#
add_executable(greyhash.elf config.h defines.h main.cpp)
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
)
add_custom_command(OUTPUT greyhash.hex COMMAND arm-none-eabi-objcopy -O ihex greyhash.elf greyhash.hex DEPENDS greyhash.elf)
add_custom_command(OUTPUT greyhash.bin COMMAND arm-none-eabi-objcopy -O binary -S greyhash.elf greyhash.bin DEPENDS greyhash.elf)
add_custom_target(greyhash ALL SOURCES greyhash.hex greyhash.bin)
add_custom_target(flash-greyhash COMMAND st-flash --reset write greyhash.bin 0x8000000 SOURCES greyhash.bin DEPENDS greyhash.bin)

#
# peters platine v2 front
#
add_executable(pcbv2_front.elf config.h defines.h main.cpp)
target_link_libraries(pcbv2_front.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(pcbv2_front.elf PRIVATE
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
        #     -DIS_BACK
        -DAMPERE2BIT_CONV=28
)
add_custom_command(OUTPUT pcbv2_front.hex COMMAND arm-none-eabi-objcopy -O ihex pcbv2_front.elf pcbv2_front.hex DEPENDS pcbv2_front.elf)
add_custom_command(OUTPUT pcbv2_front.bin COMMAND arm-none-eabi-objcopy -O binary -S pcbv2_front.elf pcbv2_front.bin DEPENDS pcbv2_front.elf)
add_custom_target(pcbv2_front ALL SOURCES pcbv2_front.hex pcbv2_front.bin)
add_custom_target(flash-pcbv2_front COMMAND st-flash --reset write pcbv2_front.bin 0x8000000 SOURCES pcbv2_front.bin DEPENDS pcbv2_front.bin)

#
# peters platine v2 back
#
add_executable(pcbv2_back.elf config.h defines.h main.cpp)
target_link_libraries(pcbv2_back.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(pcbv2_back.elf PRIVATE
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
        -DAMPERE2BIT_CONV=28
)
add_custom_command(OUTPUT pcbv2_back.hex COMMAND arm-none-eabi-objcopy -O ihex pcbv2_back.elf pcbv2_back.hex DEPENDS pcbv2_back.elf)
add_custom_command(OUTPUT pcbv2_back.bin COMMAND arm-none-eabi-objcopy -O binary -S pcbv2_back.elf pcbv2_back.bin DEPENDS pcbv2_back.elf)
add_custom_target(pcbv2_back ALL SOURCES pcbv2_back.hex pcbv2_back.bin)
add_custom_target(flash-pcbv2_back COMMAND st-flash --reset write pcbv2_back.bin 0x8000000 SOURCES pcbv2_back.bin DEPENDS pcbv2_back.bin)

#
# peters platine v2 motortest
#
add_executable(pcbv2_test.elf config.h defines.h main.cpp)
target_link_libraries(pcbv2_test.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(pcbv2_test.elf PRIVATE
        -DMOTOR_TEST
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
        #    -DFEATURE_CAN
        #    -DCAN_LOG_UNKNOWN_ADDR
        #    -DIS_BACK
        -DAMPERE2BIT_CONV=28
        -DLEFT_HALL_ABC
        #-DLEFT_HALL_ACB
        #-DLEFT_HALL_BAC
        #-DLEFT_HALL_BCA
        #-DLEFT_HALL_CAB
        #-DLEFT_HALL_CBA

        -DRIGHT_HALL_ABC
        #-DRIGHT_HALL_ACB
        #-DRIGHT_HALL_BAC
        #-DRIGHT_HALL_BCA
        #-DRIGHT_HALL_CAB
        #-DRIGHT_HALL_CBA

        #-DFEATURE_BETTER_FOC_CONFIG
        #-DLEFT_PHASE_MEAS_AB
        #-DLEFT_PHASE_MEAS_AC
        #-DLEFT_PHASE_MEAS_BC

        #-DRIGHT_PHASE_MEAS_AB
        #-DRIGHT_PHASE_MEAS_AC
        #-DRIGHT_PHASE_MEAS_BC
)
add_custom_command(OUTPUT pcbv2_test.hex COMMAND arm-none-eabi-objcopy -O ihex pcbv2_test.elf pcbv2_test.hex DEPENDS pcbv2_test.elf)
add_custom_command(OUTPUT pcbv2_test.bin COMMAND arm-none-eabi-objcopy -O binary -S pcbv2_test.elf pcbv2_test.bin DEPENDS pcbv2_test.elf)
add_custom_target(pcbv2_test ALL SOURCES pcbv2_test.hex pcbv2_test.bin)
add_custom_target(flash-pcbv2_test COMMAND st-flash --reset write pcbv2_test.bin 0x8000000 SOURCES pcbv2_test.bin DEPENDS pcbv2_test.bin)


#
# commander v2 front
#
add_executable(commander_v2_front.elf config.h defines.h main.cpp)
target_link_libraries(commander_v2_front.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(commander_v2_front.elf PRIVATE
        #-DMOTOR_TEST
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
        #    -DIS_BACK
        #-DAMPERE2BIT_CONV=28
        -DLEFT_HALL_ABC
        #-DLEFT_HALL_ACB
        #-DLEFT_HALL_BAC
        #-DLEFT_HALL_BCA
        #-DLEFT_HALL_CAB
        #-DLEFT_HALL_CBA

        -DRIGHT_HALL_ABC
        #-DRIGHT_HALL_ACB
        #-DRIGHT_HALL_BAC
        #-DRIGHT_HALL_BCA
        #-DRIGHT_HALL_CAB
        #-DRIGHT_HALL_CBA

        #-DFEATURE_BETTER_FOC_CONFIG
        #-DLEFT_PHASE_MEAS_AB
        #-DLEFT_PHASE_MEAS_AC
        #-DLEFT_PHASE_MEAS_BC

        #-DRIGHT_PHASE_MEAS_AB
        #-DRIGHT_PHASE_MEAS_AC
        #-DRIGHT_PHASE_MEAS_BC
)
add_custom_command(OUTPUT commander_v2_front.hex COMMAND arm-none-eabi-objcopy -O ihex commander_v2_front.elf commander_v2_front.hex DEPENDS commander_v2_front.elf)
add_custom_command(OUTPUT commander_v2_front.bin COMMAND arm-none-eabi-objcopy -O binary -S commander_v2_front.elf commander_v2_front.bin DEPENDS commander_v2_front.elf)
add_custom_target(commander_v2_front ALL SOURCES commander_v2_front.hex commander_v2_front.bin)
add_custom_target(flash-commander_v2_front COMMAND st-flash --reset write commander_v2_front.bin 0x8000000 SOURCES commander_v2_front.bin DEPENDS commander_v2_front.bin)


#
# commander v2 back
#
add_executable(commander_v2_back.elf config.h defines.h main.cpp)
target_link_libraries(commander_v2_back.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(commander_v2_back.elf PRIVATE
        #-DMOTOR_TEST
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
        #-DAMPERE2BIT_CONV=28
        -DLEFT_HALL_ABC
        #-DLEFT_HALL_ACB
        #-DLEFT_HALL_BAC
        #-DLEFT_HALL_BCA
        #-DLEFT_HALL_CAB
        #-DLEFT_HALL_CBA

        -DRIGHT_HALL_ABC
        #-DRIGHT_HALL_ACB
        #-DRIGHT_HALL_BAC
        #-DRIGHT_HALL_BCA
        #-DRIGHT_HALL_CAB
        #-DRIGHT_HALL_CBA

        #-DFEATURE_BETTER_FOC_CONFIG
        #-DLEFT_PHASE_MEAS_AB
        #-DLEFT_PHASE_MEAS_AC
        #-DLEFT_PHASE_MEAS_BC

        #-DRIGHT_PHASE_MEAS_AB
        #-DRIGHT_PHASE_MEAS_AC
        #-DRIGHT_PHASE_MEAS_BC
)
add_custom_command(OUTPUT commander_v2_back.hex COMMAND arm-none-eabi-objcopy -O ihex commander_v2_back.elf commander_v2_back.hex DEPENDS commander_v2_back.elf)
add_custom_command(OUTPUT commander_v2_back.bin COMMAND arm-none-eabi-objcopy -O binary -S commander_v2_back.elf commander_v2_back.bin DEPENDS commander_v2_back.elf)
add_custom_target(commander_v2_back ALL SOURCES commander_v2_back.hex commander_v2_back.bin)
add_custom_target(flash-commander_v2_back COMMAND st-flash --reset write commander_v2_back.bin 0x8000000 SOURCES commander_v2_back.bin DEPENDS commander_v2_back.bin)


# util targets
add_custom_target(debug
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg)
add_custom_target(unlock0
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0")
add_custom_target(unlock1
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" -c "mww 0x40022010 0x2220" -c "sleep 100" -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" -c targets -c "halt" -c "stm32f1x unlock 0")
add_custom_target(unlock2
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c targets -c "halt" -c "stm32f1x unlock 0")
