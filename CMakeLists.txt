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
    bobbycar-protocol/bobbycar-can.h
    bobbycar-protocol/bobbycar-common.h
    bobbycar-protocol/bobbycar-serial.h
)




#
# motor test
#
add_executable(motortest.elf config.h defines.h main.cpp)
target_link_libraries(motortest.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(motortest.elf PRIVATE
    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
#    -DGSCHISSENES_HALL
#    -DGSCHISSENE_PWM_FREQ
#    -DFEATURE_BUTTON
#    -DPETERS_PLATINE
#    -DHUARN2
#    -DHUARN3
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
# feedc0de front
#
add_executable(feedcode-front.elf config.h defines.h main.cpp)
target_link_libraries(feedcode-front.elf stm32_hal emanuel_foc_model bobbycar-protocol)
target_compile_options(feedcode-front.elf PRIVATE
#    -DMOTOR_TEST
    -DFEATURE_IGNORE_OTHER_MOTOR
    -DGSCHISSENES_HALL
#    -DGSCHISSENE_PWM_FREQ
#    -DFEATURE_BUTTON
    -DPETERS_PLATINE
#    -DHUARN2
#    -DHUARN3
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
#    -DGSCHISSENES_HALL
#    -DGSCHISSENE_PWM_FREQ
#    -DFEATURE_BUTTON
    -DPETERS_PLATINE
#    -DHUARN2
#    -DHUARN3
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





# util targets
add_custom_target(debug
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg)
add_custom_target(unlock0
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0")
add_custom_target(unlock1
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" -c "mww 0x40022010 0x2220" -c "sleep 100" -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" -c targets -c "halt" -c "stm32f1x unlock 0")
add_custom_target(unlock2
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c targets -c "halt" -c "stm32f1x unlock 0")
