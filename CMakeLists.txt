cmake_minimum_required (VERSION 2.6)

project(bobbycar-controller-firmware ASM C CXX)

set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-gcc)

set(COMMON_FLAGS "-mcpu=cortex-m3 -mthumb -Wall -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${COMMON_FLAGS} -x assembler-with-cpp")
set(CMAKE_C_FLAGS "${COMMON_FLAGS} -std=gnu11")
set(CMAKE_CXX_FLAGS "${COMMON_FLAGS} -std=c++17")

set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXE_LINKER_FLAGS "-specs=nosys.specs -T${CMAKE_SOURCE_DIR}/STM32F103RCTx_FLASH.ld -lc -lm -lnosys -lstdc++ -Wl,--gc-sections -Wl,-Map=${CMAKE_BINARY_DIR}/hover.map,--cref")

set(CMAKE_SHARED_LIBRARY_LINK_ASM_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS)

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

add_executable(firmware.elf
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

    bobbycar-foc-model/BLDC_controller.h
    bobbycar-foc-model/BLDC_controller.c
    bobbycar-foc-model/BLDC_controller_data.c
    bobbycar-foc-model/rtwtypes.h

    bobbycar-protocol/protocol.h

    startup_stm32f103xe.s
    system_stm32f1xx.c

    config.h
    defines.h
    main.cpp
)

add_custom_command(OUTPUT firmware.hex
                   COMMAND arm-none-eabi-objcopy -O ihex firmware.elf firmware.hex
                   DEPENDS firmware.elf)

add_custom_command(OUTPUT firmware.bin
                   COMMAND arm-none-eabi-objcopy -O binary -S firmware.elf firmware.bin
                   DEPENDS firmware.elf)

add_custom_target(firmware ALL
                  SOURCES firmware.hex firmware.bin)

add_custom_target(flash
                  COMMAND st-flash --reset write firmware.bin 0x8000000
                  SOURCES firmware.bin)

add_custom_target(debug
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg)

add_custom_target(unlock0
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "stm32f1x unlock 0")
add_custom_target(unlock1
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c "mww 0x40022010 0x220" -c "mww 0x40022010 0x260" -c "sleep 100" -c "mww 0x40022010 0x230" -c "mwh 0x1ffff800 0x5AA5" -c "sleep 1000" -c "mww 0x40022010 0x2220" -c "sleep 100" -c "mdw 0x40022010" -c "mdw 0x4002201c" -c "mdw 0x1ffff800" -c targets -c "halt" -c "stm32f1x unlock 0")
add_custom_target(unlock2
                  COMMAND openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c init -c "reset halt" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xCDEF89AB" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xCDEF89AB" -c targets -c "halt" -c "stm32f1x unlock 0")
