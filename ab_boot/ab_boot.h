#pragma once

#include <stdint.h>

#define CONFIG_START 0x0803f800
#define FLASH_START  0x08000000
#define FLASH_END    0x08040000

#define APP_A_START  0x08002000
#define APP_B_START  0x08020800

struct ab_boot_config
{
    uint32_t boot_partition;
};
