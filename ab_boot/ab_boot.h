#pragma once

#include <stdint.h>

#define FLASH_START  0x08000000
#define AB_BOOT_SIZE 0x00002000

#define APP_A_START  0x08002000
#define APP_B_START  0x08020800
                     // 122 KiB
#define APP_SIZE        0x1e800

#define CONFIG_START 0x0803f800
#define CONFIG_SIZE       0x800

#define FLASH_END    0x08040000

struct ab_boot_config
{
    uint32_t *boot_partition;
};
