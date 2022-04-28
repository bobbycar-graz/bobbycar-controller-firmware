#include <stdint.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"
#include "ab_boot.h"
#include "persist/persist.h"

static inline void __attribute__((noreturn)) boot_image(uint32_t sp, uint32_t entry) {
    asm volatile(
        "   mov sp, %0    \n"
        "   blx  %1       \n"
        "1: b   1b        \n"
        :
        : "r" (sp), "r" (entry)
    );
    __builtin_unreachable();
}

static bool is_valid_boot_address(uint32_t *bootp)
{
    return (uint32_t)bootp >= FLASH_START &&
        (uint32_t)bootp < FLASH_END &&
        ((uint32_t)bootp & 3) == 0;
}

int main()
{
    struct ab_boot_config *flash_config = (struct ab_boot_config *)CONFIG_START;
    struct ab_boot_config *config = NULL;
    if (is_persist_valid() &&
        is_valid_boot_address(persist.ab_boot_config.boot_partition))
    {
        // Invalidate persist
        persist.checksum = 0;
        config = &persist.ab_boot_config;
    }
    else if (is_valid_boot_address(flash_config->boot_partition))
    {
        config = flash_config;
    }

    uint32_t *bootp;
    if (config)
        bootp = config->boot_partition;
    else
        bootp = (uint32_t *)APP_A_START;

    uint32_t sp = bootp[0];
    uint32_t entry = bootp[1];
    boot_image(sp, entry);
}
