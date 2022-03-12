#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "ab_boot.h"

int main()
{
    struct ab_boot_config *config = (struct ab_boot_config *)CONFIG_START;

    uint32_t *bootp;
    if (config->boot_partition >= FLASH_START &&
        config->boot_partition < FLASH_END &&
        (config->boot_partition & 3) == 0)
    {
        bootp = (uint32_t *)config->boot_partition;
    }
    else
    {
        bootp = (uint32_t *)APP_A_START;
    }

    uint32_t sp = bootp[0];
    uint32_t entry = bootp[1];
    asm volatile(
        "   mov sp, %0    \n"
        "   bx  %1        \n"
        "1: b   1b        \n"
        :
        : "r" (sp), "r" (entry)
    );
}
