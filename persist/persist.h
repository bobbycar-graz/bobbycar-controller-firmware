#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "ab_boot/ab_boot.h"

struct persist_data {
    struct ab_boot_config ab_boot_config;
    uint32_t checksum;
};

extern struct persist_data persist;

static inline uint32_t calculate_persist_checksum() {
    uint32_t checksum = 0;
    uint32_t *pd = (uint32_t *)&persist;
    for (int i = 0; i < (sizeof(persist) - 4) / 4; i++) {
        checksum += pd[i];
        checksum = (checksum >> 3) | (checksum << 29);
    }

    checksum = ~checksum;

    return checksum;
}

static inline void update_persist_checksum() {
    persist.checksum = calculate_persist_checksum();
}

static inline bool is_persist_valid() {
    return calculate_persist_checksum() == persist.checksum;
}

static inline void request_boot_image(uint32_t *bootp) {
    persist.ab_boot_config.boot_partition = bootp;
    update_persist_checksum();
}
